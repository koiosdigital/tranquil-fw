# Server-Side Purchase Receipt Integration

This document describes how the cloud server should implement purchase receipt generation and synchronization with Tranquil devices.

## Overview

Purchase receipts allow users to permanently own specific patterns, independent of their subscription status. When a user purchases a pattern:

1. Server generates a signed `PurchaseReceipt` for that device
2. Receipt is delivered via WebSocket (sync or download response)
3. Device saves receipt to `/sd/patterns/{uuid}.licdat`
4. Device can play the pattern even if subscription expires

## Purchase Receipt Format

### Protobuf Message

```protobuf
message PurchaseReceipt {
    string pattern_uuid = 1;       // Pattern this receipt authorizes
    string for_device = 2;         // Must match device certificate CN
    int64 purchased_at = 3;        // Unix timestamp UTC
    string receipt_id = 4;         // Unique receipt ID for support
    string pattern_name = 5;       // Human-readable name (for diagnostics)
}
```

### Binary File Format (`.licdat`)

```
Offset  Size    Field
0x0000  4       magic = "KDPR" (0x5250444B, little-endian)
0x0004  2       version = 0x0001
0x0006  2       flags (reserved, set to 0)
0x0008  4       payload_len (protobuf size)
0x000C  4       signature_len (RSA signature size)
0x0010  var     payload (serialized PurchaseReceipt)
        var     signature (RSA-2048 signature over payload)
```

## Signature Generation

The server signs the serialized protobuf payload using RSA-2048 with SHA-256:

```python
import hashlib
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding

def sign_receipt(payload_bytes: bytes, private_key) -> bytes:
    """Sign a PurchaseReceipt payload with RSA-2048."""
    signature = private_key.sign(
        payload_bytes,
        padding.PKCS1v15(),
        hashes.SHA256()
    )
    return signature
```

The server must use the same private key that corresponds to the public key embedded in device firmware (`drm_license.cpp`).

## WebSocket Message Flows

### 1. Boot-Time Sync (SyncPurchasesRequest/Response)

When a device connects to the cloud WebSocket, it sends `SyncPurchasesRequest` to retrieve all purchases for that device.

**Request:** `SyncPurchasesRequest` (empty message)

**Response:**
```protobuf
message SyncPurchasesResponse {
    repeated PurchaseReceiptBundle receipts = 1;
}

message PurchaseReceiptBundle {
    string pattern_uuid = 1;
    bytes payload = 2;      // Serialized PurchaseReceipt
    bytes signature = 3;    // RSA signature over payload
}
```

**Server Implementation:**

```python
def handle_sync_purchases_request(device_cn: str) -> SyncPurchasesResponse:
    """Return all purchase receipts for a device."""
    # Query database for all purchases by this device
    purchases = db.query(
        "SELECT pattern_uuid, purchased_at, receipt_id, pattern_name "
        "FROM purchases WHERE device_cn = ?",
        device_cn
    )

    response = SyncPurchasesResponse()

    for purchase in purchases:
        receipt = PurchaseReceipt(
            pattern_uuid=purchase.pattern_uuid,
            for_device=device_cn,
            purchased_at=purchase.purchased_at,
            receipt_id=purchase.receipt_id,
            pattern_name=purchase.pattern_name
        )

        payload = receipt.SerializeToString()
        signature = sign_receipt(payload, SERVER_PRIVATE_KEY)

        bundle = PurchaseReceiptBundle(
            pattern_uuid=purchase.pattern_uuid,
            payload=payload,
            signature=signature
        )
        response.receipts.append(bundle)

    return response
```

### 2. Pattern Download (PatternDownloadResponse)

When a device requests a pattern download, include the purchase receipt if the pattern is purchased.

**Extended Response:**
```protobuf
message PatternDownloadResponse {
    string pattern_uuid = 3;
    string download_url = 4;
    PatternInfo pattern = 6;
    // Include if pattern is purchased by this device
    bytes purchase_receipt_payload = 7;
    bytes purchase_receipt_signature = 8;
}
```

**Server Implementation:**

```python
def handle_pattern_download_request(device_cn: str, pattern_uuid: str) -> PatternDownloadResponse:
    """Handle pattern download request, including receipt if purchased."""
    pattern = db.get_pattern(pattern_uuid)

    response = PatternDownloadResponse(
        pattern_uuid=pattern_uuid,
        download_url=generate_download_url(pattern_uuid),
        pattern=pattern_to_proto(pattern)
    )

    # Check if this device has purchased this pattern
    purchase = db.query_one(
        "SELECT * FROM purchases WHERE device_cn = ? AND pattern_uuid = ?",
        device_cn, pattern_uuid
    )

    if purchase:
        receipt = PurchaseReceipt(
            pattern_uuid=pattern_uuid,
            for_device=device_cn,
            purchased_at=purchase.purchased_at,
            receipt_id=purchase.receipt_id,
            pattern_name=pattern.name
        )

        payload = receipt.SerializeToString()
        response.purchase_receipt_payload = payload
        response.purchase_receipt_signature = sign_receipt(payload, SERVER_PRIVATE_KEY)

    return response
```

## Database Schema (Server)

```sql
CREATE TABLE purchases (
    id SERIAL PRIMARY KEY,
    device_cn VARCHAR(128) NOT NULL,        -- e.g., "TRANQUIL-XXXXX.iotdevices.koiosdigital.net"
    pattern_uuid VARCHAR(40) NOT NULL,
    receipt_id VARCHAR(64) UNIQUE NOT NULL, -- e.g., "rcpt_abc123xyz"
    purchased_at BIGINT NOT NULL,           -- Unix timestamp
    pattern_name VARCHAR(128),
    order_id VARCHAR(64),                   -- Reference to payment system
    created_at TIMESTAMP DEFAULT NOW(),

    UNIQUE(device_cn, pattern_uuid)         -- One purchase per device per pattern
);

CREATE INDEX idx_purchases_device ON purchases(device_cn);
CREATE INDEX idx_purchases_pattern ON purchases(pattern_uuid);
```

## Purchase Flow (End-to-End)

```
User initiates purchase in app
         │
         ▼
┌─────────────────────────┐
│ Payment processed       │
│ (Stripe, Apple, etc.)   │
└─────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│ Server creates purchase │
│ record in database      │
└─────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│ Server sends receipt    │
│ via WebSocket push      │
│ (or waits for sync)     │
└─────────────────────────┘
         │
         ▼
Device receives receipt
         │
         ├── Verifies signature (RSA-2048)
         ├── Verifies for_device matches cert CN
         ├── Saves to /sd/patterns/{uuid}.licdat
         └── Updates ManifestDatabase.purchased=true
```

## Security Considerations

### Device Binding

Each receipt is bound to a specific device via the `for_device` field, which must match the device's certificate CN. This prevents receipt sharing between devices.

The device validates this by:
1. Extracting CN from its X.509 certificate
2. Comparing with `for_device` field in receipt
3. Rejecting if mismatch

### Signature Verification

Receipts are signed with the server's RSA private key. Devices verify using the pinned public key in firmware. This ensures:
- Receipts cannot be forged
- Receipts cannot be modified after signing

### Receipt ID Uniqueness

Each `receipt_id` should be globally unique (e.g., UUID or prefixed ID like `rcpt_xxx`). This allows:
- Customer support lookups
- Duplicate detection
- Audit trails

## API Endpoints (REST - Optional)

While receipt sync is WebSocket-based, you may want REST endpoints for admin/support:

```
GET /api/admin/purchases?device_cn={cn}
    Returns all purchases for a device

GET /api/admin/purchases/{receipt_id}
    Returns specific purchase details

POST /api/admin/purchases
    Body: { device_cn, pattern_uuid, order_id }
    Creates a new purchase (after payment verification)

DELETE /api/admin/purchases/{receipt_id}
    Revokes a purchase (device will lose access on next sync)
```

## PatternInfo Ownership Fields

When returning pattern metadata, include ownership information:

```protobuf
message PatternInfo {
    // ... existing fields ...
    bool is_owned = 12;            // True if purchased
    int64 purchased_at = 13;       // Unix timestamp, 0 if not owned
    string receipt_id = 14;        // For support, empty if not owned
}
```

Server should populate these based on the requesting device's purchases:

```python
def pattern_to_proto(pattern, device_cn: str) -> PatternInfo:
    info = PatternInfo(
        uuid=pattern.uuid,
        name=pattern.name,
        # ... other fields ...
    )

    # Check ownership for this device
    purchase = db.query_one(
        "SELECT * FROM purchases WHERE device_cn = ? AND pattern_uuid = ?",
        device_cn, pattern.uuid
    )

    if purchase:
        info.is_owned = True
        info.purchased_at = purchase.purchased_at
        info.receipt_id = purchase.receipt_id

    return info
```

## Error Handling

### Invalid Receipt on Device

If a device receives an invalid receipt (bad signature, wrong device), it should:
1. Log the error
2. Not save the receipt
3. Continue processing other receipts (for sync)
4. Pattern remains inaccessible without valid subscription

### Missing Receipt After Purchase

If a user completes purchase but device doesn't receive receipt:
1. Device will request via `SyncPurchasesRequest` on next boot/reconnect
2. User can manually trigger sync from app
3. Support can verify purchase via `receipt_id` and resend

## Testing

### Generate Test Receipt

```python
def generate_test_receipt(device_cn: str, pattern_uuid: str) -> tuple[bytes, bytes]:
    """Generate a test receipt for development."""
    receipt = PurchaseReceipt(
        pattern_uuid=pattern_uuid,
        for_device=device_cn,
        purchased_at=int(time.time()),
        receipt_id=f"test_{uuid.uuid4().hex[:12]}",
        pattern_name="Test Pattern"
    )

    payload = receipt.SerializeToString()
    signature = sign_receipt(payload, TEST_PRIVATE_KEY)

    return payload, signature
```

### Verify Receipt Manually

```bash
# Dump receipt file header
xxd -l 16 /sd/patterns/{uuid}.licdat

# Expected: 4b44 5052 0100 0000 [payload_len] [sig_len]
#           K D  P R  v1   flags
```
