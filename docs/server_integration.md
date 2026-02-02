# Tranquil Server Integration Guide

This document describes how to implement server-side handling for Tranquil device DRM, licensing, and encrypted pattern downloads.

## Overview

The Tranquil device communicates with the cloud server via WebSocket using Protocol Buffers. The server must handle:

1. **License Management** - Issue and renew device licenses
2. **Pattern Downloads** - Provide encrypted patterns with device-specific encryption
3. **Device Authentication** - Verify device identity via mTLS certificates

## Protocol Buffer Messages

All messages are wrapped in `TranquilMessage`. See `components/protobufs/proto/kd/v1/tranquil.proto` for full definitions.

---

## 1. License System

### License Request (Device → Server)

```protobuf
message LicenseRequest {
    string device_id = 1;              // Certificate CN (e.g., "TRANQUIL-188B0EFB5C6C.iotdevices.koiosdigital.net")
    bytes current_license_hash = 2;    // SHA-256 of current license (for freshness check)
    int64 device_timestamp = 3;        // Device's current UTC timestamp
}
```

### License Response (Server → Device)

```protobuf
message LicenseResponse {
    bool success = 1;
    LicensePayload license = 2;        // The license content
    bytes signature = 3;               // RSA-SHA256 signature over serialized LicensePayload
    int64 server_timestamp = 4;
    string error = 5;                  // Error message if success=false
}

message LicensePayload {
    string for_device = 1;             // Must match device certificate CN exactly
    uint32 max_patterns = 2;           // Maximum number of patterns device can store
    int64 valid_from = 3;              // Unix timestamp UTC
    int64 valid_to = 4;                // Unix timestamp UTC
    string license_id = 5;             // Unique license ID for tracking/revocation
    int64 issued_at = 6;               // When license was issued
}
```

### TypeScript Implementation

```typescript
import * as crypto from 'crypto';
import * as protobuf from 'protobufjs';

// Load your protobuf definitions
const root = await protobuf.load('tranquil.proto');
const LicensePayload = root.lookupType('kd.v1.LicensePayload');
const LicenseResponse = root.lookupType('kd.v1.LicenseResponse');

// RSA-2048 private key for signing licenses (keep secure!)
const LICENSE_PRIVATE_KEY = `-----BEGIN RSA PRIVATE KEY-----
... your private key ...
-----END RSA PRIVATE KEY-----`;

interface LicenseParams {
  deviceId: string;          // From mTLS certificate CN
  maxPatterns: number;
  validDays: number;
  licenseId?: string;
}

function createLicenseResponse(params: LicenseParams): Uint8Array {
  const now = Math.floor(Date.now() / 1000);
  const validTo = now + (params.validDays * 24 * 60 * 60);

  // Create license payload
  const licensePayload = {
    forDevice: params.deviceId,
    maxPatterns: params.maxPatterns,
    validFrom: now,
    validTo: validTo,
    licenseId: params.licenseId || `LIC-${Date.now()}-${crypto.randomBytes(4).toString('hex').toUpperCase()}`,
    issuedAt: now,
  };

  // Serialize payload for signing
  const payloadMessage = LicensePayload.create(licensePayload);
  const payloadBytes = LicensePayload.encode(payloadMessage).finish();

  // Sign with RSA-SHA256
  const sign = crypto.createSign('SHA256');
  sign.update(payloadBytes);
  const signature = sign.sign(LICENSE_PRIVATE_KEY);

  // Create response
  const response = LicenseResponse.create({
    success: true,
    license: licensePayload,
    signature: signature,
    serverTimestamp: now,
  });

  return LicenseResponse.encode(response).finish();
}

// Example: Handle license request
function handleLicenseRequest(deviceId: string, request: any): Uint8Array {
  // Verify device exists and is authorized
  const device = await getDeviceByDN(deviceId);
  if (!device) {
    return LicenseResponse.encode({
      success: false,
      error: 'Device not found',
      serverTimestamp: Math.floor(Date.now() / 1000),
    }).finish();
  }

  // Check subscription status, etc.
  const maxPatterns = device.subscription === 'premium' ? 1000 : 100;

  return createLicenseResponse({
    deviceId: deviceId,
    maxPatterns: maxPatterns,
    validDays: 365,
  });
}
```

### License File Format (On Device SD Card)

The device stores licenses at `/sd/license.dat`:

```
Offset  Size  Field
------  ----  -----
0x0000  4     Magic: "KDLC" (0x4B444C43)
0x0004  2     Version: 0x0001
0x0006  2     Flags: 0x0000 (reserved)
0x0008  4     Payload length (little-endian)
0x000C  4     Signature length (little-endian)
0x0010  var   Serialized LicensePayload (protobuf)
var     var   RSA-SHA256 signature (256 bytes for RSA-2048)
```

---

## 2. Pattern Download System

### Pattern Format

Patterns use a **binary format** for compact storage and fast parsing:

```c
// Point format: 6 bytes per point
struct BinaryPoint {
    float theta;     // 4 bytes, angle in radians (signed, can exceed ±2π)
    uint16_t rho;    // 2 bytes, radius 0-65535 maps to 0.0-1.0
};
```

**Rho Conversion:**
- Server: `rho_uint16 = Math.round(rho_float * 65535)`
- Device: `rho_float = rho_uint16 / 65535.0`

### Request Pattern Download (Device → Server)

```protobuf
message RequestPatternDownload {
    string pattern_uuid = 1;
}
```

### Pattern Download Response (Server → Device)

```protobuf
message PatternDownloadResponse {
    bool success = 1;
    string error = 2;
    string pattern_uuid = 3;
    string download_url = 4;           // HTTPS URL to download encrypted pattern
    string token = 5;                  // Bearer token for download_url
    PatternInfo pattern = 6;           // Pattern metadata
    EncryptionMetadata encryption = 7; // Required when pattern.encrypted=true
}

message PatternInfo {
    string uuid = 1;
    string name = 2;
    string creator = 3;
    bool encrypted = 4;
    uint32 size_bytes = 5;             // Original unencrypted size
    bool reversible = 6;
    float start_point = 7;
    string created_at = 8;             // ISO 8601 timestamp
}

message EncryptionMetadata {
    bytes encrypted_key = 1;           // RSA-OAEP encrypted AES-256 key (512 bytes)
    bytes iv = 2;                      // AES-CTR IV (16 bytes)
    uint64 original_size = 3;          // Unencrypted pattern size
    bytes original_hash = 4;           // SHA-256 of plaintext pattern
    uint32 point_count = 5;            // Number of points in pattern
}
```

### Encryption Scheme

Patterns are encrypted using **hybrid encryption**:

1. **AES-256-CTR** for the pattern data (symmetric, fast, supports streaming)
2. **RSA-4096-OAEP-SHA256** for the AES key (asymmetric, device-specific)

The device's RSA public key is extracted from its mTLS certificate.

**Why CTR over GCM?** CTR mode allows random-access decryption and efficient streaming. Integrity is verified on download via the SHA-256 hash in the header.

### TypeScript Implementation

```typescript
import * as crypto from 'crypto';
import * as fs from 'fs';

/**
 * Binary point format: float32 theta + uint16 rho = 6 bytes
 */
interface BinaryPoint {
  theta: number;  // radians
  rho: number;    // 0.0 to 1.0
}

/**
 * Convert pattern points to binary format
 */
function createBinaryPayload(points: BinaryPoint[]): Buffer {
  const buf = Buffer.alloc(points.length * 6);
  points.forEach((p, i) => {
    buf.writeFloatLE(p.theta, i * 6);
    buf.writeUInt16LE(Math.round(p.rho * 65535), i * 6 + 4);
  });
  return buf;
}

/**
 * Parse text pattern (theta rho per line) to binary points
 */
function parseTextPattern(text: string): BinaryPoint[] {
  return text
    .split('\n')
    .filter(line => line.trim() && !line.startsWith('#'))
    .map(line => {
      const [theta, rho] = line.trim().split(/\s+/).map(parseFloat);
      return { theta, rho };
    });
}

interface EncryptPatternResult {
  encryptedData: Buffer;       // AES-CTR ciphertext (no tag)
  encryptedKey: Buffer;        // RSA-OAEP encrypted AES key
  iv: Buffer;                  // 16-byte IV
  originalSize: number;
  originalHash: Buffer;        // SHA-256 of plaintext
  pointCount: number;
}

/**
 * Encrypt a pattern for a specific device
 *
 * @param patternData - Binary pattern data (BinaryPoint array)
 * @param devicePublicKeyPem - Device's RSA-4096 public key from mTLS certificate
 * @param pointCount - Number of points in pattern
 */
function encryptPattern(
  patternData: Buffer,
  devicePublicKeyPem: string,
  pointCount: number
): EncryptPatternResult {
  // Generate random AES-256 key and IV
  const aesKey = crypto.randomBytes(32);  // 256 bits
  const iv = crypto.randomBytes(16);      // 128 bits for CTR

  // Calculate original hash (for integrity verification on download)
  const originalHash = crypto.createHash('sha256').update(patternData).digest();

  // Encrypt pattern with AES-256-CTR
  const cipher = crypto.createCipheriv('aes-256-ctr', aesKey, iv);
  const ciphertext = Buffer.concat([
    cipher.update(patternData),
    cipher.final(),
  ]);

  // Encrypt AES key with device's RSA public key using OAEP-SHA256
  const encryptedKey = crypto.publicEncrypt(
    {
      key: devicePublicKeyPem,
      padding: crypto.constants.RSA_PKCS1_OAEP_PADDING,
      oaepHash: 'sha256',
    },
    aesKey
  );

  // Securely clear the AES key from memory
  aesKey.fill(0);

  return {
    encryptedData: ciphertext,
    encryptedKey,
    iv,
    originalSize: patternData.length,
    originalHash,
    pointCount,
  };
}

/**
 * Create encrypted pattern file for download
 *
 * Returns the complete .dat file that the device will download
 */
function createEncryptedPatternFile(
  textPatternData: string,
  devicePublicKeyPem: string
): Buffer {
  // Parse text pattern and convert to binary
  const points = parseTextPattern(textPatternData);
  const binaryPayload = createBinaryPayload(points);

  const encrypted = encryptPattern(binaryPayload, devicePublicKeyPem, points.length);

  // Build the encrypted pattern file format
  // See EncryptedPatternHeader in EncryptedPatternReader.h

  const MAGIC = 0x4B444550;  // "KDEP"
  const VERSION = 0x0001;
  const SCHEME = 1;          // RSA-OAEP + AES-CTR
  const FLAGS = 0x01;        // Binary format

  // Header is 576 bytes total
  const header = Buffer.alloc(576);
  let offset = 0;

  // Magic (4 bytes)
  header.writeUInt32LE(MAGIC, offset); offset += 4;

  // Version (2 bytes)
  header.writeUInt16LE(VERSION, offset); offset += 2;

  // Scheme (1 byte)
  header.writeUInt8(SCHEME, offset); offset += 1;

  // Flags (1 byte)
  header.writeUInt8(FLAGS, offset); offset += 1;

  // Original size (4 bytes)
  header.writeUInt32LE(encrypted.originalSize, offset); offset += 4;

  // Point count (4 bytes)
  header.writeUInt32LE(encrypted.pointCount, offset); offset += 4;

  // Original hash (32 bytes)
  encrypted.originalHash.copy(header, offset); offset += 32;

  // Encrypted key (512 bytes for RSA-4096)
  if (encrypted.encryptedKey.length !== 512) {
    throw new Error(`Expected 512-byte encrypted key, got ${encrypted.encryptedKey.length}`);
  }
  encrypted.encryptedKey.copy(header, offset); offset += 512;

  // IV (16 bytes for CTR)
  encrypted.iv.copy(header, offset); offset += 16;

  // Verify header size
  if (offset !== 576) {
    throw new Error(`Header size mismatch: ${offset} != 576`);
  }

  // Combine: header + ciphertext (no auth tag with CTR)
  return Buffer.concat([header, encrypted.encryptedData]);
}

/**
 * Create unencrypted binary pattern file
 *
 * For patterns that don't require DRM protection
 */
function createUnencryptedPatternFile(textPatternData: string): Buffer {
  const points = parseTextPattern(textPatternData);
  const binaryPayload = createBinaryPayload(points);

  const MAGIC = 0x42524854;  // "THRB"

  // Header is 8 bytes
  const header = Buffer.alloc(8);
  header.writeUInt32LE(MAGIC, 0);
  header.writeUInt32LE(points.length, 4);

  return Buffer.concat([header, binaryPayload]);
}

// Example: Handle pattern download request
async function handlePatternDownloadRequest(
  deviceId: string,
  patternUuid: string
): Promise<any> {
  // Get device's certificate (stored as fullchain)
  const device = await getDeviceByDN(deviceId);
  if (!device || !device.fullchainPem) {
    return {
      success: false,
      error: 'Device certificate not found',
    };
  }

  // Extract public key from leaf certificate in fullchain
  const publicKey = getDevicePublicKey(device.fullchainPem);

  // Get pattern data
  const pattern = await getPatternById(patternUuid);
  if (!pattern) {
    return {
      success: false,
      error: 'Pattern not found',
    };
  }

  // Read plaintext pattern (text format from creator)
  const patternText = await fs.promises.readFile(pattern.filePath, 'utf-8');

  // Encrypt for this specific device
  const encryptedFile = createEncryptedPatternFile(patternText, publicKey);

  // Store encrypted file temporarily and generate download URL
  const downloadToken = crypto.randomBytes(32).toString('hex');
  const downloadPath = `/tmp/patterns/${patternUuid}-${device.id}.dat`;
  await fs.promises.writeFile(downloadPath, encryptedFile);

  // Store token -> file mapping (expires in 1 hour)
  await storeDownloadToken(downloadToken, downloadPath, 3600);

  // Parse for metadata
  const points = parseTextPattern(patternText);
  const binaryPayload = createBinaryPayload(points);
  const encrypted = encryptPattern(binaryPayload, publicKey, points.length);

  return {
    success: true,
    patternUuid: patternUuid,
    downloadUrl: `https://tranquil.api.koiosdigital.net/download/${downloadToken}`,
    token: downloadToken,
    pattern: {
      uuid: pattern.uuid,
      name: pattern.name,
      creator: pattern.creator,
      encrypted: true,
      sizeBytes: binaryPayload.length,
      reversible: pattern.reversible,
      startPoint: pattern.startPoint,
      createdAt: pattern.createdAt.toISOString(),
    },
    encryption: {
      encryptedKey: encrypted.encryptedKey,
      iv: encrypted.iv,
      originalSize: encrypted.originalSize,
      originalHash: encrypted.originalHash,
      pointCount: encrypted.pointCount,
    },
  };
}
```

### Encrypted Pattern File Format (.dat with KDEP magic)

The encrypted pattern file stored on the device SD card:

```
Offset   Size   Field
------   ----   -----
0x0000   4      Magic: "KDEP" (0x4B444550)
0x0004   2      Version: 0x0001
0x0006   1      Encryption scheme: 1 (RSA-OAEP + AES-CTR)
0x0007   1      Flags: 0x01 (binary format)
0x0008   4      Original size in bytes (little-endian uint32)
0x000C   4      Point count (little-endian uint32)
0x0010   32     Original SHA-256 hash (verified on download only)
0x0030   512    RSA-4096-OAEP encrypted AES-256 key
0x0230   16     AES-CTR IV (128-bit)
0x0240   var    AES-256-CTR encrypted binary points

Total header: 576 bytes (0x240)
Binary points: 6 bytes each (float32 theta + uint16 rho)
```

### Unencrypted Pattern File Format (.dat with THRB magic)

For patterns that don't require DRM:

```
Offset   Size   Field
------   ----   -----
0x0000   4      Magic: "THRB" (0x42524854)
0x0004   4      Point count (little-endian uint32)
0x0008   var    Binary points (6 bytes each)

Total header: 8 bytes
Binary points: 6 bytes each (float32 theta + uint16 rho)
```

---

## 3. Device Certificate Handling

Devices authenticate using mTLS with certificates signed by your CA. The certificate CN contains the device identifier.

### Fullchain Certificates

Devices store a **fullchain certificate** in NVS which contains:
1. **Leaf certificate** (device cert) - Contains the device's RSA-4096 public key
2. **Intermediate CA certificate(s)** - Chain to your root CA

When encrypting patterns, you need the **leaf certificate's public key**. Extract it from the fullchain:

```typescript
import * as crypto from 'crypto';

/**
 * Extract the leaf (device) certificate from a fullchain PEM
 *
 * A fullchain PEM contains multiple certificates concatenated:
 * -----BEGIN CERTIFICATE-----
 * (leaf cert)
 * -----END CERTIFICATE-----
 * -----BEGIN CERTIFICATE-----
 * (intermediate CA)
 * -----END CERTIFICATE-----
 */
function extractLeafCertificate(fullchainPem: string): string {
  // Split on certificate boundaries and get the first one
  const certRegex = /-----BEGIN CERTIFICATE-----[\s\S]*?-----END CERTIFICATE-----/g;
  const certs = fullchainPem.match(certRegex);

  if (!certs || certs.length === 0) {
    throw new Error('No certificates found in fullchain');
  }

  return certs[0];  // First certificate is the leaf (device cert)
}

/**
 * Get the device's RSA public key from a fullchain certificate
 */
function getDevicePublicKey(fullchainPem: string): string {
  const leafCert = extractLeafCertificate(fullchainPem);
  const cert = new crypto.X509Certificate(leafCert);
  return cert.publicKey.export({ type: 'spki', format: 'pem' }) as string;
}

// Usage:
const devicePublicKey = getDevicePublicKey(device.fullchainPem);
const encryptedFile = createEncryptedPatternFile(patternData, devicePublicKey);
```

### Certificate CN Format

```
TRANQUIL-{DEVICE_ID}.iotdevices.koiosdigital.net

Example: TRANQUIL-188B0EFB5C6C.iotdevices.koiosdigital.net
```

### Extracting Device ID

```typescript
import * as tls from 'tls';

function getDeviceIdFromConnection(socket: tls.TLSSocket): string | null {
  const cert = socket.getPeerCertificate();
  if (!cert || !cert.subject || !cert.subject.CN) {
    return null;
  }

  // CN format: TRANQUIL-XXXX.iotdevices.koiosdigital.net
  return cert.subject.CN;
}

// Or from certificate PEM
function getDeviceIdFromCert(certPem: string): string | null {
  const cert = new crypto.X509Certificate(certPem);
  return cert.subject.match(/CN=([^,]+)/)?.[1] || null;
}
```

---

## 4. WebSocket Message Handling

### Complete TypeScript WebSocket Handler

```typescript
import { WebSocket, WebSocketServer } from 'ws';
import * as https from 'https';
import * as protobuf from 'protobufjs';

const root = await protobuf.load('tranquil.proto');
const TranquilMessage = root.lookupType('kd.v1.TranquilMessage');

const wss = new WebSocketServer({
  server: httpsServer,
  verifyClient: (info, callback) => {
    // Require valid client certificate
    const cert = (info.req.socket as tls.TLSSocket).getPeerCertificate();
    if (!cert || !cert.subject) {
      callback(false, 401, 'Client certificate required');
      return;
    }
    callback(true);
  },
});

wss.on('connection', (ws, req) => {
  const deviceId = getDeviceIdFromConnection(req.socket as tls.TLSSocket);
  console.log(`Device connected: ${deviceId}`);

  ws.on('message', async (data: Buffer) => {
    try {
      const message = TranquilMessage.decode(data);
      const response = await handleMessage(deviceId, message);
      if (response) {
        ws.send(response);
      }
    } catch (err) {
      console.error('Message handling error:', err);
    }
  });
});

async function handleMessage(deviceId: string, message: any): Promise<Uint8Array | null> {
  // Check which message type was sent
  const messageCase = message.message;  // protobuf oneof field

  switch (Object.keys(message)[0]) {
    case 'licenseRequest':
      return wrapResponse('licenseResponse',
        await handleLicenseRequest(deviceId, message.licenseRequest));

    case 'requestPatternDownload':
      return wrapResponse('patternDownloadResponse',
        await handlePatternDownloadRequest(deviceId, message.requestPatternDownload.patternUuid));

    case 'certReport':
      await handleCertReport(deviceId, message.certReport);
      return null;  // No response needed

    case 'uploadCoreDump':
      await handleCoreDump(deviceId, message.uploadCoreDump);
      return null;

    default:
      console.log(`Unhandled message type from ${deviceId}`);
      return null;
  }
}

function wrapResponse(field: string, response: any): Uint8Array {
  const wrapper = TranquilMessage.create({ [field]: response });
  return TranquilMessage.encode(wrapper).finish();
}
```

---

## 5. Security Considerations

### Key Management

1. **License Signing Key**: RSA-2048 private key, keep in secure key vault (AWS KMS, HashiCorp Vault, etc.)
2. **Device Public Keys**: Extracted from mTLS certificates, can be cached
3. **AES Keys**: Generate fresh for each pattern encryption, never store

### Validation

1. **License Requests**: Verify device exists, check subscription status, rate limit
2. **Pattern Downloads**: Verify device owns/has access to pattern, check license limits
3. **Time Validation**: Server should reject requests with large clock skew (>5 minutes)

### Rate Limiting

```typescript
const rateLimiter = new Map<string, { count: number; resetAt: number }>();

function checkRateLimit(deviceId: string, limit: number, windowMs: number): boolean {
  const now = Date.now();
  let entry = rateLimiter.get(deviceId);

  if (!entry || entry.resetAt < now) {
    entry = { count: 0, resetAt: now + windowMs };
    rateLimiter.set(deviceId, entry);
  }

  entry.count++;
  return entry.count <= limit;
}

// Usage: 10 license requests per minute
if (!checkRateLimit(deviceId, 10, 60000)) {
  return { success: false, error: 'Rate limit exceeded' };
}
```

---

## 6. Testing

### Generate Test License

Use the Python script in `keys/generate_license.py`:

```bash
cd keys
python generate_license.py \
  --device "TRANQUIL-188B0EFB5C6C.iotdevices.koiosdigital.net" \
  --max-patterns 100 \
  --valid-years 75 \
  --output license.dat \
  --c-output test_license.h
```

### Verify License Signature (Node.js)

```typescript
function verifyLicenseSignature(
  payloadBytes: Buffer,
  signature: Buffer,
  publicKeyPem: string
): boolean {
  const verify = crypto.createVerify('SHA256');
  verify.update(payloadBytes);
  return verify.verify(publicKeyPem, signature);
}
```

### Test Pattern Encryption

```typescript
// Create a test pattern (text format)
const testPatternText = '0.0 0.5\n0.1 0.6\n0.2 0.7\n0.3 0.8\n';

// Device test public key (from test certificate)
const devicePublicKey = fs.readFileSync('test_device_public.pem', 'utf-8');

// Encrypt and save
const encryptedFile = createEncryptedPatternFile(testPatternText, devicePublicKey);
fs.writeFileSync('test_pattern.dat', encryptedFile);

console.log(`Created encrypted pattern: ${encryptedFile.length} bytes`);
console.log(`  Header: 576 bytes`);
console.log(`  Ciphertext: ${encryptedFile.length - 576} bytes`);
console.log(`  Points: ${(encryptedFile.length - 576) / 6} points`);

// Create unencrypted version
const unencryptedFile = createUnencryptedPatternFile(testPatternText);
fs.writeFileSync('test_pattern_unenc.dat', unencryptedFile);
console.log(`Created unencrypted pattern: ${unencryptedFile.length} bytes`);
```

---

## Appendix: File Format Summary

| File | Location | Magic | Description |
|------|----------|-------|-------------|
| License | `/sd/license.dat` | `KDLC` (0x4B444C43) | Signed license with device binding |
| Encrypted Pattern | `/sd/patterns/{uuid}.dat` | `KDEP` (0x4B444550) | Device-specific encrypted binary pattern |
| Unencrypted Pattern | `/sd/patterns/{uuid}.dat` | `THRB` (0x42524854) | Unencrypted binary pattern |

### Binary Point Format

All patterns use binary point format (6 bytes per point):

| Field | Type | Size | Description |
|-------|------|------|-------------|
| theta | float32 | 4 bytes | Angle in radians (little-endian) |
| rho | uint16 | 2 bytes | Radius 0-65535 → 0.0-1.0 (little-endian) |
