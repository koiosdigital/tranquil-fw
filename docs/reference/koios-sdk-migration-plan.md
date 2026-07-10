# Tranquil koios_sdk + TRANQUIL-fleet migration plan

Status: **planning** · Owner: Aiden · Last updated: 2026-07-09

Migrate tranquil off its hand-rolled cloud stack onto the shared `koios_sdk`
cloudlink, and stand up a **TRANQUIL fleet** in the new `device-api` so the
device's DRM/licensing/store flow works end-to-end. Spans three repos plus a
SaaS/dnet prerequisite.

---

## 1. Architecture: before → after

**Before**
```
tranquil-fw ──hand-rolled mTLS WSS──▶ device.api.koiosdigital.net (device-api-old / gone)
                                       licensing via admin HTTP on tranquil-patterns-store
cloud_sockets_init() is commented out → nothing connects, no license, no store token
```

**After**
```
tranquil-fw ──koios_sdk cloudlink (mTLS)──▶ dnet gateway  wss://vn-sec.koios.sh
                                              │  (device.message webhook, HMAC)
                                              ▼
                                     device-api  src/fleet/tranquil.ts
                                              │  reply via sendDeviceMessage → SaaS → device
                                              ▼
                          mints signed License + store_token JWT
                                              │
              device uses store_token ──REST──▶ tranquil-patterns-store (catalog / encrypted download)
OTA: koios_sdk ota.h ──▶ ota.api.koios.sh (device JWT from cloudlink)
```

Key shift: the device no longer holds a socket to the tranquil backend. It
connects to **dnet**; dnet forwards protobuf frames to `device-api` as webhooks
and relays replies back. Cert/PKI, JWT issuance, twin, and liveness are owned by
**SaaS PKI / dnet**, not by us.

---

## 1b. Concrete infrastructure (Phase 0 — provisioned)

- **TRANQUIL fleet UUID:** `1c5015d8-3f58-4999-8b74-6cbd54b4c49c`
  → goes in `device-api` `wrangler.jsonc` `KOIOS_FLEET_TYPES`:
  `"1c5015d8-3f58-4999-8b74-6cbd54b4c49c":"TRANQUIL"`.
- **Two independent PKIs — do not conflate:**
  1. **mTLS device identity (connection auth).** ECC leaf certs `CN=TRANQUIL-XXXX`
     issued by the **TRANQUIL intermediate CA** (`CN=TRANQUIL`, below), chaining to
     *Koios Platform Root ECC*. Validated by dnet on connect; provisioned onto
     devices by `kd_common` + SaaS PKI. The device's `for_device` license field =
     this cert CN.
     ```
     -----BEGIN CERTIFICATE-----  (TRANQUIL fleet intermediate CA)
     MIIBlTCCARugAwIBAgIQK9I9vEol0/v1JswPkJlVrjAKBggqhkjOPQQDAjAiMSAw
     HgYDVQQDExdLb2lvcyBQbGF0Zm9ybSBSb290IEVDQzAgFw0yNjA3MDkyMTAxMTla
     GA8yMDU2MDcwOTIxMDExOVowEzERMA8GA1UEAxMIVFJBTlFVSUwwdjAQBgcqhkjO
     PQIBBgUrgQQAIgNiAAQH4oKmNH/iDPydQ49oE6qRLJpG0XfVwJQWm5GdsGRElCBq
     afGFugssw9VSrxVyBKfvHgwIv59Dic+4fcX+Y6NGclhve+8SXv8qDZuveS+hK+EG
     e1fmfq7qa/SmQTMOyvOjIzAhMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQD
     AgEGMAoGCCqGSM49BAMCA2gAMGUCMGJeq9iF2YNHv9GwZeP5GcINTjNvVtZ1npwZ
     HfxTAnAvdpEd/tGZykCMBxty/mSDYwIxAJ6EGOPlKf9In4COOYxzXrJxT8rMZuUO
     ucWqqjZu/3mISvAbNRz3UTeKYwn+fbRowg==
     -----END CERTIFICATE-----
     ```
  2. **License signing (DRM).** Separate **RSA-2048** key. Public half embedded in
     firmware at `main/drm/drm_license.cpp:27-37`; private half = `device-api`
     `LICENSE_SIGNING_KEY` (must equal the key `tranquil-patterns-store` signs with).
     Nothing to do with the ECC mTLS CA above.

## 2. The load-bearing constraints (get these wrong = every device bricks its licensing)

1. **License signing key must match the firmware's embedded public key.**
   Firmware pins an RSA-2048 public key at `main/drm/drm_license.cpp:27-37` and
   verifies `RSA-PKCS1V15 / SHA-256` (`drm_license.cpp:120,306`). Signing lives
   solely in `tranquil-patterns-store` (`src/crypto.ts:187 signLicense`); its
   `LICENSE_SIGNING_KEY` must be the private half of that embedded key.
   device-api delegates to the store and holds no signing key. (Confirmed good.)
2. **`serializeLicensePayload` must be byte-identical.** The signature covers a
   hand-rolled protobuf serialization (`tranquil-patterns-store/src/crypto.ts:159`),
   not JSON. Field numbers/order/types are load-bearing — the device re-parses it.
   Port that function unchanged.
3. **Store token constants.** JWT is HS256 with `iss:"wcp"`, `aud:"wcp"`, signed
   by `USER_JWT_SECRET` (`tranquil-patterns-store/src/auth.ts:110`). Minted inside
   the store's `/license/internal` handler, so the constants + secret stay in one
   place; device-api never touches them.
4. **Cloudlink caches the device cert for process lifetime** (`core/cloudlink.c`
   `if(!s.cert_pem)`). After a cert renew, a plain reconnect re-presents the old
   cert. Must call `sockets_invalidate_cert_cache()` (deinit+init) — nemoto's
   pattern (`nemoto-fw/main/sockets/sockets.cpp:68-119`).
5. **Coordinated cutover.** Firmware pointed at `vn-sec.koios.sh` only works once
   (a) the TRANQUIL fleet exists in dnet, (b) device certs map CN→TRANQUIL fleet,
   (c) `device-api` TRANQUIL fleet is deployed. Sequence accordingly.

---

## 3. Where the logic actually lives (surprise finding)

`device-api-old` has **almost no** tranquil logic — no TRANQUIL device type, no
license/store/purchase code. The real tranquil cloud logic is in
**`tranquil-patterns-store`** (CF Worker + D1 + R2 + KV). So the cloud migration
is really *port the needed bits of tranquil-patterns-store into device-api's new
fleet module*, not device-api-old → device-api.

`device-api` is ~40% pre-staged: `src/protobufs/kd/v1/tranquil_pb.ts` already
exists (63-case `TranquilMessage`). Missing: the fleet type, handler, tables,
routes, config.

---

## 4. Workstreams

- **A — Firmware** (`tranquil-fw`): swap hand-rolled sockets for cloudlink; add OTA.
- **B — Cloud fleet** (`device-api`): register TRANQUIL fleet; port license/store-token signing.
- **C — Store/commerce** (`tranquil-patterns-store`): keep standalone (recommended) or fold in.
- **D — SaaS/dnet + PKI** (external): create TRANQUIL fleet, cert-CN mapping, webhook. **Prerequisite / another owner.**

---

## 5. Phased plan

### Phase 0 — SaaS/dnet prerequisites (blocks the firmware cutover)
- [x] Create the **TRANQUIL fleet** — UUID `1c5015d8-3f58-4999-8b74-6cbd54b4c49c`.
- [x] TRANQUIL intermediate mTLS CA issued (see §1b).
- [ ] Ensure device provisioning issues ECC leaf certs `CN=TRANQUIL-XXXX` chaining to
  that CA, and dnet maps the fleet → `TRANQUIL`.
- [ ] Subscribe the TRANQUIL fleet's `device.message` / `device.connected` webhooks
  to `device-api` `POST /v1/hooks/koios`.
- Deliverable: a provisioned dev unit that dnet recognizes as TRANQUIL.

### Phase 1 — device-api: register the TRANQUIL fleet (skeleton, no DRM yet)
- `src/env.ts:36` add `"TRANQUIL"` to `DeviceType`.
- `wrangler.jsonc:51` `KOIOS_FLEET_TYPES` add `"<tranquil-fleet-uuid>":"TRANQUIL"`.
- New migration: widen `devices.type` CHECK to include `TRANQUIL`; add
  `tranquil_patterns`, `tranquil_playlists`, `tranquil_config`, `tranquil_schedules`
  (mirror `nemoto_*` conflict-key conventions in `migrations/0002`).
- `src/hooks/koios.ts:120` add `case "TRANQUIL": return handleTranquilMessage(...)`.
- `src/fleet/tranquil.ts` (copy `src/fleet/nemoto.ts`): claim (reuse
  `src/auth/claim.ts` + `device_claims`) + `JoinResponse` + schedule/config/
  pattern/playlist digest-then-diff sync.
- `src/routes/tranquil.ts` + mount in `src/index.ts:124-133` (WEB CRUD).
- Deliverable: TRANQUIL device recognized, claim + basic sync work.

### Phase 2 — device-api: License + store token (the DRM core) — **delegated**
Rather than copy the store's signing key into device-api, delegate minting to
`tranquil-patterns-store` over a **service binding** — the same pattern MATRX
uses for the renderer. The RSA key + store-token secret never leave the store.
- `tranquil-patterns-store`: extract `mintDeviceLicense(env, device_id)` (shared
  by the admin `POST /license` and a new **`POST /license/internal`**). The
  internal route is guarded by a shared `INTERNAL_API_SECRET` header (the store
  is publicly routed, unlike a binding-only worker). Add `INTERNAL_API_SECRET`
  to `src/types.ts`.
- `device-api`: add the `PATTERNS_STORE` service binding
  (`wrangler.jsonc` services, `PATTERNS_STORE: Fetcher` in `env.ts`) and a single
  `INTERNAL_API_SECRET` secret (a low-value shared token, **not** the RSA key).
- `handleLicense`: `env.PATTERNS_STORE.fetch("/license/internal", {device_id: cn})`
  → relay `{license, signature, server_timestamp}` into a `LicenseResponse`
  protobuf (base64 signature → bytes) via `sendDeviceMessage`. `for_device` is the
  device's self-reported cert CN.
- `GetStoreTokenRequest`: firmware serves it locally from the cached license
  (`main/api/license_api.cpp` `/api/license/store-token`); no cloud handler.
- Deliverable: a claimed device receives a valid, signature-verifying license
  with a working store token, and device-api holds **zero** signing secrets.

### Phase 3 — tranquil-fw: cloudlink migration
- Vendor `koios_sdk` as a component (`components/koios_sdk`, git submodule). **Use
  the newer copy that has `device_class`** (matrx/nemoto vendored), not the stale
  `/Users/aiden/Projects/Koios/koios-sdk` canonical checkout. Update
  `main/CMakeLists.txt` + `main/idf_component.yml` to require `koios_sdk`.
- Rewrite `main/sockets/sockets.cpp` to the nemoto/matrx template (~507 → ~120
  lines): delete `esp_websocket_client` stack, `State` enum, inbox/outbox queues,
  reconnect/queue/state timers, WiFi/IP handlers, `start_client`, cert caching.
  Keep the SPIRAM `spiram_allocator`. Add `on_message`/`on_session_ready`/
  `on_disconnect` + `cloudlink_cfg()`:
  - `url = "wss://vn-sec.koios.sh"` (change from `device.api.koiosdigital.net`)
  - `auth_mode = KOIOS_CLOUD_AUTH_MTLS`
  - `device_class = "esp32s3-tranquil"` (OTA fleet targeting; distinct from the
    app-level `hardware_model="tranquil"` in SystemInfo)
  - `max_msg_size` ≥ 16 KB (coredump/purchase payloads; today's 8 KB too small)
- `main/sockets/messages.cpp`: keep every `cloud_msg_send_*`; rewire only the two
  primitives — `cloud_msg_queue_raw` and `cloud_msg_queue` call
  `koios_cloudlink_send(data,len)` instead of `xQueueSend`. Delete `cloud_msg_init`
  + `g_outbox`. **This one seam keeps the whole dispatcher/ResponseRouter graph
  untouched** (`ResponseRouter::sendToCloud` → `cloud_msg_queue_raw`).
- `main/sockets/handlers.cpp`: keep all DRM/license/purchase/cert handlers. Add
  `handlers_on_connected()` boot handshake: device_info → coredump → cert_report →
  license_request → sync_purchases (+ claim_if_needed). In
  `handle_cert_renew_response` add `sockets_invalidate_cert_cache()`.
- `main/main.cpp`: add `koios_ota_init(nullptr);` after `kd_common_init()`;
  **uncomment `cloud_sockets_init();`**.
- Deliverable: device connects to dnet, claims, license lands, store token
  retrievable, OTA wired.

### Phase 3.5 — device-api store front door (done)
device-api fronts `tranquil-patterns-store` over the `PATTERNS_STORE` service
binding so the app + firmware reach the catalog/downloads through one origin
(same pattern as `RENDERER`). The store stays a public worker for now (the
`sisyphusimporter` container uploads to it directly and can't hold a binding).
- **store**: `authMiddleware` also accepts `X-Internal-Secret` (== `INTERNAL_API_SECRET`)
  as user-level auth, so every store route works over the binding unchanged;
  `POST /license` accepts `isAdmin || isInternal`. Public store-token auth is
  untouched (non-breaking).
- **device-api**: `src/store.ts` (`storeFetch`/`proxyStore`), web plane
  `app.all("/v1/store/*")` (OIDC), device plane `devicePlane.all("/store/*")`
  (dnet JWT). License minting delegates via `storeFetch("/license")` — device-api
  holds no signing keys, only `INTERNAL_API_SECRET`.
- **Client repoint (when their store features are built):** app `cloud.ts`
  `STORE_API_URL` → `https://api.koiosdigital.net/v1/store`; firmware fetches
  `download_url` against `https://api.koiosdigital.net/d/v1/store` (store returns
  relative URLs, so only the base changes).
- **Full privatization (DONE):** the store is now a private worker
  (`workers_dev:false`, `preview_urls:false`, no routes), refactored to
  `@hono/zod-openapi` (`OpenAPIHono` + `createRoute` + zod DTOs + `doc31`) like
  kd-matrx-renderer-api. All self-auth removed (no store tokens / admin-user
  JWT / `INTERNAL_API_SECRET` / `auth_token` query param) — it trusts binding
  traffic. device-api's `proxyStore`/`storeFetch` no longer send any secret.
  Signing keys (`LICENSE_SIGNING_KEY`, `USER_JWT_SECRET`) stay in the store.
  Note: the `sisyphusimporter` (public admin uploader) is now cut off until it
  moves behind a device-api admin route.

### Phase 3.7 — Secure pattern download (authoritative cert, no submitted PEM) — DONE
The device never submits a certificate; identity is resolved server-side from
its verified token, and the pattern is encrypted to the exact provisioned cert.
```
firmware ──RequestPatternDownload(uuid)──▶ device-api (tranquil fleet handler)
  device-api: deviceId ──iot-api──▶ certificate_id ──pki.api.koios.sh (pki.read M2M)──▶ leaf SPKI + CN
  device-api: POST tranquil-store /patterns/:uuid/encrypted { device_cn, public_key }  (private binding)
  device-api ◀── one-time /patterns/download/<token> ── store
firmware ◀── PatternDownloadResponse{ download_url = https://api…/dl/<token>, pattern } ── device-api
firmware ──GET /dl/<token> (authless, one-time)──▶ device-api ──binding──▶ store  → KDEP file
firmware: EncryptedPatternReader decrypts with its DS-peripheral private key
```
- **koios-sh-pki**: external `GET /v1/certificates/:id` gated on `pki.read`,
  tenant-scoped, returns `{ subject_cn, cert_pem, public_key_spki, status }` —
  SPKI parsed from the stored leaf via `jose.importX509`. Reached by external
  HTTPS (koiosdigital ≠ koios — separate platforms, no same-account binding);
  device-api calls it with an `pki.read` M2M token, exactly like iot-api.
- **device-api**: `getDeviceCertificate(deviceId)` (iot-api `certificate_id` → PKI),
  `requestPatternDownload` handler, authless one-time `/dl/:token`, `SELF_BASE_URL`.
- **store**: encrypt takes `{ device_cn, public_key }`; dropped `@peculiar/x509`
  and all cert parsing. `POST /patterns/:uuid/encrypted` is now internal-only.
- **firmware**: unchanged — already requests over the socket, fetches the
  absolute authless URL, and decrypts KDEP with its provisioned key.
- Pinning to iot-api's *active* `certificate_id` (not "latest by device") means a
  pattern is bound to exactly the installed cert. A leaked/forged cert can't
  enter the flow (nothing is submitted); the file is useless without the
  provisioned private key.

### Phase 4 — Purchases / entitlements (green-field — defer-able)
Not implemented anywhere today (proto-only). `max_patterns=100` license covers
the near term.
- device-api: `tranquil_purchases` table + `signPurchaseReceipt` (RSA, mirror
  `signLicense`) + `handleSyncPurchases`.
- firmware already has `handle_sync_purchases_response` + `drm_purchase` — wire up.

### Phase 5 — tranquil-app: store UI
- Wire `PatternsView.vue` store tab (currently a stub returning local patterns) to
  `src/api/rest/cloud.ts` using the store token from `/api/license/store-token`.
- Purchase flow (depends on Phase 4).

---

## 6. Open decisions
1. **tranquil-patterns-store: standalone vs fold into device-api.**
   Recommend **standalone** — it already works (catalog, R2, encryption, one-time
   download tokens). device-api only mints license + store_token, sharing
   `LICENSE_SIGNING_KEY`/`USER_JWT_SECRET`. Folding in means adding an R2 binding
   and porting the encryption/THRB/KDEP + catalog code — larger, riskier, later.
2. **Purchases now or deferred.** Recommend **defer to Phase 4**.
3. **SaaS/dnet fleet + PKI owner.** Phase 0 is external to these three repos —
   confirm who provisions the fleet and cert-CN→fleet mapping.

## 7. Suggested sequencing
Phase 0 (external, start now) ∥ Phase 1 → Phase 2 (cloud) can proceed against a
provisioned dev unit. Phase 3 (firmware) needs Phases 0–2 landed to test
end-to-end, but the code rewrite can be developed in parallel behind the still-
commented `cloud_sockets_init()`. Phases 4–5 follow.
