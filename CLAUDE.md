# Claude Code Instructions

Project-specific instructions for Claude Code when working on the Tranquil firmware.

## API Documentation Sync

**IMPORTANT:** When modifying any of the following, you MUST update the corresponding API documentation:

| Change Type | Files to Update |
|-------------|-----------------|
| REST API endpoints (`main/api/*.cpp`) | `docs/swagger.json` |
| WebSocket handlers (`main/api/handlers/*.cpp`) | `docs/asyncapi.yaml` |
| Operations core (`main/api/ops/*.cpp`, behaviour shared by REST and WS) | `docs/swagger.json` and `docs/asyncapi.yaml` |
| Protobuf definitions (`components/protobufs/proto/kd/v1/*.proto`) | `docs/asyncapi.yaml` |
| Message dispatcher registration (`main/api/dispatcher/message_dispatcher.cpp`) | `docs/asyncapi.yaml` |

### API layering

- Side effects and validation live once, in `main/api/ops/` (`player_ops`, `led_ops`, `library_ops`, `schedule_ops`, `system_ops`). Every op returns an `OpResult { esp_err_t err; const char* detail; }` and posts the matching `library_notifier::*_changed()` itself.
- REST handlers (`main/api/*_api.cpp`) and WebSocket handlers (`main/api/handlers/*.cpp`) are codecs only: parse the wire format, call the op, serialise the result. Do not put a side effect in a handler.
- One error mapping per transport: `api_common::send_op_error()` (esp_err -> HTTP status) and `HandlerBase::makeCommandResult()` (esp_err -> CommandResult). Do not hand-map errors in handlers.
- Shared serialisers (`pattern_to_json` / `build_pattern_info_message`, `playlist_to_json` / `build_playlist_list_message`, `player_ops::to_json` / `state_to_proto`, ...) live next to their ops; REST and WS must not grow separate copies.

### Checklist for API Changes

1. **Adding a new REST endpoint:**
   - Add handler in `main/api/` or existing `*_api.cpp` file
   - Register in the corresponding `register_handlers()` function
   - Add path, method, parameters, request/response schemas to `docs/swagger.json`

2. **Adding a new WebSocket message:**
   - Define message in `components/protobufs/proto/kd/v1/tranquil.proto`
   - Add handler in `main/api/handlers/`
   - Register in `dispatcher_register_standard_handlers()` if needed
   - Add message to `docs/asyncapi.yaml` under both publish (client->device) or subscribe (device->client)
   - Add schema definition in `docs/asyncapi.yaml` components/schemas

3. **Modifying existing messages/endpoints:**
   - Update the corresponding schema in `swagger.json` or `asyncapi.yaml`
   - Ensure field names, types, and descriptions match the implementation

## Hardware Constraints

- Max WebSocket clients: 4
- Max inbound WebSocket frame size: 8KB (outbound frames and every handler response are capped at 32 KB by `HandlerBase::serialize`)
- Max cloud message size: 32 KB (`MAX_MSG_SIZE` in `main/sockets/sockets.cpp`)
- Pattern upload chunk size: 4KB

## Memory Considerations

- Use SPIRAM (`heap_caps_malloc` with `MALLOC_CAP_SPIRAM`) for large allocations
- Static allocations preferred for handler response buffers
- Always check malloc return values
