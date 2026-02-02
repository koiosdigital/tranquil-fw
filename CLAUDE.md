# Claude Code Instructions

Project-specific instructions for Claude Code when working on the Tranquil firmware.

## API Documentation Sync

**IMPORTANT:** When modifying any of the following, you MUST update the corresponding API documentation:

| Change Type | Files to Update |
|-------------|-----------------|
| REST API endpoints (`main/api/*.cpp`) | `docs/swagger.json` |
| WebSocket handlers (`main/api/handlers/*.cpp`) | `docs/asyncapi.yaml` |
| Protobuf definitions (`components/protobufs/proto/kd/v1/*.proto`) | `docs/asyncapi.yaml` |
| Message dispatcher registration (`main/api/dispatcher/message_dispatcher.cpp`) | `docs/asyncapi.yaml` |

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
- Max WebSocket frame size: 8KB
- Max cloud message size: 8KB
- Pattern upload chunk size: 4KB

## Memory Considerations

- Use SPIRAM (`heap_caps_malloc` with `MALLOC_CAP_SPIRAM`) for large allocations
- Static allocations preferred for handler response buffers
- Always check malloc return values
