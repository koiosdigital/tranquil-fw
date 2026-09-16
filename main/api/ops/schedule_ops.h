// Schedule operations: set / run, and the REST JSON codec with wire-value
// validation. Shared by schedule_api.cpp and schedule_handler.cpp; the
// protobuf codec lives in ScheduleManager.
#pragma once

#include "op_result.h"
#include "api_common.h"
#include "schedule_manager.h"

#include <optional>
#include <vector>

namespace schedule_ops {

// Replace the schedule. The requester is answered with the stored schedule
// by the transport; pass its socket fd (or exclude_cloud) so the notifier's
// push skips it.
OpResult set_schedule(std::vector<ScheduleItem> items,
                      const std::optional<QuietHours>& quiet,
                      int exclude_local_fd = -1,
                      bool exclude_cloud = false);

// Run one action now. A failure carries the reason in detail
// ("Quiet hours active", ...).
OpResult run_action(const ScheduleAction& action, bool force);

// REST shape (docs/swagger.json Schedule).
void to_json(api_common::ChunkBuf* cb);

// JSON -> model. Every numeric field is range-checked and enum values are
// validated; on failure *err names the offending field (static string).
bool action_from_json(const cJSON* json, ScheduleAction& out, const char** err);
bool schedule_from_json(const cJSON* root, std::vector<ScheduleItem>& items,
                        std::optional<QuietHours>& quiet, const char** err);

}  // namespace schedule_ops
