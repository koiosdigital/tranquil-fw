// Local upload-pipeline progress broadcaster.
//
// The pattern-download bar (download_progress.*) covers the store-download path
// (request -> HTTP -> convert -> thumbnail, one 0-100 bar). This module covers
// the *local upload* path the proto documents separately: a user POSTs a .thr
// to /api/patterns and watches it convert (THR ASCII -> THRB binary) and then
// render a thumbnail. Those two stages each get their own report so the app can
// show "converting… / rendering…" with a real percentage and a terminal
// complete/failed.
//
// Both reports are broadcast to local WS clients and to the cloud link. Unlike
// download_progress these are stateless one-shot sends (the client tracks its
// own state), so there is no map/mutex here.
#pragma once

#include <cstdint>
#include <string>

namespace pipeline_progress {

// stage: "converting" (in progress) | "complete" | "failed"
void conversion(const std::string& uuid, const char* stage, uint8_t pct,
                const std::string& error = "");

// stage: "rendering" (in progress) | "complete" | "failed"
void thumb(const std::string& uuid, const char* stage, uint8_t pct,
           const std::string& error = "");

}  // namespace pipeline_progress
