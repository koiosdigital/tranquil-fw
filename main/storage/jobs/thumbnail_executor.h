#pragma once

#include "job_processor.h"

namespace jobs {

/**
 * @brief Executes thumbnail generation jobs
 *
 * Renders pattern to 600x600 grayscale PNG.
 * Works with both encrypted and unencrypted patterns.
 */
class ThumbnailExecutor : public IJobExecutor {
public:
    ThumbnailExecutor() = default;

    JobResult execute(const Job& job) override;
    JobType jobType() const override { return JobType::Thumbnail; }

private:
    // Stream pattern points through renderer
    JobResult renderPattern(const std::string& pattern_uuid,
                            bool encrypted,
                            const std::string& output_path);
};

} // namespace jobs
