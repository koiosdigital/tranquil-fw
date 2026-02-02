#pragma once

#include "job_processor.h"
#include "job_types.h"

namespace jobs {

/**
 * @brief Executor for pattern download jobs
 *
 * Downloads patterns from cloud URLs and saves them to SD card.
 * On success, enqueues a thumbnail generation job.
 */
class DownloadExecutor : public IJobExecutor {
public:
    JobResult execute(const Job& job) override;
    JobType jobType() const override { return JobType::Download; }

private:
    JobResult performDownload(const std::string& pattern_uuid,
                              const DownloadJobData& data);
};

} // namespace jobs
