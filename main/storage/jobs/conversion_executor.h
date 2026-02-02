#pragma once

#include "job_processor.h"

namespace jobs {

/**
 * @brief Executes binary conversion jobs (.tmp -> .dat)
 *
 * Converts ASCII .thr format to binary THRB format.
 * After successful conversion, enqueues a thumbnail generation job.
 */
class ConversionExecutor : public IJobExecutor {
public:
    ConversionExecutor() = default;

    JobResult execute(const Job& job) override;
    JobType jobType() const override { return JobType::Conversion; }

private:
    // Single-pass ASCII THR to binary THRB conversion
    esp_err_t convertToBinary(const char* input_path,
                               const char* output_path,
                               size_t* out_point_count);

    // Add pattern to database after successful conversion
    esp_err_t addPatternToDatabase(const std::string& uuid,
                                    const std::string& name,
                                    size_t file_size,
                                    bool encrypted);
};

} // namespace jobs
