#include "conversion_executor.h"
#include "job_queue.h"
#include "ManifestDatabase.h"
#include "PatternReader.h"
#include "esp_log.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <sys/stat.h>
#include <unistd.h>

static const char* TAG = "ConversionExecutor";

namespace jobs {

JobResult ConversionExecutor::execute(const Job& job) {
    // Parse job data
    ConversionJobData data = ConversionJobData::fromJson(job.job_data);

    if (data.temp_path.empty()) {
        return JobResult::fail_permanent("Missing temp_path in job data");
    }

    // Rolls back the orphan Pattern row the upload/download handler created
    // before conversion — on a permanent failure it would forever point at a
    // pattern with no .dat file.
    auto rollback_pattern = [&job]() {
        if (job.pattern_id != 0) {
            if (ManifestDatabase::instance().deletePattern(job.pattern_id) == ESP_OK) {
                ESP_LOGW(TAG, "Rolled back orphan pattern row id=%u after permanent conversion failure",
                    job.pattern_id);
            }
        }
    };

    // Get the pattern to find its external_uuid for file paths
    auto pattern = ManifestDatabase::instance().getPattern(job.pattern_id);
    if (!pattern) {
        return JobResult::fail_permanent("Pattern not found for job");
    }
    const std::string& pattern_uuid = pattern->external_uuid;

    // Verify input file exists — it never reappears, so this is permanent
    struct stat st;
    if (stat(data.temp_path.c_str(), &st) != 0) {
        rollback_pattern();
        return JobResult::fail_permanent("Input file not found: " + data.temp_path);
    }

    // Build output paths (use external_uuid for file naming)
    char binary_path[128];
    char final_path[128];
    snprintf(binary_path, sizeof(binary_path), "/sd/patterns/%s.thrb", pattern_uuid.c_str());
    snprintf(final_path, sizeof(final_path), "/sd/patterns/%s.dat", pattern_uuid.c_str());

    // Perform conversion. The input file is only deleted on SUCCESS or on
    // PERMANENT failure — unlinking it on a transient failure guarantees
    // every retry fails.
    size_t point_count = 0;
    esp_err_t result = convertToBinary(data.temp_path.c_str(), binary_path, &point_count);

    if (result != ESP_OK || point_count == 0) {
        unlink(binary_path);  // partial output is never useful
        if (result == ESP_OK && point_count == 0) {
            // Input parsed cleanly but contains no points — retrying the
            // same file can never succeed.
            unlink(data.temp_path.c_str());
            rollback_pattern();
            return JobResult::fail_permanent("No valid points found in input file");
        }
        // I/O error — keep the input so a retry has something to convert
        return JobResult::fail("Conversion failed");
    }

    // Atomic rename to final path
    if (rename(binary_path, final_path) != 0) {
        unlink(binary_path);
        return JobResult::fail("Failed to rename to final path");
    }

    // Get final file size
    size_t file_size = 0;
    if (stat(final_path, &st) == 0) {
        file_size = st.st_size;
    }

    // Update pattern in database with final size
    pattern->size_bytes = file_size;
    pattern->encrypted = data.encrypted;
    if (!data.name.empty()) {
        pattern->name = data.name;
    }
    esp_err_t db_result = ManifestDatabase::instance().updatePattern(job.pattern_id, *pattern);
    if (db_result != ESP_OK) {
        // Keep the input so a retry can regenerate the output
        unlink(final_path);
        return JobResult::fail("Failed to update pattern in database");
    }

    // Success - delete temp file
    unlink(data.temp_path.c_str());

    ESP_LOGD(TAG, "Conversion complete: %s (%zu points)", pattern_uuid.c_str(), point_count);

    // Enqueue thumbnail generation job using pattern's internal ID
    ThumbnailJobData thumb_data;
    thumb_data.encrypted = data.encrypted;
    thumb_data.output_path = "/sd/previews/" + pattern_uuid + ".png";

    JobQueue::instance().enqueueThumbnail(job.pattern_id, thumb_data, -1);

    return JobResult::ok();
}

esp_err_t ConversionExecutor::convertToBinary(const char* input_path,
                                               const char* output_path,
                                               size_t* out_point_count) {
    FILE* in = fopen(input_path, "r");
    FILE* out = fopen(output_path, "wb");

    if (!in || !out) {
        if (in) fclose(in);
        if (out) fclose(out);
        return ESP_FAIL;
    }

    // Write placeholder header (will update point_count at end)
    UnencryptedPatternHeader header = {
        .magic = BINARY_PATTERN_MAGIC,
        .point_count = 0
    };

    if (fwrite(&header, sizeof(header), 1, out) != 1) {
        fclose(in);
        fclose(out);
        return ESP_FAIL;
    }

    // Batch writes for better I/O performance
    constexpr size_t BATCH_SIZE = 256;
    BinaryPoint batch[BATCH_SIZE];
    size_t batch_idx = 0;
    size_t point_count = 0;

    char line[256];
    while (fgets(line, sizeof(line), in)) {
        // Skip leading whitespace
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;

        // Skip comments and empty lines
        if (*p == '#' || *p == '\n' || *p == '\r' || *p == '\0') {
            continue;
        }

        // Fast parsing with strtod
        char* end;
        double theta = strtod(p, &end);
        if (end == p) continue;

        p = end;
        while (*p == ' ' || *p == '\t') p++;

        double rho = strtod(p, &end);
        if (end == p) continue;

        // Add to batch
        batch[batch_idx].theta = static_cast<float>(theta);
        batch[batch_idx].rho = static_cast<uint16_t>(
            std::round(std::clamp(rho, 0.0, 1.0) * 65535.0));
        batch_idx++;
        point_count++;

        // Flush batch when full
        if (batch_idx == BATCH_SIZE) {
            if (fwrite(batch, sizeof(BinaryPoint), batch_idx, out) != batch_idx) {
                fclose(in);
                fclose(out);
                return ESP_FAIL;
            }
            batch_idx = 0;
        }
    }

    // Flush remaining points
    if (batch_idx > 0) {
        if (fwrite(batch, sizeof(BinaryPoint), batch_idx, out) != batch_idx) {
            fclose(in);
            fclose(out);
            return ESP_FAIL;
        }
    }

    fclose(in);

    // Seek back and update header with actual point count
    if (fseek(out, 0, SEEK_SET) != 0) {
        fclose(out);
        return ESP_FAIL;
    }

    header.point_count = static_cast<uint32_t>(point_count);
    if (fwrite(&header, sizeof(header), 1, out) != 1) {
        fclose(out);
        return ESP_FAIL;
    }

    fclose(out);
    *out_point_count = point_count;
    return ESP_OK;
}

esp_err_t ConversionExecutor::addPatternToDatabase(const std::string& external_uuid,
                                                    const std::string& name,
                                                    size_t file_size,
                                                    bool encrypted) {
    Pattern pattern;
    pattern.id = 0;  // Will be assigned by TQDB
    pattern.external_uuid = external_uuid;
    pattern.name = name.empty() ? external_uuid : name;
    pattern.creator = "Uploaded";
    pattern.encrypted = encrypted;
    pattern.size_bytes = file_size;
    pattern.created_at = ManifestDatabase::currentTimestamp();

    return ManifestDatabase::instance().addPattern(pattern);
}

} // namespace jobs
