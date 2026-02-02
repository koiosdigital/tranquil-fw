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
        return JobResult::fail("Missing temp_path in job data");
    }

    // Verify input file exists
    struct stat st;
    if (stat(data.temp_path.c_str(), &st) != 0) {
        return JobResult::fail("Input file not found: " + data.temp_path);
    }

    // Build output paths
    char binary_path[128];
    char final_path[128];
    snprintf(binary_path, sizeof(binary_path), "/sd/patterns/%s.thrb", job.pattern_uuid.c_str());
    snprintf(final_path, sizeof(final_path), "/sd/patterns/%s.dat", job.pattern_uuid.c_str());

    // Perform conversion
    size_t point_count = 0;
    esp_err_t result = convertToBinary(data.temp_path.c_str(), binary_path, &point_count);

    if (result != ESP_OK || point_count == 0) {
        unlink(data.temp_path.c_str());
        unlink(binary_path);
        if (point_count == 0) {
            return JobResult::fail("No valid points found in input file");
        }
        return JobResult::fail("Conversion failed");
    }

    // Atomic rename to final path
    if (rename(binary_path, final_path) != 0) {
        unlink(data.temp_path.c_str());
        unlink(binary_path);
        return JobResult::fail("Failed to rename to final path");
    }

    // Get final file size
    size_t file_size = 0;
    if (stat(final_path, &st) == 0) {
        file_size = st.st_size;
    }

    // Add to database
    esp_err_t db_result = addPatternToDatabase(job.pattern_uuid, data.name, file_size, data.encrypted);
    if (db_result != ESP_OK) {
        unlink(final_path);
        unlink(data.temp_path.c_str());
        return JobResult::fail("Failed to add pattern to database");
    }

    // Success - delete temp file
    unlink(data.temp_path.c_str());

    ESP_LOGD(TAG, "Conversion complete: %s (%zu points)", job.pattern_uuid.c_str(), point_count);

    // Enqueue thumbnail generation job
    ThumbnailJobData thumb_data;
    thumb_data.encrypted = data.encrypted;
    thumb_data.output_path = "/sd/previews/" + job.pattern_uuid + ".png";

    JobQueue::instance().enqueueThumbnail(job.pattern_uuid, thumb_data, -1);

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

esp_err_t ConversionExecutor::addPatternToDatabase(const std::string& uuid,
                                                    const std::string& name,
                                                    size_t file_size,
                                                    bool encrypted) {
    Pattern pattern;
    pattern.uuid = uuid;
    pattern.name = name.empty() ? uuid : name;
    pattern.creator = "Uploaded";
    pattern.encrypted = encrypted;
    pattern.size_bytes = file_size;
    pattern.created_at = ManifestDatabase::currentTimestamp();

    return ManifestDatabase::instance().addPattern(pattern);
}

} // namespace jobs
