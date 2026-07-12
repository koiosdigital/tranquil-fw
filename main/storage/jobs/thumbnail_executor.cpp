#include "thumbnail_executor.h"
#include "job_queue.h"
#include "PatternReader.h"
#include "ManifestDatabase.h"
#include "thumbnail/framebuffer.h"
#include "thumbnail/bezier_renderer.h"
#include "thumbnail/png_encoder.h"
#include "esp_log.h"
#include <cerrno>
#include <cstdio>
#include <sys/stat.h>
#include <unistd.h>
#include <memory>

static const char* TAG = "ThumbnailExecutor";

namespace jobs {

    JobResult ThumbnailExecutor::execute(const Job& job) {
        // Get pattern by internal ID to find external_uuid — a row that doesn't
        // exist now will not exist on retry either
        auto pattern = ManifestDatabase::instance().getPattern(job.pattern_id);
        if (!pattern) {
            return JobResult::fail_permanent("Pattern not found");
        }
        const std::string& pattern_uuid = pattern->external_uuid;

        // Parse job data
        ThumbnailJobData data = ThumbnailJobData::fromJson(job.job_data);

        if (data.output_path.empty()) {
            // Default output path using external_uuid
            data.output_path = "/sd/previews/" + pattern_uuid + ".png";
        }

        // Ensure previews directory exists
        struct stat st = { 0 };
        if (stat("/sd/previews", &st) == -1) {
            if (mkdir("/sd/previews", 0775) != 0) {
                return JobResult::fail("Failed to create previews directory");
            }
        }

        // Render the pattern using external_uuid (file path)
        return renderPattern(pattern_uuid, data.encrypted, data.output_path);
    }

    JobResult ThumbnailExecutor::renderPattern(const std::string& pattern_uuid,
        bool encrypted,
        const std::string& output_path) {
        ESP_LOGD(TAG, "Rendering thumbnail for pattern: %s (encrypted=%d)",
            pattern_uuid.c_str(), encrypted);

        // Create pattern reader
        auto reader = createPatternReader(encrypted);
        if (!reader) {
            return JobResult::fail("Failed to create pattern reader");
        }

        // Open pattern file
        esp_err_t err = reader->open(pattern_uuid.c_str());
        if (err != ESP_OK) {
            // If the .dat file is simply gone, no amount of retrying will bring
            // it back — fail permanently instead of burning retries.
            char dat_path[128];
            snprintf(dat_path, sizeof(dat_path), "/sd/patterns/%s.dat", pattern_uuid.c_str());
            struct stat dat_st;
            errno = 0;
            if (stat(dat_path, &dat_st) != 0 && errno == ENOENT) {
                return JobResult::fail_permanent("Pattern file missing: " + std::string(dat_path));
            }
            return JobResult::fail("Failed to open pattern file");
        }

        size_t total_points = reader->getTotalLines();
        if (total_points == 0) {
            reader->close();
            return JobResult::fail_permanent("Pattern has no points");
        }

        ESP_LOGD(TAG, "Pattern has %zu points", total_points);

        // Allocate framebuffer in SPIRAM
        auto fb = std::make_unique<thumbnail::Framebuffer>();
        if (!fb->isValid()) {
            reader->close();
            return JobResult::fail("Failed to allocate framebuffer (SPIRAM)");
        }

        // Create renderer
        thumbnail::BezierRenderer renderer(*fb);
        renderer.beginPath();

        // Stream all points through renderer
        // Use subsampling for very large patterns to keep memory/time reasonable
        size_t step = 1;
        if (total_points > 1000000) {
            step = total_points / 1000000;  // Limit to ~1M points for rendering
        }

        size_t rendered_count = 0;
        size_t point_index = 0;

        while (reader->hasMore()) {
            PatternPoint pt = reader->readNext();
            if (!pt.valid) break;

            // Only render every 'step' points for large patterns
            if (point_index % step == 0) {
                thumbnail::PolarPoint polar;
                polar.theta = static_cast<float>(pt.theta);
                polar.rho = static_cast<float>(pt.rho);
                renderer.addPolarPoint(polar);
                rendered_count++;
            }
            point_index++;
        }

        renderer.endPath();
        reader->close();

        ESP_LOGD(TAG, "Rendered %zu points (total %zu, step %zu)",
            rendered_count, total_points, step);

        // Encode to PNG
        thumbnail::ThumbnailError png_err = thumbnail::PngEncoder::encodeToFile(*fb, output_path.c_str());
        if (png_err != thumbnail::ThumbnailError::OK) {
            return JobResult::fail("Failed to encode PNG");
        }

        // Get output file size for logging
        struct stat st;
        if (stat(output_path.c_str(), &st) == 0) {
            ESP_LOGD(TAG, "Thumbnail saved: %s (%ld bytes)", output_path.c_str(), (long)st.st_size);
        }

        return JobResult::ok();
    }

} // namespace jobs
