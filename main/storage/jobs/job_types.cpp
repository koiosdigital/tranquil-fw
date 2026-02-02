#include "job_types.h"
#include "cJSON.h"
#include <cstring>

namespace jobs {

const char* jobTypeToString(JobType type) {
    switch (type) {
        case JobType::Conversion: return "conversion";
        case JobType::Thumbnail:  return "thumbnail";
        case JobType::Download:   return "download";
        default:                  return "unknown";
    }
}

const char* jobStatusToString(JobStatus status) {
    switch (status) {
        case JobStatus::Pending:    return "pending";
        case JobStatus::InProgress: return "in_progress";
        case JobStatus::Completed:  return "completed";
        case JobStatus::Failed:     return "failed";
        default:                    return "unknown";
    }
}

JobType jobTypeFromString(const char* str) {
    if (!str) return JobType::Conversion;
    if (strcmp(str, "conversion") == 0) return JobType::Conversion;
    if (strcmp(str, "thumbnail") == 0)  return JobType::Thumbnail;
    if (strcmp(str, "download") == 0)   return JobType::Download;
    return JobType::Conversion;
}

JobStatus jobStatusFromString(const char* str) {
    if (!str) return JobStatus::Pending;
    if (strcmp(str, "pending") == 0)     return JobStatus::Pending;
    if (strcmp(str, "in_progress") == 0) return JobStatus::InProgress;
    if (strcmp(str, "completed") == 0)   return JobStatus::Completed;
    if (strcmp(str, "failed") == 0)      return JobStatus::Failed;
    return JobStatus::Pending;
}

std::string ConversionJobData::toJson() const {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "temp_path", temp_path.c_str());
    cJSON_AddStringToObject(root, "name", name.c_str());
    cJSON_AddBoolToObject(root, "encrypted", encrypted);

    char* str = cJSON_PrintUnformatted(root);
    std::string result(str);
    free(str);
    cJSON_Delete(root);
    return result;
}

ConversionJobData ConversionJobData::fromJson(const std::string& json) {
    ConversionJobData data;
    cJSON* root = cJSON_Parse(json.c_str());
    if (!root) return data;

    cJSON* temp_path = cJSON_GetObjectItem(root, "temp_path");
    if (cJSON_IsString(temp_path)) {
        data.temp_path = temp_path->valuestring;
    }

    cJSON* name = cJSON_GetObjectItem(root, "name");
    if (cJSON_IsString(name)) {
        data.name = name->valuestring;
    }

    cJSON* encrypted = cJSON_GetObjectItem(root, "encrypted");
    if (cJSON_IsBool(encrypted)) {
        data.encrypted = cJSON_IsTrue(encrypted);
    }

    cJSON_Delete(root);
    return data;
}

std::string ThumbnailJobData::toJson() const {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "encrypted", encrypted);
    cJSON_AddStringToObject(root, "output_path", output_path.c_str());

    char* str = cJSON_PrintUnformatted(root);
    std::string result(str);
    free(str);
    cJSON_Delete(root);
    return result;
}

ThumbnailJobData ThumbnailJobData::fromJson(const std::string& json) {
    ThumbnailJobData data;
    cJSON* root = cJSON_Parse(json.c_str());
    if (!root) return data;

    cJSON* encrypted = cJSON_GetObjectItem(root, "encrypted");
    if (cJSON_IsBool(encrypted)) {
        data.encrypted = cJSON_IsTrue(encrypted);
    }

    cJSON* output_path = cJSON_GetObjectItem(root, "output_path");
    if (cJSON_IsString(output_path)) {
        data.output_path = output_path->valuestring;
    }

    cJSON_Delete(root);
    return data;
}

std::string DownloadJobData::toJson() const {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "download_url", download_url.c_str());
    cJSON_AddStringToObject(root, "pattern_name", pattern_name.c_str());
    cJSON_AddStringToObject(root, "pattern_creator", pattern_creator.c_str());
    cJSON_AddNumberToObject(root, "size_bytes", static_cast<double>(size_bytes));
    cJSON_AddBoolToObject(root, "encrypted", encrypted);
    cJSON_AddBoolToObject(root, "reversible", reversible);
    cJSON_AddNumberToObject(root, "start_point", start_point);
    cJSON_AddStringToObject(root, "created_at", created_at.c_str());

    // Purchase receipt (if present)
    if (!receipt_payload_b64.empty()) {
        cJSON_AddStringToObject(root, "receipt_payload_b64", receipt_payload_b64.c_str());
    }
    if (!receipt_signature_b64.empty()) {
        cJSON_AddStringToObject(root, "receipt_signature_b64", receipt_signature_b64.c_str());
    }

    char* str = cJSON_PrintUnformatted(root);
    std::string result(str);
    free(str);
    cJSON_Delete(root);
    return result;
}

DownloadJobData DownloadJobData::fromJson(const std::string& json) {
    DownloadJobData data;
    cJSON* root = cJSON_Parse(json.c_str());
    if (!root) return data;

    cJSON* item;

    item = cJSON_GetObjectItem(root, "download_url");
    if (cJSON_IsString(item)) data.download_url = item->valuestring;

    item = cJSON_GetObjectItem(root, "pattern_name");
    if (cJSON_IsString(item)) data.pattern_name = item->valuestring;

    item = cJSON_GetObjectItem(root, "pattern_creator");
    if (cJSON_IsString(item)) data.pattern_creator = item->valuestring;

    item = cJSON_GetObjectItem(root, "size_bytes");
    if (cJSON_IsNumber(item)) data.size_bytes = static_cast<int64_t>(item->valuedouble);

    item = cJSON_GetObjectItem(root, "encrypted");
    if (cJSON_IsBool(item)) data.encrypted = cJSON_IsTrue(item);

    item = cJSON_GetObjectItem(root, "reversible");
    if (cJSON_IsBool(item)) data.reversible = cJSON_IsTrue(item);

    item = cJSON_GetObjectItem(root, "start_point");
    if (cJSON_IsNumber(item)) data.start_point = item->valueint;

    item = cJSON_GetObjectItem(root, "created_at");
    if (cJSON_IsString(item)) data.created_at = item->valuestring;

    // Purchase receipt (optional)
    item = cJSON_GetObjectItem(root, "receipt_payload_b64");
    if (cJSON_IsString(item)) data.receipt_payload_b64 = item->valuestring;

    item = cJSON_GetObjectItem(root, "receipt_signature_b64");
    if (cJSON_IsString(item)) data.receipt_signature_b64 = item->valuestring;

    cJSON_Delete(root);
    return data;
}

} // namespace jobs
