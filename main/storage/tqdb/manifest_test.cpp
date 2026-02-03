/**
 * @file manifest_test.cpp
 * @brief ManifestDatabase self-test implementation
 */

#include "manifest_internal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>
#include <string>

static const char* TAG = MANIFEST_TAG;

// ─────────────────────────────────────────────────────────────────────────────
// Self-test
// ─────────────────────────────────────────────────────────────────────────────

namespace {
    struct SelfTestContext {
        ManifestDatabase* db;
        int iterations;
        esp_err_t result;
        SemaphoreHandle_t done;
    };

    void selfTestTask(void* arg) {
        auto* ctx = static_cast<SelfTestContext*>(arg);
        auto* db = ctx->db;
        int iterations = ctx->iterations;

        ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
        ESP_LOGI(TAG, "ManifestDatabase Self-Test: %d iterations (TQDB backend)", iterations);
        ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");

        size_t initial_patterns = db->getPatternCount();
        size_t initial_playlists = db->getPlaylistCount();
        size_t initial_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);

        std::vector<uint32_t> test_pattern_ids;
        std::vector<uint32_t> test_playlist_ids;

        int64_t t_start, t_end;
        int64_t pattern_create_us = 0, pattern_read_us = 0, pattern_update_us = 0, pattern_delete_us = 0;
        int64_t playlist_create_us = 0, playlist_read_us = 0, playlist_update_us = 0, playlist_delete_us = 0;
        int errors = 0;

        // Pattern CREATE
        ESP_LOGI(TAG, "--- Pattern CREATE x%d ---", iterations);
        t_start = esp_timer_get_time();
        for (int i = 0; i < iterations; i++) {
            Pattern p;
            p.name = "Test Pattern " + std::to_string(i);
            p.creator = "SelfTest";
            p.popularity = i;
            p.reversible = (i % 2) == 0;
            p.encrypted = false;
            p.size_bytes = 1000 + i;

            esp_err_t rc = db->addPattern(p);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "Pattern CREATE failed at i=%d: %d", i, rc);
                errors++;
            }
            else {
                test_pattern_ids.push_back(p.id);
            }

            if ((i + 1) % 10 == 0) {
                ESP_LOGI(TAG, "  Created %d patterns, heap=%zu", i + 1,
                    heap_caps_get_free_size(MALLOC_CAP_8BIT));
            }
        }
        t_end = esp_timer_get_time();
        pattern_create_us = t_end - t_start;

        // Pattern READ
        ESP_LOGI(TAG, "--- Pattern READ x%d ---", iterations);
        t_start = esp_timer_get_time();
        for (int i = 0; i < iterations && i < (int)test_pattern_ids.size(); i++) {
            auto p = db->getPattern(test_pattern_ids[i]);
            if (!p.has_value()) {
                ESP_LOGE(TAG, "Pattern READ failed at i=%d", i);
                errors++;
            }
        }
        t_end = esp_timer_get_time();
        pattern_read_us = t_end - t_start;

        // Pattern UPDATE
        ESP_LOGI(TAG, "--- Pattern UPDATE x%d ---", iterations);
        t_start = esp_timer_get_time();
        for (int i = 0; i < iterations && i < (int)test_pattern_ids.size(); i++) {
            Pattern p;
            p.name = "Updated Pattern " + std::to_string(i);
            p.creator = "SelfTest-Updated";
            p.popularity = i * 10;
            p.reversible = (i % 2) != 0;
            p.size_bytes = 2000 + i;

            esp_err_t rc = db->updatePattern(test_pattern_ids[i], p);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "Pattern UPDATE failed at i=%d: %d", i, rc);
                errors++;
            }
        }
        t_end = esp_timer_get_time();
        pattern_update_us = t_end - t_start;

        // Playlist tests
        ESP_LOGI(TAG, "--- Playlist CREATE x%d ---", iterations);
        t_start = esp_timer_get_time();
        for (int i = 0; i < iterations; i++) {
            Playlist pl;
            pl.name = "Test Playlist " + std::to_string(i);
            pl.description = "Self-test playlist";

            for (int j = 0; j < std::min(3, (int)test_pattern_ids.size()); j++) {
                pl.pattern_ids.push_back(test_pattern_ids[j]);
            }

            esp_err_t rc = db->addPlaylist(pl);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "Playlist CREATE failed at i=%d: %d", i, rc);
                errors++;
            }
            else {
                test_playlist_ids.push_back(pl.id);
            }
        }
        t_end = esp_timer_get_time();
        playlist_create_us = t_end - t_start;

        ESP_LOGI(TAG, "--- Playlist READ x%d ---", iterations);
        t_start = esp_timer_get_time();
        for (int i = 0; i < iterations && i < (int)test_playlist_ids.size(); i++) {
            auto pl = db->getPlaylist(test_playlist_ids[i]);
            if (!pl.has_value()) {
                ESP_LOGE(TAG, "Playlist READ failed at i=%d", i);
                errors++;
            }
        }
        t_end = esp_timer_get_time();
        playlist_read_us = t_end - t_start;

        ESP_LOGI(TAG, "--- Playlist UPDATE x%d ---", iterations);
        t_start = esp_timer_get_time();
        for (int i = 0; i < iterations && i < (int)test_playlist_ids.size(); i++) {
            Playlist pl;
            pl.name = "Updated Playlist " + std::to_string(i);
            pl.description = "Updated description";
            for (int j = 0; j < std::min(3, (int)test_pattern_ids.size()); j++) {
                pl.pattern_ids.push_back(test_pattern_ids[j]);
            }

            esp_err_t rc = db->updatePlaylist(test_playlist_ids[i], pl);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "Playlist UPDATE failed at i=%d: %d", i, rc);
                errors++;
            }
        }
        t_end = esp_timer_get_time();
        playlist_update_us = t_end - t_start;

        // Cleanup
        ESP_LOGI(TAG, "--- Playlist DELETE x%d ---", (int)test_playlist_ids.size());
        t_start = esp_timer_get_time();
        for (uint32_t id : test_playlist_ids) {
            esp_err_t rc = db->deletePlaylist(id);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "Playlist DELETE failed: %d", rc);
                errors++;
            }
        }
        t_end = esp_timer_get_time();
        playlist_delete_us = t_end - t_start;

        ESP_LOGI(TAG, "--- Pattern DELETE x%d ---", (int)test_pattern_ids.size());
        t_start = esp_timer_get_time();
        for (uint32_t id : test_pattern_ids) {
            esp_err_t rc = db->deletePattern(id);
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "Pattern DELETE failed: %d", rc);
                errors++;
            }
        }
        t_end = esp_timer_get_time();
        pattern_delete_us = t_end - t_start;

        // Verify cleanup
        size_t final_patterns = db->getPatternCount();
        size_t final_playlists = db->getPlaylistCount();
        if (final_patterns != initial_patterns) {
            ESP_LOGE(TAG, "Pattern cleanup failed: expected %zu, got %zu", initial_patterns, final_patterns);
            errors++;
        }
        if (final_playlists != initial_playlists) {
            ESP_LOGE(TAG, "Playlist cleanup failed: expected %zu, got %zu", initial_playlists, final_playlists);
            errors++;
        }

        size_t final_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        int heap_diff = (int)initial_heap - (int)final_heap;

        ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
        ESP_LOGI(TAG, "RESULTS (%d iterations each):", iterations);
        ESP_LOGI(TAG, "───────────────────────────────────────────────────────────");
        ESP_LOGI(TAG, "Pattern  CREATE: %6lld ms total, %4lld us/op",
            pattern_create_us / 1000, pattern_create_us / iterations);
        ESP_LOGI(TAG, "Pattern  READ:   %6lld ms total, %4lld us/op",
            pattern_read_us / 1000, pattern_read_us / iterations);
        ESP_LOGI(TAG, "Pattern  UPDATE: %6lld ms total, %4lld us/op",
            pattern_update_us / 1000, pattern_update_us / iterations);
        ESP_LOGI(TAG, "Pattern  DELETE: %6lld ms total, %4lld us/op",
            pattern_delete_us / 1000, pattern_delete_us / iterations);
        ESP_LOGI(TAG, "───────────────────────────────────────────────────────────");
        ESP_LOGI(TAG, "Playlist CREATE: %6lld ms total, %4lld us/op",
            playlist_create_us / 1000, playlist_create_us / iterations);
        ESP_LOGI(TAG, "Playlist READ:   %6lld ms total, %4lld us/op",
            playlist_read_us / 1000, playlist_read_us / iterations);
        ESP_LOGI(TAG, "Playlist UPDATE: %6lld ms total, %4lld us/op",
            playlist_update_us / 1000, playlist_update_us / iterations);
        ESP_LOGI(TAG, "Playlist DELETE: %6lld ms total, %4lld us/op",
            playlist_delete_us / 1000, playlist_delete_us / iterations);
        ESP_LOGI(TAG, "───────────────────────────────────────────────────────────");
        ESP_LOGI(TAG, "DRAM delta: %+d bytes (%s)", heap_diff, heap_diff <= 1024 ? "OK" : "LEAK?");
        ESP_LOGI(TAG, "Errors: %d", errors);
        ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");

        if (errors > 0) {
            ESP_LOGE(TAG, "SELF-TEST FAILED with %d errors", errors);
            ctx->result = ESP_FAIL;
        }
        else {
            ESP_LOGI(TAG, "SELF-TEST PASSED");
            ctx->result = ESP_OK;
        }

        xSemaphoreGive(ctx->done);
        vTaskDelete(nullptr);
    }
}

esp_err_t ManifestDatabase::selfTest(int iterations) {
    SelfTestContext ctx;
    ctx.db = this;
    ctx.iterations = iterations;
    ctx.result = ESP_FAIL;
    ctx.done = xSemaphoreCreateBinary();

    if (!ctx.done) {
        ESP_LOGE(TAG, "Failed to create semaphore for self-test");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t rc = xTaskCreate(
        selfTestTask,
        "db_selftest",
        8192,
        &ctx,
        5,
        nullptr
    );

    if (rc != pdPASS) {
        vSemaphoreDelete(ctx.done);
        ESP_LOGE(TAG, "Failed to create self-test task");
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(ctx.done, portMAX_DELAY);
    vSemaphoreDelete(ctx.done);

    return ctx.result;
}
