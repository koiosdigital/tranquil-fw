/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_client.h
 * @brief TEE Client API - Phase 1 World Controller Test
 *
 * This module provides the client-side interface for testing
 * World Controller switching between app (World 1) and TEE (World 0).
 *
 * Usage:
 *   1. Call tee_init() once at app startup to switch from W0 to W1
 *   2. Call tee_test_call() to verify W1->W0->W1 round-trip
 *   3. Check tee_get_call_count() to verify TEE processed the call
 */

#ifndef TEE_CLIENT_H
#define TEE_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "tee_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize TEE client and switch to World 1
 *
 * The app starts in World 0 (set by bootloader). This function:
 * 1. Verifies the TEE API table is valid
 * 2. Configures WCL for W0->W1 transition
 * 3. Executes the world switch
 * 4. Returns in World 1
 *
 * Must be called once before any TEE service calls.
 *
 * @return TEE_OK on success, negative error code on failure
 */
int tee_init(void);

/**
 * @brief Check if app is running in World 1
 *
 * @return true if world switch has been performed
 */
bool tee_is_world1(void);

/**
 * @brief Test call to TEE (W1->W0->W1 round-trip)
 *
 * Sends a request to the TEE test handler:
 * - Input: request value
 * - Output: request + 0x1000
 *
 * This call triggers:
 * 1. W1->W0 switch (via WCL entry point)
 * 2. TEE handler execution
 * 3. W0->W1 switch (via trampoline)
 *
 * @param input Value to send to TEE
 * @return Response from TEE (input + 0x1000), or negative error code
 */
int32_t tee_test_call(uint32_t input);

/**
 * @brief Get TEE call counter
 *
 * Returns the number of times TEE handlers have been invoked.
 * Useful for verifying world switching works.
 *
 * @return Call count, or 0 if TEE not initialized
 */
uint32_t tee_get_call_count(void);

/**
 * @brief Get last TEE result code
 *
 * @return Last result from TEE handler
 */
int32_t tee_get_last_result(void);

#ifdef __cplusplus
}
#endif

#endif /* TEE_CLIENT_H */
