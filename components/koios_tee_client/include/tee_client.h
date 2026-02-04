/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_client.h
 * @brief TEE client API for application code
 *
 * Provides functions for the application (running in World 1) to
 * call TEE services (running in World 0).
 */

#ifndef TEE_CLIENT_H
#define TEE_CLIENT_H

#include "tee_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize TEE client
 *
 * Verifies that the TEE was properly initialized by the bootloader
 * by checking the API magic and version.
 *
 * @return TEE_OK on success, error code otherwise
 */
tee_error_t tee_client_init(void);

/**
 * @brief Check if TEE is available
 *
 * @return true if TEE API is valid and available
 */
bool tee_client_available(void);

/**
 * @brief Call a TEE service
 *
 * Performs a world switch to W0, executes the service, and returns.
 *
 * @param service_id Service to invoke (see tee_service_id_t)
 * @return Result code from TEE handler
 */
int32_t tee_call(tee_service_id_t service_id);

/**
 * @brief Call a TEE service with parameters
 *
 * @param service_id Service to invoke
 * @param params Array of up to 8 parameters (NULL for no params)
 * @param num_params Number of parameters
 * @return Result code from TEE handler
 */
int32_t tee_call_params(tee_service_id_t service_id,
                        const uint32_t* params,
                        size_t num_params);

/**
 * @brief Get TEE call count
 *
 * Returns the number of successful TEE calls. Useful for debugging
 * and verifying TEE operation.
 *
 * @return Number of TEE calls made
 */
uint32_t tee_get_call_count(void);

/**
 * @brief Get last TEE result
 *
 * @return Result code from last TEE call
 */
int32_t tee_get_last_result(void);

/**
 * @brief Get TEE debug stage
 *
 * Returns the debug stage counter from the TEE. Useful for
 * diagnosing crashes during TEE calls.
 *
 * @return Debug stage value (0-11, see tee_entry_stub.S)
 */
uint32_t tee_get_debug_stage(void);

/**
 * @brief Check if currently in World 1 (non-secure)
 *
 * @return true if executing in World 1
 */
bool tee_is_world1(void);

#ifdef __cplusplus
}
#endif

#endif /* TEE_CLIENT_H */
