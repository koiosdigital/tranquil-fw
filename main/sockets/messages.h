// Outbound message helpers for cloud WebSocket client
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <kd/v1/tranquil.pb-c.h>

// Initialize message system with outbox queue
void cloud_msg_init(QueueHandle_t outbox);

// Queue a protobuf message for sending
bool cloud_msg_queue_raw(const uint8_t* data, size_t len);
bool cloud_msg_queue(const Kd__V1__TranquilMessage* message);

// Send device info to cloud
void cloud_msg_send_device_info();

// Upload coredump if present
void cloud_msg_upload_coredump();

// Send claim request if device needs claiming
void cloud_msg_send_claim_if_needed();

// Send current certificate to server for expiry check
void cloud_msg_send_cert_report();

// Send certificate renewal request with CSR
void cloud_msg_send_cert_renew_request(const char* csr, size_t csr_len);

// Request license from cloud server
void cloud_msg_send_license_request();

// Request pattern download from cloud
void cloud_msg_send_pattern_download_request(const char* pattern_uuid);

// Request sync of all purchase receipts from cloud
void cloud_msg_send_sync_purchases_request();
