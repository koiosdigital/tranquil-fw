#pragma once

#include "stusb4500.h"
#include "esp_err.h"
#include <cstdint>

struct UsbPdConfig {
    struct Pdo {
        float voltage;
        float current;
    };

    Pdo pdos[3] = {{5.0f, 3.0f}, {15.0f, 3.0f}, {20.0f, 5.0f}};
    uint8_t pdo_count = 3;
    bool usb_comm_capable = true;
    bool external_power = true;
};

class UsbPdController {
public:
    explicit UsbPdController(const UsbPdConfig& config = UsbPdConfig{});
    ~UsbPdController();

    // Non-copyable
    UsbPdController(const UsbPdController&) = delete;
    UsbPdController& operator=(const UsbPdController&) = delete;

    // Movable
    UsbPdController(UsbPdController&& other) noexcept;
    UsbPdController& operator=(UsbPdController&& other) noexcept;

    // Status methods
    bool is_initialized() const noexcept { return initialized_; }
    esp_err_t get_last_error() const noexcept { return last_error_; }

    // Singleton access for global usage pattern
    static UsbPdController& instance();

private:
    STUSB4500 stusb_;
    bool initialized_ = false;
    esp_err_t last_error_ = ESP_OK;

    esp_err_t configure_pdos(const UsbPdConfig& config);
};

// Legacy compatibility function
void stusb_init();
