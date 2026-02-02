#include "usb_pd.h"

#include "esp_log.h"
#include "esp_system.h"
#include <cmath>

static const char* TAG = "usb_pd";

UsbPdController::UsbPdController(const UsbPdConfig& config) {
    last_error_ = stusb_.begin();
    if (last_error_ != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STUSB4500: %s", esp_err_to_name(last_error_));
        return;
    }

    last_error_ = configure_pdos(config);
    if (last_error_ != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PDOs: %s", esp_err_to_name(last_error_));
        return;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "USB-PD controller initialized successfully");
}

UsbPdController::~UsbPdController() {
    if (initialized_) {
        stusb_.end();
        initialized_ = false;
    }
}

UsbPdController::UsbPdController(UsbPdController&& other) noexcept
    : stusb_(std::move(other.stusb_))
    , initialized_(other.initialized_)
    , last_error_(other.last_error_) {
    other.initialized_ = false;
}

UsbPdController& UsbPdController::operator=(UsbPdController&& other) noexcept {
    if (this != &other) {
        if (initialized_) {
            stusb_.end();
        }
        stusb_ = std::move(other.stusb_);
        initialized_ = other.initialized_;
        last_error_ = other.last_error_;
        other.initialized_ = false;
    }
    return *this;
}

esp_err_t UsbPdController::configure_pdos(const UsbPdConfig& config) {
    // Read current NVM configuration
    esp_err_t ret = stusb_.read_nvm();
    bool need_nvm_update = (ret != ESP_OK);

    if (!need_nvm_update) {
        // Check if current configuration matches desired
        for (int i = 0; i < config.pdo_count; ++i) {
            float voltage = stusb_.get_voltage(i + 1);
            float current = stusb_.get_current(i + 1);

            if (std::fabs(voltage - config.pdos[i].voltage) > 0.1f ||
                std::fabs(current - config.pdos[i].current) > 0.1f) {
                need_nvm_update = true;
                break;
            }
        }
    }

    if (need_nvm_update) {
        ESP_LOGI(TAG, "Updating NVM with PDO configuration");

        for (int i = 0; i < config.pdo_count; ++i) {
            (void)stusb_.set_voltage(i + 1, config.pdos[i].voltage);
            (void)stusb_.set_current(i + 1, config.pdos[i].current);
        }

        (void)stusb_.set_pdo_number(config.pdo_count);
        (void)stusb_.set_usb_comm_capable(config.usb_comm_capable);
        (void)stusb_.set_external_power(config.external_power);
        (void)stusb_.write_nvm();

        ESP_LOGI(TAG, "NVM updated: %d PDOs configured", config.pdo_count);
    }
    else {
        ESP_LOGI(TAG, "NVM configuration already matches desired settings");
    }

    return ESP_OK;
}

UsbPdController& UsbPdController::instance() {
    static UsbPdController controller;
    return controller;
}

// Legacy compatibility function
void stusb_init() {
    auto& controller = UsbPdController::instance();
    if (!controller.is_initialized()) {
        ESP_LOGE(TAG, "USB-PD initialization failed: %s",
            esp_err_to_name(controller.get_last_error()));
        esp_restart();
    }
}
