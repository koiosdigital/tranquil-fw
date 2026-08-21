#include "uartbus.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include <array>
#include <cstring>

namespace {
constexpr const char* TAG = "UartBus";

// TMC2209 UART protocol constants
constexpr uint8_t kSyncByte = 0x05;
constexpr uint8_t kWriteFlag = 0x80;
constexpr uint8_t kReplyMasterAddr = 0xFF;
constexpr size_t kWritePacketSize = 8;
constexpr size_t kReadRequestSize = 4;
constexpr size_t kReadResponseSize = 8;
constexpr size_t kReadBufferSize = 12;
constexpr uint32_t kTxTimeoutMs = 100;
constexpr uint32_t kRxTimeoutMs = 200;
constexpr uint32_t kResponseDelayUs = 500;
constexpr uint32_t kBusLockTimeoutMs = 1000;

// RAII lock for the shared-bus mutex; releases on every early-return path.
class BusLock {
public:
    BusLock(SemaphoreHandle_t m, TickType_t timeout) noexcept
        : mutex_(m), locked_(m == nullptr || xSemaphoreTake(m, timeout) == pdTRUE) {}
    ~BusLock() { if (mutex_ && locked_) xSemaphoreGive(mutex_); }
    BusLock(const BusLock&) = delete;
    BusLock& operator=(const BusLock&) = delete;
    [[nodiscard]] bool locked() const noexcept { return locked_; }
private:
    SemaphoreHandle_t mutex_;
    bool locked_;
};
} // namespace

namespace tmc {

UartBus::~UartBus() {
    deinitialize();
}

esp_err_t UartBus::initialize(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t baud) {
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    const uart_config_t config = {
        .baud_rate = static_cast<int>(baud),
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };

    if (auto err = uart_param_config(port, &config); err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART params: %s", esp_err_to_name(err));
        return err;
    }

    if (auto err = uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
        return err;
    }

    if (auto err = uart_driver_install(port, kBufferSize, kBufferSize, 0, nullptr, 0); err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
        return err;
    }

    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create bus mutex");
        uart_driver_delete(port);
        return ESP_ERR_NO_MEM;
    }

    uart_port_ = port;
    initialized_ = true;
    ESP_LOGD(TAG, "UART initialized on port %d at %" PRIu32 " baud", port, baud);

    return ESP_OK;
}

void UartBus::deinitialize() noexcept {
    if (!initialized_) {
        return;
    }

    uart_driver_delete(uart_port_);
    if (mutex_) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
    initialized_ = false;
    uart_port_ = UART_NUM_MAX;
}

uint8_t UartBus::calculate_crc(const uint8_t* data, size_t length) noexcept {
    uint8_t crc = 0;

    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const bool mix = ((crc >> 7) ^ (byte & 1)) != 0;
            crc <<= 1;
            if (mix) {
                crc ^= 0x07;
            }
            byte >>= 1;
        }
    }

    return crc;
}

esp_err_t UartBus::write_register(uint8_t addr, uint8_t reg, uint32_t value) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    BusLock lock(mutex_, pdMS_TO_TICKS(kBusLockTimeoutMs));
    if (!lock.locked()) {
        ESP_LOGW(TAG, "Bus busy: write addr=%u reg=0x%02X timed out", addr, reg);
        return ESP_ERR_TIMEOUT;
    }

    std::array<uint8_t, kWritePacketSize> packet = {
        kSyncByte,
        addr,
        static_cast<uint8_t>(reg | kWriteFlag),
        static_cast<uint8_t>((value >> 24) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF),
        0  // CRC placeholder
    };
    packet[7] = calculate_crc(packet.data(), 7);

    uart_flush_input(uart_port_);

    const int written = uart_write_bytes(uart_port_, packet.data(), packet.size());
    if (written != static_cast<int>(packet.size())) {
        return ESP_ERR_TIMEOUT;
    }

    if (auto err = uart_wait_tx_done(uart_port_, pdMS_TO_TICKS(kTxTimeoutMs)); err != ESP_OK) {
        return err;
    }

    ESP_LOGD(TAG, "Write addr=%u reg=0x%02X value=0x%08lX", addr, reg, value);
    return ESP_OK;
}

std::optional<uint32_t> UartBus::read_register(uint8_t addr, uint8_t reg) {
    if (!initialized_) {
        return std::nullopt;
    }

    BusLock lock(mutex_, pdMS_TO_TICKS(kBusLockTimeoutMs));
    if (!lock.locked()) {
        ESP_LOGW(TAG, "Bus busy: read addr=%u reg=0x%02X timed out", addr, reg);
        return std::nullopt;
    }

    // Build read request packet
    std::array<uint8_t, kReadRequestSize> request = {
        kSyncByte,
        addr,
        reg,
        0  // CRC placeholder
    };
    request[3] = calculate_crc(request.data(), 3);

    uart_flush_input(uart_port_);

    const int written = uart_write_bytes(uart_port_, request.data(), request.size());
    if (written != static_cast<int>(request.size())) {
        return std::nullopt;
    }

    if (uart_wait_tx_done(uart_port_, pdMS_TO_TICKS(kTxTimeoutMs)) != ESP_OK) {
        return std::nullopt;
    }

    // Wait for response
    esp_rom_delay_us(kResponseDelayUs);

    std::array<uint8_t, kReadBufferSize> buffer{};
    const int read_bytes = uart_read_bytes(uart_port_, buffer.data(), buffer.size(), pdMS_TO_TICKS(kRxTimeoutMs));

    if (read_bytes < static_cast<int>(kReadResponseSize)) {
        return std::nullopt;
    }

    // Find response packet (starts with sync byte and master address 0xFF)
    const uint8_t* response = nullptr;
    for (int i = 0; i <= read_bytes - static_cast<int>(kReadResponseSize); ++i) {
        if (buffer[i] == kSyncByte && buffer[i + 1] == kReplyMasterAddr) {
            response = &buffer[i];
            break;
        }
    }

    if (!response) {
        return std::nullopt;
    }

    // Validate CRC
    if (calculate_crc(response, 7) != response[7]) {
        return std::nullopt;
    }

    const uint32_t value = (static_cast<uint32_t>(response[3]) << 24) |
                           (static_cast<uint32_t>(response[4]) << 16) |
                           (static_cast<uint32_t>(response[5]) << 8) |
                           static_cast<uint32_t>(response[6]);

    ESP_LOGD(TAG, "Read addr=%u reg=0x%02X value=0x%08lX", addr, reg, value);
    return value;
}

esp_err_t UartBus::read_register(uint8_t addr, uint8_t reg, uint32_t* out_value) {
    if (!out_value) {
        return ESP_ERR_INVALID_ARG;
    }

    auto result = read_register(addr, reg);
    if (!result) {
        return ESP_ERR_NOT_FOUND;
    }

    *out_value = *result;
    return ESP_OK;
}

} // namespace tmc
