#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <cstdint>
#include <optional>

namespace tmc {

class UartBus {
public:
    static constexpr uint32_t kDefaultBaudRate = 115200;
    static constexpr size_t kBufferSize = 1024;

    UartBus() = default;
    ~UartBus();

    // Non-copyable, non-moveable (owns hardware resource)
    UartBus(const UartBus&) = delete;
    UartBus& operator=(const UartBus&) = delete;
    UartBus(UartBus&&) = delete;
    UartBus& operator=(UartBus&&) = delete;

    [[nodiscard]] esp_err_t initialize(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin,
                                        uint32_t baud = kDefaultBaudRate);
    void deinitialize() noexcept;

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }
    [[nodiscard]] uart_port_t port() const noexcept { return uart_port_; }

    [[nodiscard]] esp_err_t write_register(uint8_t addr, uint8_t reg, uint32_t value);
    [[nodiscard]] std::optional<uint32_t> read_register(uint8_t addr, uint8_t reg);

    // Legacy API compatibility
    [[nodiscard]] esp_err_t read_register(uint8_t addr, uint8_t reg, uint32_t* out_value);

private:
    [[nodiscard]] static uint8_t calculate_crc(const uint8_t* data, size_t length) noexcept;

    uart_port_t uart_port_ = UART_NUM_MAX;
    bool initialized_ = false;

    // Serializes whole read/write transactions on the shared half-duplex bus.
    // Both motor drivers and the status getters (is_stalled / get_*_current)
    // share one UartBus; without this a status read from the API task could
    // interleave with an apply_motor_config() write and corrupt a transaction
    // (mismatched reply framing, wrong CRC).
    SemaphoreHandle_t mutex_ = nullptr;
};

} // namespace tmc

// Backwards compatibility alias
using UartBus = tmc::UartBus;
