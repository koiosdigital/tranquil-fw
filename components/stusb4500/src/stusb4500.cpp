#include "stusb4500.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {
constexpr const char* TAG = "STUSB4500";

// CC Status register bit masks
constexpr uint8_t kCcStatusCc1Mask = 0x03;
constexpr uint8_t kCcStatusCc2Mask = 0x0C;
constexpr uint8_t kCcStatusCc2Shift = 2;
constexpr uint8_t kCcStatusConnectResult = 0x10;
constexpr uint8_t kCcStatusLooking4Connection = 0x20;

// PD TypeC Status register bit masks
constexpr uint8_t kPdTypecHandshakeCheck = 0x80;
constexpr uint8_t kPdTypecFsmStateMask = 0x1F;

// PRT Status register bit masks
constexpr uint8_t kPrtHwResetReceived = 0x01;
constexpr uint8_t kPrtSoftResetReceived = 0x02;
constexpr uint8_t kPrtDataRole = 0x04;
constexpr uint8_t kPrtPowerRole = 0x08;
constexpr uint8_t kPrtPdContract = 0x10;
constexpr uint8_t kPrtStartupPower = 0x20;
constexpr uint8_t kPrtMsgReceived = 0x40;
constexpr uint8_t kPrtMsgSent = 0x80;

// VSAFE0V threshold
constexpr uint8_t kVsafe0vThreshold = 0x01;

// I2C timeouts
constexpr uint32_t kI2cTimeoutMs = 250;
constexpr uint32_t kNvmDefaultTimeoutMs = 100;
constexpr uint32_t kNvmWriteTimeoutMs = 500;
} // namespace

namespace stusb {

// Default NVM values from SparkFun library
const STUSB4500::NvmSectors STUSB4500::kDefaultNvmSectors = {{
    {0x00, 0x00, 0xB0, 0xAA, 0x00, 0x45, 0x00, 0x00},
    {0x10, 0x40, 0x9C, 0x1C, 0xFF, 0x01, 0x3C, 0xDF},
    {0x02, 0x40, 0x0F, 0x00, 0x32, 0x00, 0xFC, 0xF1},
    {0x00, 0x19, 0x56, 0xAF, 0xF5, 0x35, 0x5F, 0x00},
    {0x00, 0x4B, 0x90, 0x21, 0x43, 0x00, 0x40, 0xFB}
}};

STUSB4500::~STUSB4500() {
    end();
}

STUSB4500::STUSB4500(STUSB4500&& other) noexcept
    : i2c_bus_handle_(other.i2c_bus_handle_)
    , device_handle_(other.device_handle_)
    , device_address_(other.device_address_)
    , is_initialized_(other.is_initialized_)
    , nvm_sectors_read_(other.nvm_sectors_read_)
    , nvm_sectors_(other.nvm_sectors_)
{
    other.i2c_bus_handle_ = nullptr;
    other.device_handle_ = nullptr;
    other.is_initialized_ = false;
    other.nvm_sectors_read_ = false;
}

STUSB4500& STUSB4500::operator=(STUSB4500&& other) noexcept {
    if (this != &other) {
        end();
        i2c_bus_handle_ = other.i2c_bus_handle_;
        device_handle_ = other.device_handle_;
        device_address_ = other.device_address_;
        is_initialized_ = other.is_initialized_;
        nvm_sectors_read_ = other.nvm_sectors_read_;
        nvm_sectors_ = other.nvm_sectors_;

        other.i2c_bus_handle_ = nullptr;
        other.device_handle_ = nullptr;
        other.is_initialized_ = false;
        other.nvm_sectors_read_ = false;
    }
    return *this;
}

esp_err_t STUSB4500::begin(uint8_t device_address) {
    if (is_initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    device_address_ = device_address;

    // Configure I2C master bus
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = CONFIG_STUSB4500_I2C_PORT,
        .sda_io_num = static_cast<gpio_num_t>(CONFIG_STUSB4500_SDA_PIN),
        .scl_io_num = static_cast<gpio_num_t>(CONFIG_STUSB4500_SCL_PIN),
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
    };

    if (auto ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle_); ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure device
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address_,
        .scl_speed_hz = 100000,
    };

    if (auto ret = i2c_master_bus_add_device(i2c_bus_handle_, &device_config, &device_handle_); ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(i2c_bus_handle_);
        i2c_bus_handle_ = nullptr;
        return ret;
    }

    // Verify device ID with retries
    std::optional<uint8_t> device_id;
    for (int attempt = 0; attempt < 3; ++attempt) {
        device_id = read_device_id();
        if (device_id && *device_id != 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!device_id || *device_id == 0) {
        end();
        return ESP_ERR_NOT_FOUND;
    }

    if (*device_id != device::kEvalDeviceId && *device_id != device::kProdDeviceId) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X", *device_id);
        end();
        return ESP_ERR_NOT_FOUND;
    }

    (void)clear_alert_status();  // Best-effort cleanup, ignore errors
    is_initialized_ = true;
    ESP_LOGI(TAG, "Initialized (Device ID: 0x%02X)", *device_id);

    // Load NVM on initialization
    if (auto ret = read_nvm(); ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read NVM: %s", esp_err_to_name(ret));
    }

    return ESP_OK;
}

esp_err_t STUSB4500::end() noexcept {
    if (!is_initialized_) {
        return ESP_OK;
    }

    if (device_handle_) {
        i2c_master_bus_rm_device(device_handle_);
        device_handle_ = nullptr;
    }

    if (i2c_bus_handle_) {
        i2c_del_master_bus(i2c_bus_handle_);
        i2c_bus_handle_ = nullptr;
    }

    is_initialized_ = false;
    nvm_sectors_read_ = false;
    return ESP_OK;
}

std::optional<uint8_t> STUSB4500::read_device_id() {
    return i2c_read_register(reg::kDeviceId);
}

esp_err_t STUSB4500::read_nvm() {
    if (!is_initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (auto ret = nvm_enter_read_mode(); ret != ESP_OK) {
        return ret;
    }

    for (uint8_t i = 0; i < nvm::kSectorCount; ++i) {
        if (auto ret = nvm_read_sector(i, nvm_sectors_[i].data()); ret != ESP_OK) {
            (void)nvm_exit_test_mode();  // Best-effort cleanup on error
            return ret;
        }
    }

    if (auto ret = nvm_exit_test_mode(); ret != ESP_OK) {
        return ret;
    }

    nvm_sectors_read_ = true;
    load_pdo_settings_from_nvm();
    return ESP_OK;
}

esp_err_t STUSB4500::write_nvm(bool use_defaults) {
    if (!is_initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (use_defaults) {
        nvm_sectors_ = kDefaultNvmSectors;
    } else if (!nvm_sectors_read_) {
        if (auto ret = read_nvm(); ret != ESP_OK) {
            return ret;
        }
    }

    if (!use_defaults) {
        save_pdo_settings_to_nvm();
    }

    if (auto ret = nvm_enter_write_mode(nvm::kAllSectors); ret != ESP_OK) {
        return ret;
    }

    for (uint8_t i = 0; i < nvm::kSectorCount; ++i) {
        if (auto ret = nvm_write_sector(i, nvm_sectors_[i].data()); ret != ESP_OK) {
            (void)nvm_exit_test_mode();  // Best-effort cleanup on error
            return ret;
        }
    }

    return nvm_exit_test_mode();
}

std::optional<uint8_t> STUSB4500::get_pdo_number() {
    if (!is_initialized_) {
        return std::nullopt;
    }

    auto pdo_count = i2c_read_register(reg::kDpmPdoNumb);
    if (pdo_count) {
        return static_cast<uint8_t>(*pdo_count & 0x07);
    }
    return std::nullopt;
}

esp_err_t STUSB4500::set_pdo_number(uint8_t pdo_count) {
    if (!is_initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_write_register(reg::kDpmPdoNumb, std::min(pdo_count, static_cast<uint8_t>(3)));
}

float STUSB4500::get_voltage(uint8_t pdo_num) {
    if (!is_initialized_ || pdo_num < 1 || pdo_num > kMaxPdos) {
        return 0.0f;
    }

    uint32_t pdo_data = read_pdo_from_registers(pdo_num);
    uint32_t voltage_raw = (pdo_data >> 10) & 0x3FF;
    return voltage_raw / 20.0f;  // 50mV resolution
}

esp_err_t STUSB4500::set_voltage(uint8_t pdo_num, float voltage) {
    if (!is_initialized_ || pdo_num < 1 || pdo_num > kMaxPdos) {
        return ESP_ERR_INVALID_ARG;
    }

    voltage = std::clamp(voltage, 5.0f, 20.0f);
    if (pdo_num == 1) {
        voltage = 5.0f;  // PDO1 is fixed at 5V
    }

    uint32_t voltage_raw = static_cast<uint32_t>(voltage * 20);
    uint32_t pdo_data = read_pdo_from_registers(pdo_num);

    pdo_data &= ~(0x3FFU << 10);
    pdo_data |= (voltage_raw << 10);

    return write_pdo_to_registers(pdo_num, pdo_data);
}

float STUSB4500::get_current(uint8_t pdo_num) {
    if (!is_initialized_ || pdo_num < 1 || pdo_num > kMaxPdos) {
        return 0.0f;
    }

    uint32_t pdo_data = read_pdo_from_registers(pdo_num);
    uint32_t current_raw = pdo_data & 0x3FF;
    return current_raw * 0.01f;  // 10mA resolution
}

esp_err_t STUSB4500::set_current(uint8_t pdo_num, float current) {
    if (!is_initialized_ || pdo_num < 1 || pdo_num > kMaxPdos) {
        return ESP_ERR_INVALID_ARG;
    }

    current = std::clamp(current, 0.0f, 5.0f);
    uint32_t current_raw = static_cast<uint32_t>(current / 0.01f) & 0x3FF;

    uint32_t pdo_data = read_pdo_from_registers(pdo_num);
    pdo_data = (pdo_data & ~0x3FFU) | current_raw;

    return write_pdo_to_registers(pdo_num, pdo_data);
}

std::optional<NegotiationStatus> STUSB4500::read_negotiation_status() {
    if (!is_initialized_) {
        return std::nullopt;
    }

    NegotiationStatus status{};

    auto device_id = read_device_id();
    if (!device_id) return std::nullopt;
    status.device_id = *device_id;

    auto cc_status = read_cc_status();
    if (!cc_status) return std::nullopt;
    status.cc_status = *cc_status;

    auto pd_typec_status = read_pd_typec_status();
    if (!pd_typec_status) return std::nullopt;
    status.pd_typec_status = *pd_typec_status;

    auto prt_status = read_prt_status();
    if (!prt_status) return std::nullopt;
    status.prt_status = *prt_status;

    auto pe_fsm = i2c_read_register(reg::kPeFsm);
    if (!pe_fsm) return std::nullopt;
    status.pe_fsm_state = *pe_fsm;

    status.is_connected = status.cc_status.is_connected();
    status.pd_negotiation_complete = status.prt_status.pd_contract_active;

    return status;
}

std::optional<CcStatus> STUSB4500::read_cc_status() {
    if (!is_initialized_) {
        return std::nullopt;
    }

    auto reg_value = i2c_read_register(reg::kCcStatus);
    if (!reg_value) {
        return std::nullopt;
    }

    CcStatus status{};
    status.cc1_state = static_cast<CcState>(*reg_value & kCcStatusCc1Mask);
    status.cc2_state = static_cast<CcState>((*reg_value & kCcStatusCc2Mask) >> kCcStatusCc2Shift);
    status.cc1_connected = status.cc1_state != CcState::NotInUfp;
    status.cc2_connected = status.cc2_state != CcState::NotInUfp;
    status.connection_result = (*reg_value & kCcStatusConnectResult) != 0;
    status.looking_for_connection = (*reg_value & kCcStatusLooking4Connection) != 0;

    return status;
}

std::optional<PdTypecStatus> STUSB4500::read_pd_typec_status() {
    if (!is_initialized_) {
        return std::nullopt;
    }

    auto reg_value = i2c_read_register(reg::kPdTypecStatus);
    if (!reg_value) {
        return std::nullopt;
    }

    PdTypecStatus status{};
    status.pd_typec_handshake_check = (*reg_value & kPdTypecHandshakeCheck) != 0;
    status.fsm_state = static_cast<TypecFsmState>(*reg_value & kPdTypecFsmStateMask);

    return status;
}

std::optional<PrtStatus> STUSB4500::read_prt_status() {
    if (!is_initialized_) {
        return std::nullopt;
    }

    auto reg_value = i2c_read_register(reg::kPrtStatus);
    if (!reg_value) {
        return std::nullopt;
    }

    PrtStatus status{};
    status.hw_reset_received = (*reg_value & kPrtHwResetReceived) != 0;
    status.soft_reset_received = (*reg_value & kPrtSoftResetReceived) != 0;
    status.data_role_sink = (*reg_value & kPrtDataRole) != 0;
    status.power_role_sink = (*reg_value & kPrtPowerRole) != 0;
    status.pd_contract_active = (*reg_value & kPrtPdContract) != 0;
    status.startup_power = (*reg_value & kPrtStartupPower) != 0;
    status.message_received = (*reg_value & kPrtMsgReceived) != 0;
    status.message_sent = (*reg_value & kPrtMsgSent) != 0;

    return status;
}

esp_err_t STUSB4500::clear_alert_status() {
    if (!is_initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    auto _ = i2c_read_register(reg::kAlertStatus1);
    return ESP_OK;
}

std::optional<PowerStatus> STUSB4500::read_power_status() {
    if (!is_initialized_) {
        return std::nullopt;
    }

    PowerStatus status{};

    if (auto voltage = read_voltage()) {
        status.voltage_mv = *voltage;
    }

    if (auto current = read_current()) {
        status.current_ma = *current;
    }

    if (status.voltage_mv > 0 && status.current_ma > 0) {
        status.power_mw = static_cast<uint32_t>(status.voltage_mv) * status.current_ma / 1000;
    }

    if (auto vsafe0v = i2c_read_register(reg::kVbusVsafe0v)) {
        status.vsafe0v = (*vsafe0v & kVsafe0vThreshold) != 0;
    }

    return status;
}

std::optional<uint16_t> STUSB4500::read_voltage() {
    if (!is_initialized_) {
        return std::nullopt;
    }

    uint8_t voltage_regs[2];
    if (i2c_read(reg::kVbusVoltageLow, voltage_regs, 2) != ESP_OK) {
        return std::nullopt;
    }

    uint16_t voltage_raw = voltage_regs[0] | (voltage_regs[1] << 8);
    return voltage_raw * device::kVbusLsbMv;
}

std::optional<uint16_t> STUSB4500::read_current() {
    if (!is_initialized_) {
        return std::nullopt;
    }

    // Check for high voltage (indicates PD contract)
    auto voltage = read_voltage();
    bool high_voltage_detected = voltage && *voltage > 5500;

    // Check PD contract status
    auto prt_status = read_prt_status();
    bool pd_contract_active = (prt_status && prt_status->pd_contract_active) || high_voltage_detected;

    if (!pd_contract_active) {
        return 500;  // Standard USB 2.0 current
    }

    // Get active PDO number
    auto active_pdo = i2c_read_register(reg::kPePdo);
    if (!active_pdo) {
        // Fallback: try highest configured PDO
        auto pdo_count = get_pdo_number();
        if (pdo_count && *pdo_count > 0) {
            uint32_t pdo_data = read_pdo_from_registers(*pdo_count);
            if (pdo_data != 0) {
                return static_cast<uint16_t>((pdo_data & 0x3FF) * 10);
            }
        }
        return 3000;  // Default estimate
    }

    uint8_t active_pdo_num = *active_pdo & 0x07;
    if (active_pdo_num == 0 || active_pdo_num > kMaxPdos) {
        return 3000;
    }

    uint32_t pdo_data = read_pdo_from_registers(active_pdo_num);
    if (pdo_data == 0) {
        return 3000;
    }

    return static_cast<uint16_t>((pdo_data & 0x3FF) * 10);
}

std::optional<uint32_t> STUSB4500::read_power() {
    auto voltage = read_voltage();
    auto current = read_current();

    if (!voltage || !current) {
        return std::nullopt;
    }

    return static_cast<uint32_t>(*voltage) * *current / 1000;
}

std::optional<Pdo> STUSB4500::read_active_pdo(uint8_t pdo_num) {
    if (!is_initialized_ || pdo_num < 1 || pdo_num > kMaxPdos) {
        return std::nullopt;
    }

    uint32_t raw_pdo = read_pdo_from_registers(pdo_num);
    return raw_to_pdo(raw_pdo);
}

std::optional<std::array<Pdo, STUSB4500::kMaxPdos>> STUSB4500::read_all_active_pdos() {
    std::array<Pdo, kMaxPdos> pdos{};

    for (size_t i = 0; i < kMaxPdos; ++i) {
        auto pdo = read_active_pdo(i + 1);
        if (!pdo) {
            return std::nullopt;
        }
        pdos[i] = *pdo;
    }

    return pdos;
}

std::optional<uint8_t> STUSB4500::get_active_pdo_count() {
    return get_pdo_number();
}

std::optional<uint8_t> STUSB4500::get_negotiated_pdo_number() {
    if (!is_initialized_) {
        return std::nullopt;
    }

    auto prt_status = read_prt_status();
    if (!prt_status || !prt_status->pd_contract_active) {
        return 0;
    }

    auto active_pdo = i2c_read_register(reg::kPePdo);
    if (!active_pdo) {
        return std::nullopt;
    }

    return static_cast<uint8_t>(*active_pdo & 0x07);
}

esp_err_t STUSB4500::soft_reset() {
    if (!is_initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (auto ret = i2c_write_register(reg::kTxHeaderLow, 0x0D); ret != ESP_OK) {
        return ret;
    }

    if (auto ret = i2c_write_register(reg::kPdCommandCtrl, 0x26); ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

// I2C Implementation
esp_err_t STUSB4500::i2c_read(uint8_t reg_addr, uint8_t* data, size_t len) {
    if (!device_handle_ || !data || len == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    auto ret = i2c_master_transmit_receive(device_handle_, &reg_addr, 1, data, len, kI2cTimeoutMs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: reg=0x%02X, error=%s", reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t STUSB4500::i2c_write(uint8_t reg_addr, const uint8_t* data, size_t len) {
    if (!device_handle_ || !data || len == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    std::vector<uint8_t> buffer(len + 1);
    buffer[0] = reg_addr;
    std::memcpy(&buffer[1], data, len);

    auto ret = i2c_master_transmit(device_handle_, buffer.data(), buffer.size(), kI2cTimeoutMs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: reg=0x%02X, error=%s", reg_addr, esp_err_to_name(ret));
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    return ret;
}

std::optional<uint8_t> STUSB4500::i2c_read_register(uint8_t reg_addr) {
    uint8_t data;
    if (i2c_read(reg_addr, &data, 1) != ESP_OK) {
        return std::nullopt;
    }
    return data;
}

esp_err_t STUSB4500::i2c_write_register(uint8_t reg_addr, uint8_t data) {
    return i2c_write(reg_addr, &data, 1);
}

// NVM Implementation
esp_err_t STUSB4500::nvm_enter_read_mode() {
    if (auto ret = i2c_write_register(reg::kFtpCustPasswordReg, nvm::kPassword); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl0, 0x00); ret != ESP_OK) {
        return ret;
    }
    return i2c_write_register(reg::kFtpCtrl0, nvm::kCustPwr | nvm::kCustRstN);
}

esp_err_t STUSB4500::nvm_enter_write_mode(uint8_t sectors_to_erase) {
    if (auto ret = i2c_write_register(reg::kFtpCustPasswordReg, nvm::kPassword); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kRwBuffer, 0x00); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl0, 0x00); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl0, nvm::kCustPwr | nvm::kCustRstN); ret != ESP_OK) {
        return ret;
    }

    // Write sectors opcode
    if (auto ret = i2c_write_register(reg::kFtpCtrl1,
            ((sectors_to_erase << 3) & nvm::kCustSer) | (nvm::kOpcodeWriteSer & nvm::kCustOpcode)); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl0,
            nvm::kCustPwr | nvm::kCustRstN | nvm::kCustReq); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = nvm_wait_for_completion(kNvmWriteTimeoutMs); ret != ESP_OK) {
        return ret;
    }

    // Soft prog sector
    if (auto ret = i2c_write_register(reg::kFtpCtrl1, nvm::kOpcodeSoftProgSector & nvm::kCustOpcode); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl0,
            nvm::kCustPwr | nvm::kCustRstN | nvm::kCustReq); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = nvm_wait_for_completion(); ret != ESP_OK) {
        return ret;
    }

    // Erase sectors
    if (auto ret = i2c_write_register(reg::kFtpCtrl1, nvm::kOpcodeEraseSector & nvm::kCustOpcode); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl0,
            nvm::kCustPwr | nvm::kCustRstN | nvm::kCustReq); ret != ESP_OK) {
        return ret;
    }

    return nvm_wait_for_completion();
}

esp_err_t STUSB4500::nvm_exit_test_mode() {
    if (auto ret = i2c_write_register(reg::kFtpCtrl0, nvm::kCustRstN); ret != ESP_OK) {
        return ret;
    }
    return i2c_write_register(reg::kFtpCustPasswordReg, 0x00);
}

esp_err_t STUSB4500::nvm_read_sector(uint8_t sector_num, uint8_t* sector_data) {
    if (auto ret = i2c_write_register(reg::kFtpCtrl0, nvm::kCustPwr | nvm::kCustRstN); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl1, nvm::kOpcodeRead & nvm::kCustOpcode); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl0,
            (sector_num & nvm::kCustSect) | nvm::kCustPwr | nvm::kCustRstN | nvm::kCustReq); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = nvm_wait_for_completion(); ret != ESP_OK) {
        return ret;
    }

    return i2c_read(reg::kRwBuffer, sector_data, nvm::kSectorSize);
}

esp_err_t STUSB4500::nvm_write_sector(uint8_t sector_num, const uint8_t* sector_data) {
    if (auto ret = i2c_write(reg::kRwBuffer, sector_data, nvm::kSectorSize); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl1, nvm::kOpcodeWritePl & nvm::kCustOpcode); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl0,
            nvm::kCustPwr | nvm::kCustRstN | nvm::kCustReq); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = nvm_wait_for_completion(); ret != ESP_OK) {
        return ret;
    }

    if (auto ret = i2c_write_register(reg::kFtpCtrl1, nvm::kOpcodeProgSector & nvm::kCustOpcode); ret != ESP_OK) {
        return ret;
    }
    if (auto ret = i2c_write_register(reg::kFtpCtrl0,
            (sector_num & nvm::kCustSect) | nvm::kCustPwr | nvm::kCustRstN | nvm::kCustReq); ret != ESP_OK) {
        return ret;
    }

    return nvm_wait_for_completion();
}

esp_err_t STUSB4500::nvm_wait_for_completion(uint32_t timeout_ms) {
    const uint32_t start_time = xTaskGetTickCount();
    const uint32_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
        auto status = i2c_read_register(reg::kFtpCtrl0);
        if (!status) {
            return ESP_ERR_INVALID_RESPONSE;
        }

        if ((*status & nvm::kCustReq) == 0) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGE(TAG, "NVM operation timeout");
    return ESP_ERR_TIMEOUT;
}

uint32_t STUSB4500::read_pdo_from_registers(uint8_t pdo_num) {
    if (pdo_num < 1 || pdo_num > kMaxPdos) {
        return 0;
    }

    uint8_t pdo_regs[4];
    uint8_t base_reg = reg::kDpmSnkPdo1 + ((pdo_num - 1) * 4);

    if (i2c_read(base_reg, pdo_regs, 4) != ESP_OK) {
        return 0;
    }

    return static_cast<uint32_t>(pdo_regs[0]) |
           (static_cast<uint32_t>(pdo_regs[1]) << 8) |
           (static_cast<uint32_t>(pdo_regs[2]) << 16) |
           (static_cast<uint32_t>(pdo_regs[3]) << 24);
}

esp_err_t STUSB4500::write_pdo_to_registers(uint8_t pdo_num, uint32_t pdo_data) {
    if (pdo_num < 1 || pdo_num > kMaxPdos) {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<uint8_t, 4> pdo_regs = {
        static_cast<uint8_t>(pdo_data & 0xFF),
        static_cast<uint8_t>((pdo_data >> 8) & 0xFF),
        static_cast<uint8_t>((pdo_data >> 16) & 0xFF),
        static_cast<uint8_t>((pdo_data >> 24) & 0xFF)
    };

    uint8_t base_reg = reg::kDpmSnkPdo1 + ((pdo_num - 1) * 4);
    return i2c_write(base_reg, pdo_regs.data(), 4);
}

uint32_t STUSB4500::pdo_to_raw(const Pdo& pdo) noexcept {
    if (pdo.type != PdoType::Fixed) {
        return 0;
    }

    uint32_t raw = static_cast<uint32_t>(PdoType::Fixed) << 30;
    if (pdo.dual_role_power) raw |= (1U << 29);
    if (pdo.usb_suspend_supported) raw |= (1U << 28);
    if (pdo.unconstrained_power) raw |= (1U << 27);
    if (pdo.usb_comm_capable) raw |= (1U << 26);
    if (pdo.dual_role_data) raw |= (1U << 25);
    raw |= (static_cast<uint32_t>(pdo.peak_current) & 0x03) << 20;
    raw |= (((pdo.voltage_mv + 25) / 50) & 0x3FF) << 10;
    raw |= ((pdo.max_current_ma + 5) / 10) & 0x3FF;

    return raw;
}

Pdo STUSB4500::raw_to_pdo(uint32_t raw_pdo) noexcept {
    Pdo pdo{};
    pdo.type = static_cast<PdoType>((raw_pdo >> 30) & 0x03);

    if (pdo.type == PdoType::Fixed) {
        pdo.dual_role_power = (raw_pdo & (1U << 29)) != 0;
        pdo.usb_suspend_supported = (raw_pdo & (1U << 28)) != 0;
        pdo.unconstrained_power = (raw_pdo & (1U << 27)) != 0;
        pdo.usb_comm_capable = (raw_pdo & (1U << 26)) != 0;
        pdo.dual_role_data = (raw_pdo & (1U << 25)) != 0;
        pdo.peak_current = (raw_pdo >> 20) & 0x03;
        pdo.voltage_mv = ((raw_pdo >> 10) & 0x3FF) * 50;
        pdo.max_current_ma = (raw_pdo & 0x3FF) * 10;
    }

    return pdo;
}

void STUSB4500::load_pdo_settings_from_nvm() {
    if (!nvm_sectors_read_) {
        return;
    }

    // PDO number from sector 3, byte 2, bits 2:3
    uint8_t pdo_count = (nvm_sectors_[3][2] & 0x06) >> 1;
    (void)set_pdo_number(pdo_count);

    // PDO1 (fixed at 5V)
    (void)set_voltage(1, 5.0f);
    uint8_t current_val = (nvm_sectors_[3][2] & 0xF0) >> 4;
    float current = (current_val == 0) ? 0.0f :
                    (current_val < 11) ? (current_val * 0.25f + 0.25f) :
                    (current_val * 0.50f - 2.50f);
    (void)set_current(1, current);

    // PDO2
    float voltage2 = ((nvm_sectors_[4][1] << 2) + (nvm_sectors_[4][0] >> 6)) / 20.0f;
    (void)set_voltage(2, voltage2);
    current_val = nvm_sectors_[3][4] & 0x0F;
    current = (current_val == 0) ? 0.0f :
              (current_val < 11) ? (current_val * 0.25f + 0.25f) :
              (current_val * 0.50f - 2.50f);
    (void)set_current(2, current);

    // PDO3
    float voltage3 = (((nvm_sectors_[4][3] & 0x03) << 8) + nvm_sectors_[4][2]) / 20.0f;
    (void)set_voltage(3, voltage3);
    current_val = (nvm_sectors_[3][5] & 0xF0) >> 4;
    current = (current_val == 0) ? 0.0f :
              (current_val < 11) ? (current_val * 0.25f + 0.25f) :
              (current_val * 0.50f - 2.50f);
    (void)set_current(3, current);
}

void STUSB4500::save_pdo_settings_to_nvm() {
    if (!nvm_sectors_read_) {
        return;
    }

    // Save PDO count
    auto pdo_count = get_pdo_number();
    if (pdo_count) {
        nvm_sectors_[3][2] = (nvm_sectors_[3][2] & 0xF9) | (*pdo_count << 1);
    }

    // Convert and save PDO settings
    for (uint8_t i = 1; i <= kMaxPdos; ++i) {
        float current = get_current(i);
        uint8_t nvm_current = 0;
        if (current > 0.0f) {
            nvm_current = (current <= 3.0f) ?
                          static_cast<uint8_t>(current * 4) - 1 :
                          static_cast<uint8_t>(current * 2) + 5;
        }

        if (i == 1) {
            nvm_sectors_[3][2] = (nvm_sectors_[3][2] & 0x0F) | (nvm_current << 4);
        } else if (i == 2) {
            nvm_sectors_[3][4] = (nvm_sectors_[3][4] & 0xF0) | nvm_current;
            uint16_t voltage = static_cast<uint16_t>(get_voltage(2) * 20);
            nvm_sectors_[4][0] = (nvm_sectors_[4][0] & 0x3F) | ((voltage & 0x03) << 6);
            nvm_sectors_[4][1] = voltage >> 2;
        } else {
            nvm_sectors_[3][5] = (nvm_sectors_[3][5] & 0x0F) | (nvm_current << 4);
            uint16_t voltage = static_cast<uint16_t>(get_voltage(3) * 20);
            nvm_sectors_[4][2] = voltage & 0xFF;
            nvm_sectors_[4][3] = (nvm_sectors_[4][3] & 0xFC) | (voltage >> 8);
        }
    }
}

// Configuration functions
uint8_t STUSB4500::get_lower_voltage_limit(uint8_t pdo_num) {
    if (!nvm_sectors_read_ || pdo_num < 1 || pdo_num > kMaxPdos) {
        return 0;
    }

    if (pdo_num == 1) return 0;
    if (pdo_num == 2) return (nvm_sectors_[3][4] >> 4) + 5;
    return (nvm_sectors_[3][6] & 0x0F) + 5;
}

esp_err_t STUSB4500::set_lower_voltage_limit(uint8_t pdo_num, uint8_t percentage) {
    if (!nvm_sectors_read_ || pdo_num < 2 || pdo_num > kMaxPdos) {
        return ESP_ERR_INVALID_ARG;
    }

    percentage = std::clamp(percentage, static_cast<uint8_t>(5), static_cast<uint8_t>(20));

    if (pdo_num == 2) {
        nvm_sectors_[3][4] = (nvm_sectors_[3][4] & 0x0F) | ((percentage - 5) << 4);
    } else {
        nvm_sectors_[3][6] = (nvm_sectors_[3][6] & 0xF0) | (percentage - 5);
    }
    return ESP_OK;
}

uint8_t STUSB4500::get_upper_voltage_limit(uint8_t pdo_num) {
    if (!nvm_sectors_read_ || pdo_num < 1 || pdo_num > kMaxPdos) {
        return 0;
    }

    if (pdo_num == 1) return (nvm_sectors_[3][3] >> 4) + 5;
    if (pdo_num == 2) return (nvm_sectors_[3][5] & 0x0F) + 5;
    return (nvm_sectors_[3][6] >> 4) + 5;
}

esp_err_t STUSB4500::set_upper_voltage_limit(uint8_t pdo_num, uint8_t percentage) {
    if (!nvm_sectors_read_ || pdo_num < 1 || pdo_num > kMaxPdos) {
        return ESP_ERR_INVALID_ARG;
    }

    percentage = std::clamp(percentage, static_cast<uint8_t>(5), static_cast<uint8_t>(20));

    if (pdo_num == 1) {
        nvm_sectors_[3][3] = (nvm_sectors_[3][3] & 0x0F) | ((percentage - 5) << 4);
    } else if (pdo_num == 2) {
        nvm_sectors_[3][5] = (nvm_sectors_[3][5] & 0xF0) | (percentage - 5);
    } else {
        nvm_sectors_[3][6] = (nvm_sectors_[3][6] & 0x0F) | ((percentage - 5) << 4);
    }
    return ESP_OK;
}

float STUSB4500::get_flex_current() {
    if (!nvm_sectors_read_) return 0.0f;
    uint16_t value = ((nvm_sectors_[4][4] & 0x0F) << 6) + ((nvm_sectors_[4][3] & 0xFC) >> 2);
    return value / 100.0f;
}

esp_err_t STUSB4500::set_flex_current(float current) {
    if (!nvm_sectors_read_) return ESP_ERR_INVALID_STATE;

    current = std::clamp(current, 0.0f, 5.0f);
    uint16_t value = static_cast<uint16_t>(current * 100);

    nvm_sectors_[4][4] = (nvm_sectors_[4][4] & 0xF0) | ((value >> 6) & 0x0F);
    nvm_sectors_[4][3] = (nvm_sectors_[4][3] & 0x03) | ((value << 2) & 0xFC);
    return ESP_OK;
}

bool STUSB4500::get_external_power() {
    return nvm_sectors_read_ && (nvm_sectors_[3][2] & 0x08) != 0;
}

esp_err_t STUSB4500::set_external_power(bool enabled) {
    if (!nvm_sectors_read_) return ESP_ERR_INVALID_STATE;
    nvm_sectors_[3][2] = (nvm_sectors_[3][2] & 0xF7) | (enabled ? 0x08 : 0x00);
    return ESP_OK;
}

bool STUSB4500::get_usb_comm_capable() {
    return nvm_sectors_read_ && (nvm_sectors_[3][2] & 0x01) != 0;
}

esp_err_t STUSB4500::set_usb_comm_capable(bool enabled) {
    if (!nvm_sectors_read_) return ESP_ERR_INVALID_STATE;
    nvm_sectors_[3][2] = (nvm_sectors_[3][2] & 0xFE) | (enabled ? 0x01 : 0x00);
    return ESP_OK;
}

uint8_t STUSB4500::get_config_ok_gpio() {
    return nvm_sectors_read_ ? (nvm_sectors_[4][4] & 0x60) >> 5 : 0;
}

esp_err_t STUSB4500::set_config_ok_gpio(uint8_t config) {
    if (!nvm_sectors_read_) return ESP_ERR_INVALID_STATE;
    config = std::min(config, static_cast<uint8_t>(3));
    nvm_sectors_[4][4] = (nvm_sectors_[4][4] & 0x9F) | (config << 5);
    return ESP_OK;
}

uint8_t STUSB4500::get_gpio_ctrl() {
    return nvm_sectors_read_ ? (nvm_sectors_[1][0] & 0x30) >> 4 : 0;
}

esp_err_t STUSB4500::set_gpio_ctrl(uint8_t config) {
    if (!nvm_sectors_read_) return ESP_ERR_INVALID_STATE;
    config = std::min(config, static_cast<uint8_t>(3));
    nvm_sectors_[1][0] = (nvm_sectors_[1][0] & 0xCF) | (config << 4);
    return ESP_OK;
}

bool STUSB4500::get_power_above_5v_only() {
    return nvm_sectors_read_ && (nvm_sectors_[4][6] & 0x08) != 0;
}

esp_err_t STUSB4500::set_power_above_5v_only(bool enabled) {
    if (!nvm_sectors_read_) return ESP_ERR_INVALID_STATE;
    nvm_sectors_[4][6] = (nvm_sectors_[4][6] & 0xF7) | (enabled ? 0x08 : 0x00);
    return ESP_OK;
}

bool STUSB4500::get_req_src_current() {
    return nvm_sectors_read_ && (nvm_sectors_[4][6] & 0x10) != 0;
}

esp_err_t STUSB4500::set_req_src_current(bool enabled) {
    if (!nvm_sectors_read_) return ESP_ERR_INVALID_STATE;
    nvm_sectors_[4][6] = (nvm_sectors_[4][6] & 0xEF) | (enabled ? 0x10 : 0x00);
    return ESP_OK;
}

const char* STUSB4500::cc_state_to_string(CcState state) noexcept {
    switch (state) {
        case CcState::NotInUfp: return "Not in UFP";
        case CcState::DefaultUsb: return "Default USB";
        case CcState::Power1_5A: return "1.5A";
        case CcState::Power3_0A: return "3.0A";
        default: return "Unknown";
    }
}

const char* STUSB4500::typec_fsm_state_to_string(TypecFsmState state) noexcept {
    switch (state) {
        case TypecFsmState::UnattachedSnk: return "Unattached.SNK";
        case TypecFsmState::AttachWaitSnk: return "AttachWait.SNK";
        case TypecFsmState::AttachedSnk: return "Attached.SNK";
        case TypecFsmState::DebugAccessorySnk: return "DebugAccessory.SNK";
        default: return "Unknown";
    }
}

} // namespace stusb
