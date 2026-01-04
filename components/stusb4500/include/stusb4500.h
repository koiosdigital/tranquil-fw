#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#include <cstdint>
#include <optional>
#include <array>
#include <span>

namespace stusb {

// Device identification constants
namespace reg {
    constexpr uint8_t kDeviceId = 0x2F;
    constexpr uint8_t kAlertStatus1 = 0x0B;
    constexpr uint8_t kCcStatus = 0x11;
    constexpr uint8_t kPdTypecStatus = 0x14;
    constexpr uint8_t kPrtStatus = 0x16;
    constexpr uint8_t kResetCtrl = 0x19;
    constexpr uint8_t kPeFsm = 0x29;
    constexpr uint8_t kVbusVoltageLow = 0x26;
    constexpr uint8_t kVbusVsafe0v = 0x27;
    constexpr uint8_t kTxHeaderLow = 0x51;
    constexpr uint8_t kRwBuffer = 0x53;
    constexpr uint8_t kDpmPdoNumb = 0x70;
    constexpr uint8_t kDpmSnkPdo1 = 0x85;
    constexpr uint8_t kDpmSnkPdo2 = 0x89;
    constexpr uint8_t kDpmSnkPdo3 = 0x8D;
    constexpr uint8_t kRdoRegStatus = 0x91;
    constexpr uint8_t kPePdo = 0x93;
    constexpr uint8_t kFtpCustPasswordReg = 0x95;
    constexpr uint8_t kFtpCtrl0 = 0x96;
    constexpr uint8_t kFtpCtrl1 = 0x97;
    constexpr uint8_t kPdCommandCtrl = 0x1A;
} // namespace reg

namespace device {
    constexpr uint8_t kEvalDeviceId = 0x25;
    constexpr uint8_t kProdDeviceId = 0x21;
    constexpr uint8_t kDefaultAddress = 0x28;
    constexpr uint16_t kVbusLsbMv = 25;
} // namespace device

namespace nvm {
    constexpr uint8_t kPassword = 0x47;
    constexpr uint8_t kCustPwr = 0x80;
    constexpr uint8_t kCustRstN = 0x40;
    constexpr uint8_t kCustReq = 0x10;
    constexpr uint8_t kCustSect = 0x07;
    constexpr uint8_t kCustSer = 0xF8;
    constexpr uint8_t kCustOpcode = 0x07;
    constexpr uint8_t kOpcodeRead = 0x00;
    constexpr uint8_t kOpcodeWritePl = 0x01;
    constexpr uint8_t kOpcodeWriteSer = 0x02;
    constexpr uint8_t kOpcodeEraseSector = 0x05;
    constexpr uint8_t kOpcodeProgSector = 0x06;
    constexpr uint8_t kOpcodeSoftProgSector = 0x07;
    constexpr uint8_t kSector0 = 0x01;
    constexpr uint8_t kSector1 = 0x02;
    constexpr uint8_t kSector2 = 0x04;
    constexpr uint8_t kSector3 = 0x08;
    constexpr uint8_t kSector4 = 0x10;
    constexpr uint8_t kAllSectors = kSector0 | kSector1 | kSector2 | kSector3 | kSector4;
    constexpr size_t kSectorCount = 5;
    constexpr size_t kSectorSize = 8;
} // namespace nvm

// CC pin states
enum class CcState : uint8_t {
    NotInUfp = 0,
    DefaultUsb = 1,
    Power1_5A = 2,
    Power3_0A = 3
};

// TypeC FSM states
enum class TypecFsmState : uint8_t {
    UnattachedSnk = 0,
    AttachWaitSnk = 1,
    AttachedSnk = 2,
    DebugAccessorySnk = 3
};

// PDO types
enum class PdoType : uint8_t {
    Fixed = 0,
    Battery = 1,
    Variable = 2,
    Apdo = 3
};

// Status structures
struct CcStatus {
    CcState cc1_state = CcState::NotInUfp;
    CcState cc2_state = CcState::NotInUfp;
    bool cc1_connected = false;
    bool cc2_connected = false;
    bool connection_result = false;
    bool looking_for_connection = false;

    [[nodiscard]] bool is_connected() const noexcept {
        return connection_result && (cc1_connected || cc2_connected);
    }
};

struct PdTypecStatus {
    bool pd_typec_handshake_check = false;
    TypecFsmState fsm_state = TypecFsmState::UnattachedSnk;
};

struct PrtStatus {
    bool hw_reset_received = false;
    bool soft_reset_received = false;
    bool data_role_sink = false;
    bool power_role_sink = false;
    bool pd_contract_active = false;
    bool startup_power = false;
    bool message_received = false;
    bool message_sent = false;
};

struct NegotiationStatus {
    uint8_t device_id = 0;
    CcStatus cc_status;
    PdTypecStatus pd_typec_status;
    PrtStatus prt_status;
    uint8_t pe_fsm_state = 0;
    bool is_connected = false;
    bool pd_negotiation_complete = false;
};

struct PowerStatus {
    uint16_t voltage_mv = 0;
    uint16_t current_ma = 0;
    uint32_t power_mw = 0;
    bool vsafe0v = false;
};

struct Pdo {
    PdoType type = PdoType::Fixed;
    uint16_t voltage_mv = 0;
    uint16_t max_current_ma = 0;
    uint8_t peak_current = 0;
    bool dual_role_power = false;
    bool usb_suspend_supported = false;
    bool unconstrained_power = false;
    bool usb_comm_capable = false;
    bool dual_role_data = false;
};

class STUSB4500 {
public:
    static constexpr size_t kMaxPdos = 3;
    using NvmSectors = std::array<std::array<uint8_t, nvm::kSectorSize>, nvm::kSectorCount>;

    STUSB4500() = default;
    ~STUSB4500();

    // Non-copyable
    STUSB4500(const STUSB4500&) = delete;
    STUSB4500& operator=(const STUSB4500&) = delete;

    // Moveable
    STUSB4500(STUSB4500&& other) noexcept;
    STUSB4500& operator=(STUSB4500&& other) noexcept;

    // Initialization
    [[nodiscard]] esp_err_t begin(uint8_t device_address = device::kDefaultAddress);
    esp_err_t end() noexcept;

    // Device identification
    [[nodiscard]] std::optional<uint8_t> read_device_id();

    // NVM operations
    [[nodiscard]] esp_err_t read_nvm();
    [[nodiscard]] esp_err_t write_nvm(bool use_defaults = false);

    // PDO configuration
    [[nodiscard]] std::optional<uint8_t> get_pdo_number();
    [[nodiscard]] esp_err_t set_pdo_number(uint8_t pdo_count);
    [[nodiscard]] float get_voltage(uint8_t pdo_num);
    [[nodiscard]] esp_err_t set_voltage(uint8_t pdo_num, float voltage);
    [[nodiscard]] float get_current(uint8_t pdo_num);
    [[nodiscard]] esp_err_t set_current(uint8_t pdo_num, float current);

    // Status and monitoring
    [[nodiscard]] std::optional<NegotiationStatus> read_negotiation_status();
    [[nodiscard]] std::optional<CcStatus> read_cc_status();
    [[nodiscard]] std::optional<PdTypecStatus> read_pd_typec_status();
    [[nodiscard]] std::optional<PrtStatus> read_prt_status();
    [[nodiscard]] esp_err_t clear_alert_status();

    // Power measurement
    [[nodiscard]] std::optional<PowerStatus> read_power_status();
    [[nodiscard]] std::optional<uint16_t> read_voltage();
    [[nodiscard]] std::optional<uint16_t> read_current();
    [[nodiscard]] std::optional<uint32_t> read_power();

    // Active PDO functions
    [[nodiscard]] std::optional<Pdo> read_active_pdo(uint8_t pdo_num);
    [[nodiscard]] std::optional<std::array<Pdo, kMaxPdos>> read_all_active_pdos();
    [[nodiscard]] std::optional<uint8_t> get_active_pdo_count();
    [[nodiscard]] std::optional<uint8_t> get_negotiated_pdo_number();

    // Control functions
    [[nodiscard]] esp_err_t soft_reset();

    // Configuration functions
    [[nodiscard]] uint8_t get_lower_voltage_limit(uint8_t pdo_num);
    [[nodiscard]] esp_err_t set_lower_voltage_limit(uint8_t pdo_num, uint8_t percentage);
    [[nodiscard]] uint8_t get_upper_voltage_limit(uint8_t pdo_num);
    [[nodiscard]] esp_err_t set_upper_voltage_limit(uint8_t pdo_num, uint8_t percentage);
    [[nodiscard]] float get_flex_current();
    [[nodiscard]] esp_err_t set_flex_current(float current);
    [[nodiscard]] bool get_external_power();
    [[nodiscard]] esp_err_t set_external_power(bool enabled);
    [[nodiscard]] bool get_usb_comm_capable();
    [[nodiscard]] esp_err_t set_usb_comm_capable(bool enabled);
    [[nodiscard]] uint8_t get_config_ok_gpio();
    [[nodiscard]] esp_err_t set_config_ok_gpio(uint8_t config);
    [[nodiscard]] uint8_t get_gpio_ctrl();
    [[nodiscard]] esp_err_t set_gpio_ctrl(uint8_t config);
    [[nodiscard]] bool get_power_above_5v_only();
    [[nodiscard]] esp_err_t set_power_above_5v_only(bool enabled);
    [[nodiscard]] bool get_req_src_current();
    [[nodiscard]] esp_err_t set_req_src_current(bool enabled);

    // Accessors
    [[nodiscard]] bool is_initialized() const noexcept { return is_initialized_; }
    [[nodiscard]] bool nvm_loaded() const noexcept { return nvm_sectors_read_; }

    // Utility functions
    [[nodiscard]] static const char* cc_state_to_string(CcState state) noexcept;
    [[nodiscard]] static const char* typec_fsm_state_to_string(TypecFsmState state) noexcept;

    // Default NVM values
    static const NvmSectors kDefaultNvmSectors;

private:
    // I2C communication
    [[nodiscard]] esp_err_t i2c_read(uint8_t reg_addr, uint8_t* data, size_t len);
    [[nodiscard]] esp_err_t i2c_write(uint8_t reg_addr, const uint8_t* data, size_t len);
    [[nodiscard]] std::optional<uint8_t> i2c_read_register(uint8_t reg_addr);
    [[nodiscard]] esp_err_t i2c_write_register(uint8_t reg_addr, uint8_t data);

    // NVM access functions
    [[nodiscard]] esp_err_t nvm_enter_read_mode();
    [[nodiscard]] esp_err_t nvm_enter_write_mode(uint8_t sectors_to_erase);
    [[nodiscard]] esp_err_t nvm_exit_test_mode();
    [[nodiscard]] esp_err_t nvm_read_sector(uint8_t sector_num, uint8_t* sector_data);
    [[nodiscard]] esp_err_t nvm_write_sector(uint8_t sector_num, const uint8_t* sector_data);
    [[nodiscard]] esp_err_t nvm_wait_for_completion(uint32_t timeout_ms = 100);

    // PDO manipulation
    [[nodiscard]] uint32_t read_pdo_from_registers(uint8_t pdo_num);
    [[nodiscard]] esp_err_t write_pdo_to_registers(uint8_t pdo_num, uint32_t pdo_data);
    [[nodiscard]] static uint32_t pdo_to_raw(const Pdo& pdo) noexcept;
    [[nodiscard]] static Pdo raw_to_pdo(uint32_t raw_pdo) noexcept;

    // NVM settings management
    void load_pdo_settings_from_nvm();
    void save_pdo_settings_to_nvm();

    // Member variables
    i2c_master_bus_handle_t i2c_bus_handle_ = nullptr;
    i2c_master_dev_handle_t device_handle_ = nullptr;
    uint8_t device_address_ = device::kDefaultAddress;
    bool is_initialized_ = false;
    bool nvm_sectors_read_ = false;
    NvmSectors nvm_sectors_{};
};

} // namespace stusb

// Backwards compatibility - type aliases
using stusb4500_cc_state_t = stusb::CcState;
using stusb4500_typec_fsm_state_t = stusb::TypecFsmState;
using stusb4500_pdo_type_t = stusb::PdoType;
using stusb4500_cc_status_t = stusb::CcStatus;
using stusb4500_pd_typec_status_t = stusb::PdTypecStatus;
using stusb4500_prt_status_t = stusb::PrtStatus;
using stusb4500_negotiation_status_t = stusb::NegotiationStatus;
using stusb4500_power_status_t = stusb::PowerStatus;
using stusb4500_pdo_t = stusb::Pdo;
using STUSB4500 = stusb::STUSB4500;

// Legacy enum value mappings
constexpr auto STUSB4500_CC_STATE_NOT_IN_UFP = stusb::CcState::NotInUfp;
constexpr auto STUSB4500_CC_STATE_DEFAULT_USB = stusb::CcState::DefaultUsb;
constexpr auto STUSB4500_CC_STATE_POWER_1_5A = stusb::CcState::Power1_5A;
constexpr auto STUSB4500_CC_STATE_POWER_3_0A = stusb::CcState::Power3_0A;

constexpr auto STUSB4500_TYPEC_FSM_UNATTACHED_SNK = stusb::TypecFsmState::UnattachedSnk;
constexpr auto STUSB4500_TYPEC_FSM_ATTACH_WAIT_SNK = stusb::TypecFsmState::AttachWaitSnk;
constexpr auto STUSB4500_TYPEC_FSM_ATTACHED_SNK = stusb::TypecFsmState::AttachedSnk;
constexpr auto STUSB4500_TYPEC_FSM_DEBUG_ACCESSORY_SNK = stusb::TypecFsmState::DebugAccessorySnk;

constexpr auto STUSB4500_PDO_TYPE_FIXED = stusb::PdoType::Fixed;
constexpr auto STUSB4500_PDO_TYPE_BATTERY = stusb::PdoType::Battery;
constexpr auto STUSB4500_PDO_TYPE_VARIABLE = stusb::PdoType::Variable;
constexpr auto STUSB4500_PDO_TYPE_APDO = stusb::PdoType::Apdo;
