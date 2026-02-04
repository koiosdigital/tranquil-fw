# Koios TEE Component
# Provides shared TEE API header for app and bootloader
# The actual TEE runs in RTC FAST memory (loaded by bootloader)

idf_component_register(
    INCLUDE_DIRS "include"
)

if(CONFIG_KOIOS_TEE_ENABLED)
    message(STATUS "Koios TEE: API header at 0x600FF000")
endif()
