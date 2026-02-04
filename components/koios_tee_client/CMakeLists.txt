# Koios TEE Client Component
#
# Provides API for application to call TEE services.
# TEE code runs in RTC FAST memory (0x600FE000-0x60100000).

file(GLOB_RECURSE NESTED_SRC
    "src/*.c"
    "src/*.cpp"
)

idf_component_register(
    SRCS ${NESTED_SRC}
    INCLUDE_DIRS "include"
    REQUIRES soc
)

if(CONFIG_KOIOS_TEE_CLIENT_ENABLED)
    message(STATUS "Koios TEE Client: API at 0x600FF000, Entry at 0x600FE100")
endif()
