idf_component_register(
    SRCS
        "src/tee_init.c"
        "src/tee_wcl.c"
        "src/tee_pms.c"
        "src/tee_exit_stub.S"
        "src/tee_trampoline.S"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES bootloader_support bootloader esp_app_format soc
)

# Get the cross-compiler prefix
idf_build_get_property(target IDF_TARGET)
set(CROSS_COMPILE xtensa-${target}-elf-)

# Build TEE handlers as a separate binary linked at RTC FAST address
# tee_entry.S (assembly entry stub) + tee_handlers.c (C handlers)
set(TEE_ENTRY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/tee_entry.S)
set(TEE_HANDLERS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/tee_handlers.c)
set(TEE_HANDLERS_LD ${CMAKE_CURRENT_SOURCE_DIR}/linker/tee_handlers.ld)
set(TEE_ENTRY_OBJ ${CMAKE_CURRENT_BINARY_DIR}/tee_entry.o)
set(TEE_HANDLERS_OBJ ${CMAKE_CURRENT_BINARY_DIR}/tee_handlers.o)
set(TEE_HANDLERS_ELF ${CMAKE_CURRENT_BINARY_DIR}/tee_handlers.elf)
set(TEE_HANDLERS_BIN ${CMAKE_CURRENT_BINARY_DIR}/tee_handlers.bin)
set(TEE_HANDLERS_EMBED ${CMAKE_CURRENT_BINARY_DIR}/tee_handlers_bin.c)

# Compile assembly entry stub
add_custom_command(
    OUTPUT ${TEE_ENTRY_OBJ}
    COMMAND ${CROSS_COMPILE}gcc
        -c ${TEE_ENTRY_SRC}
        -o ${TEE_ENTRY_OBJ}
        -mabi=call0
    DEPENDS ${TEE_ENTRY_SRC}
    COMMENT "Compiling TEE entry stub"
)

# Compile C handlers object file
add_custom_command(
    OUTPUT ${TEE_HANDLERS_OBJ}
    COMMAND ${CROSS_COMPILE}gcc
        -c ${TEE_HANDLERS_SRC}
        -o ${TEE_HANDLERS_OBJ}
        -I${CMAKE_CURRENT_SOURCE_DIR}/include
        -I${IDF_PATH}/components/esp_common/include
        -I${IDF_PATH}/components/soc/${target}/include
        -DESP_PLATFORM
        -mabi=call0
        -mlongcalls
        -mtext-section-literals
        -fno-jump-tables
        -ffunction-sections
        -fdata-sections
        -Os
    DEPENDS ${TEE_HANDLERS_SRC}
    COMMENT "Compiling TEE handlers"
)

# Link entry + handlers with custom linker script
add_custom_command(
    OUTPUT ${TEE_HANDLERS_ELF}
    COMMAND ${CROSS_COMPILE}ld
        -T ${TEE_HANDLERS_LD}
        -o ${TEE_HANDLERS_ELF}
        ${TEE_ENTRY_OBJ}
        ${TEE_HANDLERS_OBJ}
        --gc-sections
    DEPENDS ${TEE_ENTRY_OBJ} ${TEE_HANDLERS_OBJ} ${TEE_HANDLERS_LD}
    COMMENT "Linking TEE handlers at 0x600FE300"
)

# Extract binary
add_custom_command(
    OUTPUT ${TEE_HANDLERS_BIN}
    COMMAND ${CROSS_COMPILE}objcopy
        -O binary
        ${TEE_HANDLERS_ELF}
        ${TEE_HANDLERS_BIN}
    DEPENDS ${TEE_HANDLERS_ELF}
    COMMENT "Extracting TEE handlers binary"
)

# Generate C file with embedded binary using script
set(EMBED_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/embed_binary.py)
add_custom_command(
    OUTPUT ${TEE_HANDLERS_EMBED}
    COMMAND ${Python_EXECUTABLE} ${EMBED_SCRIPT} ${TEE_HANDLERS_BIN} ${TEE_HANDLERS_EMBED}
    DEPENDS ${TEE_HANDLERS_BIN} ${EMBED_SCRIPT}
    COMMENT "Generating embedded TEE handlers"
)

# Add the embedded handlers as a source
target_sources(${COMPONENT_LIB} PRIVATE ${TEE_HANDLERS_EMBED})

# Force linker to include the whole archive (hooks are weak symbols)
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--whole-archive" ${COMPONENT_LIB} "-Wl,--no-whole-archive"
)
