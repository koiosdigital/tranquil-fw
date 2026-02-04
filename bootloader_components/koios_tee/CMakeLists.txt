idf_component_register(
    SRCS
        "src/tee_init.c"
        "src/tee_wcl.c"
        "src/tee_handlers.c"
        "src/tee_pms.c"
        "src/tee_trampoline.S"
        "src/tee_exit_stub.S"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES bootloader_support bootloader esp_app_format
)

# Force linker to include the whole archive (hooks are weak symbols)
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--whole-archive" ${COMPONENT_LIB} "-Wl,--no-whole-archive"
)
