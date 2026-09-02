#pragma once

#include <sys/stat.h>
#include "sdkconfig.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "driver/sdmmc_host.h"

#define MOUNT_POINT "/sd"

// SD/MMC pin assignments (configurable via menuconfig -> SD Card Configuration)
#define PIN_CLK CONFIG_SD_PIN_CLK
#define PIN_CMD CONFIG_SD_PIN_CMD
#define PIN_D0 CONFIG_SD_PIN_D0
#define PIN_D1 CONFIG_SD_PIN_D1
#define PIN_D2 CONFIG_SD_PIN_D2
#define PIN_D3 CONFIG_SD_PIN_D3
#define PIN_CD CONFIG_SD_PIN_CD

extern sdmmc_card_t* card;

void init_sd();
void deinit_sd();
esp_err_t format_sd();
