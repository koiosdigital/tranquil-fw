#pragma once

#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "driver/sdmmc_host.h"

#define MOUNT_POINT "/sd"

#define PIN_CLK 6
#define PIN_CMD 7
#define PIN_D0 5
#define PIN_D1 4
#define PIN_D2 16
#define PIN_D3 15
#define PIN_CD 8

extern sdmmc_card_t* card;

void init_sd();
void deinit_sd();
