#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"
#include "build/build_config.h"

#include "pg/pg.h"
#include "common/time.h"
#include "drivers/io_types.h"

struct wifiDev_s;
// typedef void (*wifiOpInitFuncPtr)(struct wifiDev_s * dev);
// typedef void (*wifiOpStartFuncPtr)(struct wifiDev_s * dev);
// typedef int32_t (*wifiOpReadFuncPtr)(struct wifiDev_s * dev);

typedef enum {
    WIFI_NONE        = 0,
    WIFI_ATK_MW8266D      = 1,
} wifiType_e;

typedef struct wifiDev_s {
    wifiType_e dev;
    timeMs_t delayMs;

    timeMs_t lastValidResponseTimeMs;
    // function pointers
    // wifiOpInitFuncPtr init;
    // wifiOpStartFuncPtr update;
    // wifiOpReadFuncPtr read;
} wifiDev_t;

//PG_DECLARE(wifiDev_t, wifiDev);

bool wifiATK8266Detect(wifiDev_t *dev);

void atk_8266_send_InitCmd();
void wifiUpdate(wifiDev_t *dev);