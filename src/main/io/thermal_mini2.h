#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"
#include "config/parameter_group.h"
#include "io/serial.h"

// Configuration for Mini2 thermal module control
typedef struct thermalMini2Config_s {
    bool     enabled;                 // Enable Mini2 control
    int8_t   serialPortId;            // serialPortIdentifier_e (SERIAL_PORT_NONE to auto)
    uint8_t  auxZoomIndex;            // 0..(MAX_AUX_CHANNEL_COUNT-1) index for zoom knob
    uint8_t  auxPaletteIndex;         // 0..(MAX_AUX_CHANNEL_COUNT-1) index for palette momentary switch
    uint8_t  defaultPalette;          // default palette code (0 = White Hot)
} thermalMini2Config_t;

PG_DECLARE(thermalMini2Config_t, thermalMini2Config);

bool thermalMini2Init(void);
void thermalMini2Process(timeUs_t currentTimeUs);
