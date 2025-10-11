#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"
#include "config/parameter_group.h"
#include "io/serial.h"

// Configuration for Mini2 thermal module control
typedef enum {
    THERMAL_MINI2_FFC_MODE_MANUAL = 0,   // only manual triggers (RC/off)
    THERMAL_MINI2_FFC_MODE_AUTO = 1,     // periodic auto triggers
    THERMAL_MINI2_FFC_MODE_AUTO_PLUS_RC = 2 // periodic auto + allow RC manual triggers
} thermalMini2FfcMode_e;

typedef struct thermalMini2Config_s {
    bool     enabled;                 // Enable Mini2 control
    int8_t   serialPortId;            // serialPortIdentifier_e (SERIAL_PORT_NONE to auto)
    uint8_t  auxZoomIndex;            // 0..(MAX_AUX_CHANNEL_COUNT-1) index for zoom knob
    uint8_t  auxPaletteIndex;         // 0..(MAX_AUX_CHANNEL_COUNT-1) index for palette momentary switch
    uint8_t  defaultPalette;          // default palette code (0 = White Hot)

    // FFC (shutter) configuration
    uint8_t  ffcMode;                 // thermalMini2FfcMode_e
    uint8_t  ffcAuxIndex;             // AUX channel index for manual FFC trigger
    uint16_t ffcMinIntervalMs;        // Minimum interval between FFC triggers
    bool     ffcOnBoot;               // Trigger FFC on boot/init
    bool     allowRcWhileArmed;       // Allow RC trigger while armed (default true)
} thermalMini2Config_t;

PG_DECLARE(thermalMini2Config_t, thermalMini2Config);

bool thermalMini2Init(void);
void thermalMini2Process(timeUs_t currentTimeUs);
