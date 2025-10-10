#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/streambuf.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"
#include "drivers/bus.h"
#ifdef USE_I2C
#include "drivers/bus_i2c.h"
#endif

#include "io/serial.h"
#include "rx/rx.h"

#include "thermal_mini2.h"

PG_REGISTER_WITH_RESET_FN(thermalMini2Config_t, thermalMini2Config, PG_THERMAL_MINI2_CONFIG, 0);

static void pgResetFn_thermalMini2Config(thermalMini2Config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->enabled = false;
    c->useI2C = true;
    c->serialPortId = SERIAL_PORT_NONE; // auto
    c->i2cBus = 1;                      // use I2C bus 1 by default
    c->i2cAddress = 0x00;               // set the module 7-bit address here
    c->auxZoomIndex = 1;                // AUX2 default as knob
    c->auxPaletteIndex = 0;             // AUX1 default as momentary
    c->defaultPalette = 0;              // White Hot
}

// Mini2 command framing
// Preamble: 55 43 49 12, then 0x00, then Instruction Data, then 0x00 0x00 reserved, then CRC16-MODBUS (LSB first) of Instruction Data

static serialPort_t *mini2Serial = NULL;
#ifdef USE_I2C
static I2CDevice mini2I2CDev = I2CINVALID;
static uint8_t mini2I2CAddr = 0x00;
#endif

static uint8_t currentPaletteIndex = 0;
static uint8_t paletteCodes[] = {
    0x00, // White Hot
    0x02, // Gold Sepia
    0x03, // Ironbow
    0x04, // Rainbow
    0x05, // Night
    0x06, // Aurora
    0x07, // Red_Hot
    0x08, // Jungle
    0x09, // Medical
    0x0A, // Black_Hot
    0x0B  // Golden Red Glory_Hot
};

static bool lastPalettePressed = false;
static uint8_t lastZoomX10 = 10; // 1.0x

static uint16_t crc16_modbus_update(uint16_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x0001) {
            crc = (crc >> 1) ^ 0xA001;
        } else {
            crc = (crc >> 1);
        }
    }
    return crc;
}

static uint16_t crc16_modbus_compute(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc = crc16_modbus_update(crc, data[i]);
    }
    return crc;
}

static void mini2SendFrame(const uint8_t *instr, uint32_t instrLen)
{
    uint8_t buf[64];
    uint32_t idx = 0;
    // Header
    buf[idx++] = 0x55; buf[idx++] = 0x43; buf[idx++] = 0x49; buf[idx++] = 0x12;
    buf[idx++] = 0x00; // reserved seat
    // Instruction data
    memcpy(&buf[idx], instr, instrLen);
    idx += instrLen;
    // Reserved 0x0000
    buf[idx++] = 0x00; buf[idx++] = 0x00;
    // CRC16 MODBUS over instruction data
    const uint16_t crc = crc16_modbus_compute(instr, instrLen);
    buf[idx++] = (uint8_t)(crc & 0xFF);      // LSB first
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    if (mini2Serial) {
        serialWriteBuf(mini2Serial, buf, idx);
    }
#ifdef USE_I2C
    else if (mini2I2CDev != I2CINVALID) {
        // If I2C is used, write the frame as-is (raw master transmit)
        i2cWriteBuffer(mini2I2CDev, mini2I2CAddr, 0xFF, idx, buf, true);
    }
#endif
}

// Helpers to build specific commands
static void cmdSetAnalogPAL(void)
{
    // Disable analog: Class 0x10, Index 0x10, Sub 0x4A, reserved 0x00, Para1[0]=0x00
    uint8_t instr1[14] = {0};
    instr1[0] = 0x10; instr1[1] = 0x10; instr1[2] = 0x4A; instr1[3] = 0x00; // reserved
    // Para1[0..3]
    instr1[4] = 0x00; instr1[5] = 0x00; instr1[6] = 0x00; instr1[7] = 0x00;
    // Para2[0..3]
    instr1[8] = 0x00; instr1[9] = 0x00; instr1[10] = 0x00; instr1[11] = 0x00;
    // Len[0..1]
    instr1[12] = 0x00; instr1[13] = 0x00;
    mini2SendFrame(instr1, sizeof(instr1));

    // Enable PAL: Para1[0]=0x01, Para1[1]=0x01
    uint8_t instr2[14] = {0};
    instr2[0] = 0x10; instr2[1] = 0x10; instr2[2] = 0x4A; instr2[3] = 0x00;
    instr2[4] = 0x01; instr2[5] = 0x01; instr2[6] = 0x00; instr2[7] = 0x00;
    instr2[8] = 0x00; instr2[9] = 0x00; instr2[10] = 0x00; instr2[11] = 0x00;
    instr2[12] = 0x00; instr2[13] = 0x00;
    mini2SendFrame(instr2, sizeof(instr2));
}

static void cmdSetPalette(uint8_t code)
{
    // False Color set: Class 0x10, Index 0x03, Sub 0x45, reserved 0x00, Para1[1]=code
    uint8_t instr[14] = {0};
    instr[0] = 0x10; instr[1] = 0x03; instr[2] = 0x45; instr[3] = 0x00;
    instr[4] = 0x00; instr[5] = code; instr[6] = 0x00; instr[7] = 0x00;
    instr[8] = 0x00; instr[9] = 0x00; instr[10] = 0x00; instr[11] = 0x00;
    instr[12] = 0x00; instr[13] = 0x00;
    mini2SendFrame(instr, sizeof(instr));
}

static void cmdSetCenterZoomX10(uint8_t zoomX10)
{
    // Electronic zoom-center: Class 0x01, Index 0x31, Sub 0x42, reserved 0x00, Para1[1]=zoomX10
    uint8_t instr[14] = {0};
    instr[0] = 0x01; instr[1] = 0x31; instr[2] = 0x42; instr[3] = 0x00;
    instr[4] = 0x00; instr[5] = zoomX10; instr[6] = 0x00; instr[7] = 0x00;
    instr[8] = 0x00; instr[9] = 0x00; instr[10] = 0x00; instr[11] = 0x00;
    instr[12] = 0x00; instr[13] = 0x00;
    mini2SendFrame(instr, sizeof(instr));
}

static serialPort_t *openConfiguredSerial(void)
{
    const thermalMini2Config_t *cfg = thermalMini2Config();

    serialPort_t *port = NULL;
    if (cfg->serialPortId != SERIAL_PORT_NONE) {
        port = openSerialPort((serialPortIdentifier_e)cfg->serialPortId, FUNCTION_NONE, NULL, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
        return port;
    }

    // Auto: try FUNCTION_RCDEVICE first
    serialPortConfig_t *spc = findSerialPortConfig(FUNCTION_RCDEVICE);
    if (spc) {
        port = openSerialPort(spc->identifier, FUNCTION_NONE, NULL, NULL, baudRates[BAUD_115200], MODE_RXTX, SERIAL_NOT_INVERTED);
    }
    return port;
}

bool thermalMini2Init(void)
{
    const thermalMini2Config_t *cfg = thermalMini2Config();
    if (!cfg->enabled) {
        return false;
    }

    if (cfg->useI2C) {
#ifdef USE_I2C
        // Map numeric bus to I2CDEV enum; expect 1..3
        if (cfg->i2cBus == 1) mini2I2CDev = I2CDEV_1;
        else if (cfg->i2cBus == 2) mini2I2CDev = I2CDEV_2;
        else if (cfg->i2cBus == 3) mini2I2CDev = I2CDEV_3;
#if defined(USE_I2C_DEVICE_4)
        else if (cfg->i2cBus == 4) mini2I2CDev = I2CDEV_4;
#endif
        else mini2I2CDev = I2CDEV_1; // default to bus 1

        mini2I2CAddr = cfg->i2cAddress;
        if (mini2I2CAddr != 0) {
            i2cInit(mini2I2CDev);
        }
#endif
    } else {
        mini2Serial = openConfiguredSerial();
    }

    // Initialize output format and default palette
    cmdSetAnalogPAL();
    currentPaletteIndex = cfg->defaultPalette % (sizeof(paletteCodes) / sizeof(paletteCodes[0]));
    cmdSetPalette(paletteCodes[currentPaletteIndex]);
    cmdSetCenterZoomX10(lastZoomX10);
    return true;
}

static uint16_t pwmToZoomX10(uint16_t pwm)
{
    if (pwm < 1000) pwm = 1000;
    if (pwm > 2000) pwm = 2000;
    // Map 1000..2000 to 10..80 (1.0x .. 8.0x)
    const float t = (pwm - 1000) / 1000.0f;
    const float val = 10.0f + t * 70.0f;
    uint16_t z = (uint16_t)lrintf(val);
    if (z < 10) z = 10;
    if (z > 80) z = 80;
    return (uint8_t)z;
}

void thermalMini2Process(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    const thermalMini2Config_t *cfg = thermalMini2Config();
    if (!cfg->enabled) {
        return;
    }

    // Palette control via momentary: rising edge over ~1500us threshold
    const uint8_t palChan = cfg->auxPaletteIndex;
    const uint16_t palPwm = rxGetChannelValue(NON_AUX_CHANNEL_COUNT + palChan);
    const bool pressed = palPwm > 1500;
    if (pressed && !lastPalettePressed) {
        currentPaletteIndex = (currentPaletteIndex + 1) % (sizeof(paletteCodes) / sizeof(paletteCodes[0]));
        cmdSetPalette(paletteCodes[currentPaletteIndex]);
    }
    lastPalettePressed = pressed;

    // Zoom via knob
    const uint8_t zoomChan = cfg->auxZoomIndex;
    const uint16_t zoomPwm = rxGetChannelValue(NON_AUX_CHANNEL_COUNT + zoomChan);
    const uint8_t targetZoom = (uint8_t)pwmToZoomX10(zoomPwm);
    if (targetZoom != lastZoomX10) {
        // Reduce bus spam by only sending on meaningful change (>=1 increment)
        lastZoomX10 = targetZoom;
        cmdSetCenterZoomX10(lastZoomX10);
    }
}
