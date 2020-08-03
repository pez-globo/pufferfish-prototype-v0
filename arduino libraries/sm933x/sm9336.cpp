#include <Arduino.h>
#include "SM9336.h"
#include "i2chelper.h"

SM9336::SM9336()
{
}

int SM9336::read_sample()
{
    const uint8_t CMD_LEN = 1;
    uint8_t cmd[CMD_LEN] = { 0x2E };
    uint8_t ret;
    ret = I2CHelper::i2c_write(SM9336::SM9336_I2C_ADDR_UNPROTECTED, cmd, CMD_LEN);
    if (ret != 0)
      return 1;
    
    const uint8_t DATA_LEN = 6;
    uint8_t data[DATA_LEN] = { 0 };
    ret = I2CHelper::i2c_read(SM9336::SM9336_I2C_ADDR_UNPROTECTED, data, DATA_LEN);
    if (ret != 0)
        return 2;
    
    int16_t dP_raw = (uint16_t)data[3] << 8 | data[2]; // 2's complement
    float tmp = (100.0/65535.0)*float(dP_raw) + 100.0*65536/(2.0*65535);
    mdP = ((dP_MIN-dP_MAX)/(80.0))*tmp - dP_MIN - (dP_MIN-dP_MAX)/8.0;

    return 0;
}

float SM9336::get_dP() const
{
  return mdP;
}
