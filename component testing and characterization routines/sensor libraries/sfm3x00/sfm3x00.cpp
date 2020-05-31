/*
 *  Copyright (c) 2018, Sensirion AG <joahnnes.winkelmann@sensirion.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Sensirion AG nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>

#include "sfm3x00.h"
#include "i2chelper.h"

int SFMSensor::init()
{
  // try to read serial number
  const uint8_t CMD_LEN = 2;
  uint8_t cmd0[CMD_LEN] = { 0x31, 0xAE };
  uint8_t cmd1[CMD_LEN] = { 0x31, 0xAF };

  const uint8_t DATA_LEN = 6;
  uint8_t data[DATA_LEN] = { 0 };

  uint8_t ret = I2CHelper::i2c_write(mI2CAddress, cmd0, CMD_LEN);
  if (ret != 0) {
    return 1;
  }
  ret = I2CHelper::i2c_write(mI2CAddress, cmd1, CMD_LEN);
  if (ret != 0) {
    return 2;
  }
  ret = I2CHelper::i2c_read(mI2CAddress, data, DATA_LEN);
  if (ret != 0) {
    return 3;
  }

  // at this point, we don't really care about the data just yet, but
  // we may use that in the future. Either way, the sensor responds, and
  return 0;
}

int SFMSensor::get_scale_offset()
{
  const uint8_t CMD_LEN = 2;
  uint8_t cmd1[CMD_LEN] = { 0x30, 0xDE };
  uint8_t cmd2[CMD_LEN] = { 0x30, 0xDF };
  const uint8_t DATA_LEN = 3;
  uint8_t data[DATA_LEN] = { 0 };
  uint8_t ret;

  // get scale factor
  ret = I2CHelper::i2c_write(mI2CAddress, cmd1, CMD_LEN);
  if (ret != 0) {
    return 1;
  }
  ret = I2CHelper::i2c_read(mI2CAddress, data, DATA_LEN);
  if (ret != 0) {
    return 2;
  }
  // to do: check CRC
  flow_scale = (uint16_t)data[0] << 8 | data[1];

  // get offset
  ret = I2CHelper::i2c_write(mI2CAddress, cmd2, CMD_LEN);
  if (ret != 0) {
    return 1;
  }
  ret = I2CHelper::i2c_read(mI2CAddress, data, DATA_LEN);
  if (ret != 0) {
    return 2;
  }
  // to do: check CRC
  flow_offset = (uint16_t)data[0] << 8 | data[1];

  return 0;
}

int SFMSensor::start_continuous()
{
    const uint8_t CMD_LEN = 2;
    uint8_t cmd[CMD_LEN] = { 0x10, 0x00 };
    uint8_t ret;
    
    ret = I2CHelper::i2c_write(mI2CAddress, cmd, CMD_LEN);
    if (ret != 0) {
      return 1;
    }
    
    return 0;
}

int SFMSensor::read_sample()
{
  const uint8_t DATA_LEN = 3;
  uint8_t data[DATA_LEN] = { 0 };
  uint8_t ret;

  ret = I2CHelper::i2c_read(mI2CAddress, data, DATA_LEN);
  if (ret != 0) {
    return 2;
  }
  // to do: check CRC
  uint16_t flow_raw = (uint16_t)data[0] << 8 | data[1];
  mFlow = (float(flow_raw) - float(flow_offset)) / float(flow_scale);

  return 0;
}

float SFMSensor::get_flow() const
{
  return mFlow;
}
