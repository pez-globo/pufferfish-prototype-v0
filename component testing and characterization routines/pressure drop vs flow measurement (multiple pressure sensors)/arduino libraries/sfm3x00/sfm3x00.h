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

// code adapted for SFM sensors (Hongquan Li 05/03/2020)

#ifndef SFM3x00_H
#define SFM3x00_H

class SFMSensor
{
public:
  const static int SFM3200_I2C_ADDR_40   = 0x40;
  const static int SFM3000_I2C_ADDR_40   = 0x40;

  SFMSensor(uint8_t i2cAddr) : mI2CAddress(i2cAddr) {}

  /**
   * initialize the sensor
   * @return 0 on sucess, error code otherwise
   */
  int init();

  /**
   * get flow offset and scale factor
   * @return 0 on sucess, error code otherwise
   */
  int get_scale_offset();

  /**
   * read sensor data from sensor
   * @return 0 on sucess, error code otherwise
   */
  int read_sample();

  /**
   * Returns the last flow measurement
   * @return last differential pressure value read
   */
  float get_flow() const;

  /**
   * Returns the last temperature value read - does NOT trigger a new measurement
   * @return last temperature value read
   */
  // float getTemperature()  const;

private:
  uint8_t mI2CAddress;
  float mFlow;
  float mTemperature;
  float flow_offset;
  float flow_scale;
  
};

class SFM3200 : public SFMSensor
{
public:
  SFM3200() : SFMSensor(SFMSensor::SFM3200_I2C_ADDR_40) {}
};

class SFM3000 : public SFMSensor
{
public:
  SFM3000() : SFMSensor(SFMSensor::SFM3000_I2C_ADDR_40) {}
};

#endif /* SFM3200_H */
