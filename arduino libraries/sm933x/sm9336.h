#ifndef SM9336_H
#define SM9336_H

class SM9336
{
public:
  const static int SM9336_I2C_ADDR_UNPROTECTED   = 0x6C;
  const static int SM9336_I2C_ADDR_CRCPROTECTED   = 0x6D;

  // SM9336(uint8_t i2cAddr) : mI2CAddress(i2cAddr) {}
    SM9336();

  /**
   * read sensor data from sensor
   * @return 0 on sucess, error code otherwise
   */
  int read_sample();
    
  /**
   * Returns the last flow measurement
   * @return last differential pressure value read
   */
  float get_dP() const;

  /**
   * Returns the last temperature value read - does NOT trigger a new measurement
   * @return last temperature value read
   */
  // float getTemperature()  const;

private:
    uint8_t mI2CAddress;
    float dP_MIN = -250;
    float dP_MAX = 250;
    float mdP;
  
};

#endif /* SFM3200_H */
