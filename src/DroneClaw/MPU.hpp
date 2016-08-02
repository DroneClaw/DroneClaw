/*
    DroneClaw copyright 2016
*/

#ifndef _MPU
#define _MPU

#define COUNT 2000
#define ADDRESS 0x68
#define START_ADDRESS 0x3b

class MPU;

/** The data from the MPU in a struct form */
class MPU {
  private:
    static long offset[3];
  public:
    long accel_x, accel_y, accel_z;
    int gyro_x, gyro_y, gyro_z;
    int temp;
    /** Create the instance of this MPU struct with the values from the sensor */
    inline MPU() {
      Wire.beginTransmission(ADDRESS);
      Wire.write(START_ADDRESS);
      Wire.endTransmission();
      Wire.requestFrom(ADDRESS, 14);
      while (Wire.available() < 14);
      accel_x = Wire.read() << 8 | Wire.read();
      accel_y = Wire.read() << 8 | Wire.read();
      accel_z = Wire.read() << 8 | Wire.read();
      temp = Wire.read() << 8 | Wire.read();
      gyro_x = Wire.read() << 8 | Wire.read();
      gyro_y = Wire.read() << 8 | Wire.read();
      gyro_z = Wire.read() << 8 | Wire.read();
    }
    /** Inits the Wire lib and the sensors */
    inline static void init() {
      Wire.begin();
      Wire.beginTransmission(ADDRESS);
      Wire.write(0x6b);
      Wire.write(0);
      Wire.endTransmission();
      Wire.beginTransmission(ADDRESS);
      Wire.write(0x1c);
      Wire.write(0x10);
      Wire.endTransmission();
      Wire.beginTransmission(ADDRESS);
      Wire.write(0x1b);
      Wire.write(0x08);
      Wire.endTransmission();
      Wire.beginTransmission(ADDRESS);
      Wire.write(0x1a);
      Wire.write(0x03);
      Wire.endTransmission();
    }
    /** Calibrate the gyro */
    inline static void calibrate() {
      for (int i = 0 ; i < COUNT; i++) {
        MPU mpu;
        offset[0] += mpu.gyro_x;
        offset[1] += mpu.gyro_y;
        offset[2] += mpu.gyro_z;
        delay(5);
      }
      offset[0] /= COUNT;
      offset[1] /= COUNT;
      offset[2] /= COUNT;
    }
    /** Get the offset */
    inline static long* calibration() {
      return offset;
    }
};

long MPU::offset[3] = {}; // x, y, z

#endif
