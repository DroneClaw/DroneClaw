/*
    DroneClaw copyright 2016
*/

#ifndef _MPU
#define _MPU

#define COUNT 2000
#define ADDRESS 0x68
#define START_ADDRESS 0x3b

#define VECTOR_3D 3

#define X 0
#define Y 1
#define Z 2

/** A 3d vector of x, y, z */
template <typename T> struct Vector {
  T x;
  T y;
  T z;
  inline Vector(T *values) {
    x = values[X];
    y = values[Y];
    z = values[Z];
  }
  inline Vector(T _x, T _y, T _z) {
    x = _x;
    y = _y;
    z = _z;
  }
};

/** The data from the MPU in a struct form */
class MPU {
  private:
    static long _offset[VECTOR_3D];
    long _accelerometer[VECTOR_3D];
    int _gyro[VECTOR_3D];
    int _temp;
  public:
    /** Create the instance of this MPU struct with the values from the sensor */
    inline MPU() {
      Wire.beginTransmission(ADDRESS);
      Wire.write(START_ADDRESS);
      Wire.endTransmission();
      Wire.requestFrom(ADDRESS, 14);
      while (Wire.available() < 14);
      _accelerometer[X] = Wire.read() << 8 | Wire.read();
      _accelerometer[Y] = Wire.read() << 8 | Wire.read();
      _accelerometer[Z] = Wire.read() << 8 | Wire.read();
      _temp = Wire.read() << 8 | Wire.read();
      _gyro[X] = Wire.read() << 8 | Wire.read();
      _gyro[Y] = Wire.read() << 8 | Wire.read();
      _gyro[Z] = Wire.read() << 8 | Wire.read();
    }
    /** Return a vector struct of the accelerometer data */
    inline Vector<long> accelerometer() {
      return Vector<long>(_accelerometer);
    }
    /** Return a vector struct of the gyro data */
    inline Vector<int> raw_gyro() {
      return Vector<int>(_gyro);
    }
    /** Return a vector struct of the gyro data */
    inline Vector<int> gyro() {
      return Vector<int>(_gyro[X] - _offset[X], _gyro[Y] - _offset[Y], _gyro[Z] - _offset[Z]);
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
        _offset[0] += mpu._gyro[X];
        _offset[1] += mpu._gyro[Y];
        _offset[2] += mpu._gyro[Z];
        delay(5);
      }
      _offset[0] /= COUNT;
      _offset[1] /= COUNT;
      _offset[2] /= COUNT;
    }
    /** Get the offset */
    inline static Vector<long> calibration() {
      return Vector<long>(_offset);
    }
};

long MPU::_offset[VECTOR_3D] = {};

#endif
