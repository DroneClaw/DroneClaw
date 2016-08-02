/*
    DroneClaw copyright 2016
*/

#ifndef _VECTOR_
#define _VECTOR_

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

#endif

