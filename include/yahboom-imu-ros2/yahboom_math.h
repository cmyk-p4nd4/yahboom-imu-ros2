#pragma once

#include <cstdint>

#define QUAT_X 0
#define QUAT_Y 1
#define QUAT_Z 2
#define QUAT_W 3

#define YB_EXPORT __attribute__((visibility("default")))

// \brief Represents a Scaled Raw 3D vector.
struct YB_EXPORT ScaledVector3 {
  // \brief Default constructor. Initializes the vector to (0,0,0).
  ScaledVector3(void);

  // \brief Constructor.  Initializes the vector to (sx,sy,sz).
  ScaledVector3(int16_t sx, int16_t sy, int16_t sz);

  constexpr int16_t& getX() {
    return this->data[0];
  }
  constexpr int16_t& getY() {
    return this->data[1];
  }
  constexpr int16_t& getZ() {
    return this->data[2];
  }

  int16_t data[3];
};

// \brief This structure represents a 3d vector.
struct YB_EXPORT Vector3 {
  // \brief Default constructor. Initializes the vector to (0,0,0).
  Vector3(void);

  // \brief Constructor.  Initializes the vector to (x,y,z).
  Vector3(float x, float y, float z);

  constexpr float& getX() {
    return this->data[0];
  }
  constexpr float& getY() {
    return this->data[1];
  }
  constexpr float& getZ() {
    return this->data[2];
  }

  float data[3];
};

// \brief Represents a Scaled Raw Quaternion.
struct YB_EXPORT ScaledQuaternion {
  // \brief Default constructor.  Initializes the quaternion to (0,0,0,1).
  // The quaternion is stored in X-Y-Z-W order.
  ScaledQuaternion();

  // \brief Constructor.  Initializes the quaternion to (x,y,z,w).
  ScaledQuaternion(int16_t x, int16_t y, int16_t z, int16_t w);

  constexpr int16_t& getX() {
    return this->data[0];
  }
  constexpr int16_t& getY() {
    return this->data[1];
  }
  constexpr int16_t& getZ() {
    return this->data[2];
  }
  constexpr int16_t& getW() {
    return this->data[3];
  }

  int16_t data[4];
};

// \brief Represents a quaternion.
struct YB_EXPORT Quaternion {
  // \brief Default constructor.  Initializes the quaternion to (0,0,0,1).
  // The quaternion is stored in X-Y-Z-W order.
  Quaternion();

  // \brief Constructor.  Initializes the quaternion to (x,y,z,w).
  Quaternion(float x, float y, float z, float w);

  constexpr float& getX() {
    return this->data[0];
  }
  constexpr float& getY() {
    return this->data[1];
  }
  constexpr float& getZ() {
    return this->data[2];
  }
  constexpr float& getW() {
    return this->data[3];
  }

  float data[4];
};

// \brief Represents a 3x3 matrix in row-major order.
struct YB_EXPORT Matrix3x3 {
  float data[9];
};

// \brief Represents a 2x2 matrix in row-major order
struct YB_EXPORT Matrix2x2 {
  float data[4];
};

// \brief Convert a ScaledVector3 into a Vector3
// \param src The ScaledVector
// \param scalar constant to scale
// \param dest The resulting normal Vector from the conversion
YB_EXPORT void convertScaledVector(const ScaledVector3* const __restrict__ src, const float scalar, Vector3* __restrict__ dest);

// \brief Convert a Vector3 into a ScaledVector3
// \param src a normal Vector
// \param dest The resulting scaled Vector from the conversion
YB_EXPORT void convertUnscaledVector(const Vector3* const __restrict__ src, const float scalar, ScaledVector3* __restrict__ dest);

// \brief Multiply a vector by a scalar.
// \param vec The vector to multiply by the scalar.
// \param scalar The scalar to multiply the vector by.
YB_EXPORT void vectorMul(Vector3* src, const float scalar, Vector3* dest);

// \brief Takes the sum of two vectors.
// \param src1 The first vector.
// \param src2 The second vector.
// \param dest The output vector from the operator
YB_EXPORT void vectorAdd(Vector3* src1, Vector3* src2, Vector3* dest);

// \brief Takes the difference of two vectors.
// \param from The vector at the tail end of the difference vector.
// \param to The vector at the head end of the difference vector.
// \param diff The resultant difference vector.
YB_EXPORT void vectorDiff(Vector3* from, Vector3* to, Vector3* diff);

// \brief Takes the cross product of two vectors.
// \param src1 The first vector.
// \param src2 The second vector.
// \param dest The destination vector which will recieve the cross product of the two.
YB_EXPORT void vectorCross(Vector3* src1, Vector3* src2, Vector3* dest);

// \brief Takes the dot product of two vectors.
// \param src1 The first vector.
// \param src2 The second vector.
// \return The dot product of the vectors.
YB_EXPORT float vectorDot(Vector3* src1, Vector3* src2);

// \brief Normalizes a vector.  Has no effect if the vector is of 0 magnitude.
// \param vec The vector to normalize.
YB_EXPORT void vectorNormalize(Vector3* vec);

// \brief Finds the length of a vector.
// \param vec The vector to find the length of.
// \return The length of the vector.
YB_EXPORT float vectorLength(Vector3* vec);

// \brief Converts the vectors coordinate space.
// \param vec The Vector3 that is to be converted.
// \param order The order of directions.
// \param negate Negate the order of the same index.
YB_EXPORT void vectorConvertAxes(Vector3* vec, char* fromOrder, char* toOrder, char* negate);

// \brief Convert a ScaledQuaternion into a Quaternion
// \param src The ScaledQuaternion
// \param scalar constant to scale
// \param dest The resulting normal Quaternion from the conversion
YB_EXPORT void convertScaledQuaternion(const ScaledQuaternion* const __restrict__ src, const float scalar, Quaternion* __restrict__ dest);

// \brief Convert a Quaternion into a ScaledQuaternion
// \param src The Quaternion
// \param scalar constant to scale
// \param dest The resulting Scaled Quaternion from the conversion
YB_EXPORT void convertUnscaledQuaternion(const Quaternion* const __restrict__ src, const float scalar, ScaledQuaternion* __restrict__ dest);

// \brief Multiplies two quaternions.
// \param left The first quaternion.
// \param right The second quaternion.
// \param out The resulting quaternion.
YB_EXPORT void quatMul(Quaternion* left, Quaternion* right, Quaternion* out);

// \brief Inverts an Quaternion.
// \param quad The quaternion to invert.
// \param inplace whether the result replace the input quaternion
// \param out The result quaternion, NULL if `inplace` flag is true
YB_EXPORT void quatInverse(Quaternion* quad, bool inplace, Quaternion* out);

// \brief Rotates a vector by the given rotation.
// \param orient The orientation to rotate the vector by.
// \param vec The vector to be rotated. The rotated form will be stored in this vector as well.
YB_EXPORT void quatRotate(Quaternion* quad, Vector3* vec);

// \brief Normalizes a quaternion. Has no effect if the quaternion is of 0 magnitude.
// \param quat The quaternion to normalize.
YB_EXPORT void quatNormalize(Quaternion* quat, bool inplace, Quaternion* out);

// \brief Check if this order follows right-handed rule
// \param order the order to be checked
// \param negate the order to inverse
// \return true if the order follows right-handed rule, false otherwise
YB_EXPORT bool isRightHanded(const char* order, const char* negate);

// \brief Converts the orient coordinate space.
// \param vec The Orient that is to be converted.
// \param order The order of directions.
// \param negate Negate the order of the same index.
YB_EXPORT void quatConvertAxes(Quaternion* quat, char* fromOrder, char* toOrder, char* negate);

// \brief Creates an Quaternion from an axis and an angle.
// \param axis The axis of rotation.
// \param angle The angle in radians.
// \return The new orientation.
YB_EXPORT Quaternion getQuatFromAxisAngle(Vector3* axis, float angle);

// \brief Creates an orientation that would rotate vec_from to vec_to.
// \param vec_from Origin vector.
// \param vec_to Destination vector.
// \return An orientation that, when rotating vec_from, would produce vec_to.
YB_EXPORT Quaternion getQuatFromVectors(Vector3* vec_from, Vector3* vec_to);

// \brief Converts a quaternion to a rotation matrix.
// \param orient Input quaternion.
// \param matrix Output matrix.
YB_EXPORT void convertQuatToRotationMatrix(Quaternion* Quat, Matrix3x3* matrix);
