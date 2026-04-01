#include "yahboom_imu_ros2/yahboom_math.h"

#define _USE_MATH_DEFINES
#include <cmath>

ScaledVector3::ScaledVector3(void) {
  data[0] = INT16_C(0);
  data[1] = INT16_C(0);
  data[2] = INT16_C(0);
}

ScaledVector3::ScaledVector3(int16_t sx, int16_t sy, int16_t sz) {
  data[0] = INT16_C(sx);
  data[1] = INT16_C(sy);
  data[2] = INT16_C(sz);
}

Vector3::Vector3() {
  data[0] = 0.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
}

Vector3::Vector3(float x, float y, float z) {
  data[0] = x;
  data[1] = y;
  data[2] = z;
}

ScaledQuaternion::ScaledQuaternion() {
  data[QUAT_X] = 0;
  data[QUAT_Y] = 0;
  data[QUAT_Z] = 0;
  data[QUAT_W] = 1;
}

ScaledQuaternion::ScaledQuaternion(int16_t x, int16_t y, int16_t z, int16_t w) {
  data[QUAT_X] = x;
  data[QUAT_Y] = y;
  data[QUAT_Z] = z;
  data[QUAT_W] = w;
}

Quaternion::Quaternion() {
  data[QUAT_X] = 0.0f;
  data[QUAT_Y] = 0.0f;
  data[QUAT_Z] = 0.0f;
  data[QUAT_W] = 1.0f;
}

Quaternion::Quaternion(float x, float y, float z, float w) {
  data[QUAT_X] = x;
  data[QUAT_Y] = y;
  data[QUAT_Z] = z;
  data[QUAT_W] = w;
}

void convertScaledVector(const ScaledVector3* const __restrict__ src, const float scalar, Vector3* __restrict__ dest) {
  dest->data[0] = static_cast<float>(src->data[0]) * scalar;
  dest->data[1] = static_cast<float>(src->data[1]) * scalar;
  dest->data[2] = static_cast<float>(src->data[2]) * scalar;
}

void convertUnscaledVector(const Vector3* const __restrict__ src, const float scalar, ScaledVector3* __restrict__ dest) {
  dest->data[0] = static_cast<int16_t>(src->data[0] * scalar);
  dest->data[1] = static_cast<int16_t>(src->data[1] * scalar);
  dest->data[2] = static_cast<int16_t>(src->data[2] * scalar);
}

void vectorMul(Vector3* src, const float scalar, Vector3* dest) {
  dest->data[0] = src->data[0] * scalar;
  dest->data[1] = src->data[1] * scalar;
  dest->data[2] = src->data[2] * scalar;
}

void vectorAdd(Vector3* src1, Vector3* src2, Vector3* dest) {
  dest->data[0] = src1->data[0] + src2->data[0];
  dest->data[1] = src1->data[1] + src2->data[1];
  dest->data[2] = src1->data[2] + src2->data[2];
}

void vectorDiff(Vector3* from, Vector3* to, Vector3* diff) {
  diff->data[0] = to->data[0] - from->data[0];
  diff->data[1] = to->data[1] - from->data[1];
  diff->data[2] = to->data[2] - from->data[2];
}

void vectorCross(Vector3* src1, Vector3* src2, Vector3* dest) {
  dest->data[0] = src1->data[1] * src2->data[2] - src1->data[2] * src2->data[1];
  dest->data[1] = src1->data[2] * src2->data[0] - src1->data[0] * src2->data[2];
  dest->data[2] = src1->data[0] * src2->data[1] - src1->data[1] * src2->data[0];
}

float vectorDot(Vector3* src1, Vector3* src2) { return src1->data[0] * src2->data[0] + src1->data[1] * src2->data[1] + src1->data[2] * src2->data[2]; }

void vectorNormalize(Vector3* vec) {
  float mag2 = vec->data[0] * vec->data[0] + vec->data[1] * vec->data[1] + vec->data[2] * vec->data[2];
  if (mag2 == 0.0f) return;

  float mag = std::sqrt(mag2);

  vec->data[0] /= mag;
  vec->data[1] /= mag;
  vec->data[2] /= mag;
}

float vectorLength(Vector3* vec) {
  float mag2 = vec->data[0] * vec->data[0] + vec->data[1] * vec->data[1] + vec->data[2] * vec->data[2];
  if (mag2 == 0.0f) return 0.0f;

  return std::sqrt(mag2);
}

void vectorConvertAxes(Vector3* vec, char* fromOrder, char* toOrder, char* negate) {
  Vector3 tempVec;
  tempVec.data[fromOrder[0]] = vec->data[toOrder[0]];
  tempVec.data[fromOrder[1]] = vec->data[toOrder[1]];
  tempVec.data[fromOrder[2]] = vec->data[toOrder[2]];

  for (int i = 0; i < 3; i++) {
    tempVec.data[i] *= negate[i];
  }

  *vec = tempVec;
}

void convertScaledQuaternion(const ScaledQuaternion* const __restrict__ src, const float scalar, Quaternion* __restrict__ dest) {
  dest->data[0] = static_cast<float>(src->data[0]) * scalar;
  dest->data[1] = static_cast<float>(src->data[1]) * scalar;
  dest->data[2] = static_cast<float>(src->data[2]) * scalar;
}

void convertUnscaledQuaternion(const Quaternion* const __restrict__ src, const float scalar, ScaledQuaternion* __restrict__ dest) {
  dest->data[0] = static_cast<int16_t>(src->data[0] * scalar);
  dest->data[1] = static_cast<int16_t>(src->data[1] * scalar);
  dest->data[2] = static_cast<int16_t>(src->data[2] * scalar);
}

void quatMul(Quaternion* left, Quaternion* right, Quaternion* out) {
  out->data[QUAT_W] = left->data[QUAT_W] * right->data[QUAT_W] - left->data[QUAT_X] * right->data[QUAT_X] - left->data[QUAT_Y] * right->data[QUAT_Y] - left->data[QUAT_Z] * right->data[QUAT_Z];

  out->data[QUAT_X] = left->data[QUAT_W] * right->data[QUAT_X] + left->data[QUAT_X] * right->data[QUAT_W] + left->data[QUAT_Y] * right->data[QUAT_Z] - left->data[QUAT_Z] * right->data[QUAT_Y];

  out->data[QUAT_Y] = left->data[QUAT_W] * right->data[QUAT_Y] + left->data[QUAT_Y] * right->data[QUAT_W] + left->data[QUAT_Z] * right->data[QUAT_X] - left->data[QUAT_X] * right->data[QUAT_Z];

  out->data[QUAT_Z] = left->data[QUAT_W] * right->data[QUAT_Z] + left->data[QUAT_Z] * right->data[QUAT_W] + left->data[QUAT_X] * right->data[QUAT_Y] - left->data[QUAT_Y] * right->data[QUAT_X];
}

void quatInverse(Quaternion* quat, bool inplace, Quaternion* out) {
  Quaternion* orient = out;
  if (inplace) {
    orient = quat;
  }

  orient->data[QUAT_X] = -orient->data[QUAT_X];
  orient->data[QUAT_Y] = -orient->data[QUAT_Y];
  orient->data[QUAT_Z] = -orient->data[QUAT_Z];
}

void quatRotate(Quaternion* orient, Vector3* vec) {
  Quaternion mrot;
  mrot.data[QUAT_X] = vec->data[0];
  mrot.data[QUAT_Y] = vec->data[1];
  mrot.data[QUAT_Z] = vec->data[2];
  mrot.data[QUAT_W] = 0.0f;

  Quaternion cquat;
  cquat.data[QUAT_X] = -orient->data[QUAT_X];
  cquat.data[QUAT_Y] = -orient->data[QUAT_Y];
  cquat.data[QUAT_Z] = -orient->data[QUAT_Z];
  cquat.data[QUAT_W] = orient->data[QUAT_W];

  Quaternion res;

  quatMul(orient, &mrot, &res);
  quatMul(&res, &cquat, &mrot);

  vec->data[0] = mrot.data[QUAT_X];
  vec->data[1] = mrot.data[QUAT_Y];
  vec->data[2] = mrot.data[QUAT_Z];
}

void quatNormalize(Quaternion* quat, bool inplace, Quaternion* out) {
  Quaternion* orient = inplace ? quat : out;
  float mag = orient->data[0] * orient->data[0];
  mag += orient->data[1] * orient->data[1];
  mag += orient->data[2] * orient->data[2];
  mag += orient->data[3] * orient->data[3];
  mag = std::sqrt(mag);

  if (mag > 0) {
    orient->data[0] /= mag;
    orient->data[1] /= mag;
    orient->data[2] /= mag;
    orient->data[3] /= mag;
  }
}

bool isRightHanded(const char* order, const char* negate) {
  Vector3 axis0, axis1, axis2;

  axis0.data[(int)order[0]] = 1;
  axis1.data[(int)order[1]] = 1;
  axis2.data[(int)order[2]] = 1;

  for (int i = 0; i < 3; i++) {
    axis0.data[i] *= negate[i];
    axis1.data[i] *= negate[i];
    axis2.data[i] *= negate[i];
  }

  Vector3 crossAxis, tempVect;
  vectorCross(&axis0, &axis1, &crossAxis);
  vectorNormalize(&crossAxis);
  vectorDiff(&crossAxis, &axis2, &tempVect);
  bool reg = !(vectorLength(&tempVect) < 0.0001f);

  return reg;
}

void quatConvertAxes(Quaternion* quat, char* fromOrder, char* toOrder, char* negate) {
  Vector3 tempVect = Vector3(quat->data[0], quat->data[1], quat->data[2]);
  vectorConvertAxes(&tempVect, fromOrder, toOrder, negate);

  if (isRightHanded(toOrder, negate)) {
    Vector3 orig(tempVect);
    vectorMul(&orig, -1, &tempVect);
  }

  *quat = Quaternion(tempVect.data[0], tempVect.data[1], tempVect.data[2], quat->data[3]);
}

Quaternion getQuatFromAxisAngle(Vector3* axis, float angle) {
  Vector3 n_axis = *axis;
  vectorNormalize(&n_axis);

  float ang_cos = cosf(angle / 2.0f);
  float ang_sin = sinf(angle / 2.0f);

  Quaternion out;
  out.data[QUAT_X] = n_axis.data[0] * ang_sin;
  out.data[QUAT_Y] = n_axis.data[1] * ang_sin;
  out.data[QUAT_Z] = n_axis.data[2] * ang_sin;
  out.data[QUAT_W] = ang_cos;

  return out;
}

Quaternion getQuatFromVectors(Vector3* vec_from, Vector3* vec_to) {
  Vector3 n_vec_from = *vec_from;
  Vector3 n_vec_to = *vec_to;

  vectorNormalize(&n_vec_from);
  vectorNormalize(&n_vec_to);

  Vector3 vec_cross;
  vectorCross(&n_vec_from, &n_vec_to, &vec_cross);
  float dot_val = vectorDot(&n_vec_from, &n_vec_to);

  if (dot_val <= -0.9999) {
    dot_val = -1.0f;
    Vector3 temp_vect;

    Vector3 tmp{1, 0, 0};
    vectorCross(&tmp, &n_vec_from, &temp_vect);
    if (vectorLength(&temp_vect) < 0.00001) {
      tmp = Vector3(0, 0, 1);
      vectorCross(&tmp, &n_vec_from, &temp_vect);
    }
    vectorNormalize(&temp_vect);
    Quaternion orient = getQuatFromAxisAngle(&temp_vect, (float)M_PI);
    return orient;
  } else if (dot_val >= 0.9999) {
    return Quaternion(0.0, 0.0, 0.0, 1.0f);
  }

  float orient_angle = acosf(dot_val);

  Quaternion orient = getQuatFromAxisAngle(&vec_cross, orient_angle);
  quatNormalize(&orient, true, nullptr);
  return orient;
}

void convertQuatToRotationMatrix(Quaternion* orient, Matrix3x3* matrix) {
  float fTx = 2.0f * orient->data[QUAT_X];
  float fTy = 2.0f * orient->data[QUAT_Y];
  float fTz = 2.0f * orient->data[QUAT_Z];
  float fTwx = fTx * orient->data[QUAT_W];
  float fTwy = fTy * orient->data[QUAT_W];
  float fTwz = fTz * orient->data[QUAT_W];
  float fTxx = fTx * orient->data[QUAT_X];
  float fTxy = fTy * orient->data[QUAT_X];
  float fTxz = fTz * orient->data[QUAT_X];
  float fTyy = fTy * orient->data[QUAT_Y];
  float fTyz = fTz * orient->data[QUAT_Y];
  float fTzz = fTz * orient->data[QUAT_Z];

  matrix->data[0] = 1.0f - (fTyy + fTzz);
  matrix->data[1] = fTxy - fTwz;
  matrix->data[2] = fTxz + fTwy;
  matrix->data[3] = fTxy + fTwz;
  matrix->data[4] = 1.0f - (fTxx + fTzz);
  matrix->data[5] = fTyz - fTwx;
  matrix->data[6] = fTxz - fTwy;
  matrix->data[7] = fTyz + fTwx;
  matrix->data[8] = 1.0f - (fTxx + fTyy);
}
