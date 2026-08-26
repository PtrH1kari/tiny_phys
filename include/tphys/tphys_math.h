#ifndef TPHYS_MATH_H
#define TPHYS_MATH_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TP_PI       3.14159265358979323846f
#define TP_EPSILON  1e-6f

typedef struct tp_vec3 { float x, y, z; } tp_vec3;
typedef struct tp_quat { float x, y, z, w; } tp_quat;
typedef struct tp_mat3 { tp_vec3 c0, c1, c2; } tp_mat3;
typedef struct tp_transform { tp_vec3 position; tp_quat rotation; } tp_transform;


// tp_vec3

static inline tp_vec3 tp_v3(float x, float y, float z) {
  tp_vec3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

static inline tp_vec3 tp_v3_add(tp_vec3 a, tp_vec3 b) {
  return tp_v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline tp_vec3 tp_v3_zero(void) {
  return tp_v3(0.0f, 0.0f, 0.0f);
}

static inline tp_vec3 tp_v3_sub(tp_vec3 a, tp_vec3 b) {
  return tp_v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline tp_vec3 tp_v3_scale(tp_vec3 a, float s) {
  return tp_v3(a.x * s, a.y * s, a.z * s);
}

static inline tp_vec3 tp_v3_mad(tp_vec3 a, tp_vec3 b, float s) {
  return tp_v3(a.x + b.x * s, a.y + b.y * s, a.z + b.z * s);
}

static inline float tp_v3_dot(tp_vec3 a, tp_vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline tp_vec3 tp_v3_cross(tp_vec3 a, tp_vec3 b) {
  return tp_v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static inline float tp_v3_length_sq(tp_vec3 a) {
  return tp_v3_dot(a, a);
}

static inline float tp_v3_length(tp_vec3 a) {
  return sqrtf(tp_v3_length_sq(a));
}

static inline tp_vec3 tp_v3_normalize(tp_vec3 a) {
  float len_sq = tp_v3_length_sq(a);
  if (len_sq < TP_EPSILON * TP_EPSILON) {
    return tp_v3_zero();
  }
  float inv_len = 1.0f / sqrtf(len_sq);
  return tp_v3_scale(a, inv_len);
}

// tp_quat

static inline tp_quat tp_quat_make(float x, float y, float z, float w) {
  tp_quat q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

static inline tp_quat tp_quat_identity(void) {
  return tp_quat_make(0, 0, 0, 1);
}

static inline tp_quat tp_quat_from_axis_angle(tp_vec3 axis, float radians) {
  tp_vec3 n = tp_v3_normalize(axis);
  float half = radians / 2;
  float s = sinf(half);
  float c = cosf(half);
  return tp_quat_make(n.x * s, n.y * s, n.z * s, c);
}

static inline tp_quat tp_quat_conjugate(tp_quat q) {
  return tp_quat_make(-q.x, -q.y, -q.z, q.w);
}

static inline tp_quat tp_quat_normalize(tp_quat q) {
  float len_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (len_sq < TP_EPSILON * TP_EPSILON) {
    return tp_quat_identity();
  }
  float inv_len = 1.0f / sqrtf(len_sq);
  return tp_quat_make(q.x * inv_len, q.y * inv_len, q.z * inv_len, q.w * inv_len);
}

static inline tp_quat tp_quat_mul(tp_quat a, tp_quat b) {
  tp_quat q;
  q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  return q;
}

static inline tp_vec3 tp_quat_rotate(tp_quat q, tp_vec3 v) {
  tp_vec3 u = tp_v3(q.x, q.y, q.z);
  tp_vec3 t = tp_v3_scale(tp_v3_cross(u, v), 2);
  tp_vec3 vq = tp_v3_add(tp_v3_add(v, tp_v3_scale(t, q.w)), tp_v3_cross(u, t));
  return vq;
}

static inline tp_vec3 tp_quat_rotate_inv(tp_quat q, tp_vec3 v) {
  return tp_quat_rotate(tp_quat_conjugate(q), v);
}

static inline tp_quat tp_quat_integrate(tp_quat q, tp_vec3 w, float dt) {
    tp_quat wq = tp_quat_make(w.x, w.y, w.z, 0.0f);
    tp_quat dq = tp_quat_mul(wq, q);
    float h = 0.5f * dt;
    return tp_quat_normalize(tp_quat_make(q.x + dq.x * h,
                                            q.y + dq.y * h,
                                            q.z + dq.z * h,
                                            q.w + dq.w * h));
}

// tp_mat3

static inline tp_mat3 tp_mat3_cols(tp_vec3 c0, tp_vec3 c1, tp_vec3 c2) {
  tp_mat3 m;
  m.c0 = c0;
  m.c1 = c1;
  m.c2 = c2;
  return m;
}

static inline tp_mat3 tp_mat3_identity(void) {
  tp_vec3 c0 = tp_v3(1, 0, 0);
  tp_vec3 c1 = tp_v3(0, 1, 0);
  tp_vec3 c2 = tp_v3(0, 0, 1);
  return tp_mat3_cols(c0, c1, c2);
}

static inline tp_mat3 tp_mat3_zero(void) {
  tp_vec3 c0 = tp_v3(0, 0, 0);
  tp_vec3 c1 = tp_v3(0, 0, 0);
  tp_vec3 c2 = tp_v3(0, 0, 0);
  return tp_mat3_cols(c0, c1, c2);
}

static inline tp_mat3 tp_mat3_diagonal(tp_vec3 d) {
  tp_vec3 c0 = tp_v3(d.x, 0, 0);
  tp_vec3 c1 = tp_v3(0, d.y, 0);
  tp_vec3 c2 = tp_v3(0, 0, d.z);
  return tp_mat3_cols(c0, c1, c2);
}

static inline tp_vec3 tp_mat3_mul_v3(tp_mat3 m, tp_vec3 v) {
  tp_vec3 c0 = tp_v3_scale(m.c0, v.x);
  tp_vec3 c1 = tp_v3_scale(m.c1, v.y);
  tp_vec3 c2 = tp_v3_scale(m.c2, v.z);
  return tp_v3_add(tp_v3_add(c0, c1), c2);
}

static inline tp_mat3 tp_mat3_mul(tp_mat3 a, tp_mat3 b) {
  return tp_mat3_cols(tp_mat3_mul_v3(a, b.c0),
                      tp_mat3_mul_v3(a, b.c1),
                      tp_mat3_mul_v3(a, b.c2));
}

static inline tp_mat3 tp_mat3_transpose(tp_mat3 m) {
  return tp_mat3_cols(tp_v3(m.c0.x, m.c1.x, m.c2.x),
                      tp_v3(m.c0.y, m.c1.y, m.c2.y),
                      tp_v3(m.c0.z, m.c1.z, m.c2.z));
}

static inline float tp_mat3_determinant(tp_mat3 m) {
  return tp_v3_dot(m.c0, tp_v3_cross(m.c1, m.c2));
}

static inline tp_mat3 tp_mat3_inverse(tp_mat3 m) {
  float det = tp_mat3_determinant(m);
  if (fabsf(det) < TP_EPSILON) {
    return tp_mat3_zero();
  }
  float inv_det = 1.0f / det;
  tp_vec3 r0 = tp_v3_scale(tp_v3_cross(m.c1, m.c2), inv_det);
  tp_vec3 r1 = tp_v3_scale(tp_v3_cross(m.c2, m.c0), inv_det);
  tp_vec3 r2 = tp_v3_scale(tp_v3_cross(m.c0, m.c1), inv_det);
  return tp_mat3_transpose(tp_mat3_cols(r0, r1, r2));
}

static inline tp_mat3 tp_mat3_from_quat(tp_quat q) {
  float xx = q.x * q.x;
  float yy = q.y * q.y;
  float zz = q.z * q.z;
  float xy = q.x * q.y;
  float xz = q.x * q.z;
  float yz = q.y * q.z;
  float wx = q.w * q.x;
  float wy = q.w * q.y;
  float wz = q.w * q.z;
  tp_vec3 c0 = tp_v3(1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy));
  tp_vec3 c1 = tp_v3(2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx));
  tp_vec3 c2 = tp_v3(2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy));
  return tp_mat3_cols(c0, c1, c2);
}

// tp_transform

static inline tp_transform tp_transform_identity(void) {
  tp_transform t;
  t.position = tp_v3_zero();
  t.rotation = tp_quat_identity();
  return t;
}

static inline tp_transform tp_transform_make(tp_vec3 position, tp_quat rotation) {
  tp_transform t;
  t.position = position;
  t.rotation = rotation;
  return t;
}

static inline tp_vec3 tp_transform_point(tp_transform t, tp_vec3 p) {
  return tp_v3_add(tp_quat_rotate(t.rotation, p), t.position);
}

static inline tp_vec3 tp_transform_dir(tp_transform t, tp_vec3 d) {
  return tp_quat_rotate(t.rotation, d);
}

static inline tp_vec3 tp_transform_point_inv(tp_transform t, tp_vec3 p) {
  return tp_quat_rotate_inv(t.rotation, tp_v3_sub(p, t.position));
}

static inline tp_transform tp_transform_combine(tp_transform a, tp_transform b) {
  tp_transform t;
  t.position = tp_transform_point(a, b.position);
  t.rotation = tp_quat_normalize(tp_quat_mul(a.rotation, b.rotation));
  return t;
}

static inline tp_transform tp_transform_inverse(tp_transform t) {
  tp_quat inv_r = tp_quat_conjugate(t.rotation);
  tp_vec3 inv_p = tp_quat_rotate(inv_r, tp_v3_scale(t.position, -1.0f));
  return tp_transform_make(inv_p, inv_r);
}

#ifdef __cplusplus
}
#endif

#endif /* TPHYS_MATH_H */
