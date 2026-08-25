/*
 * tphys_math.h -- vector / quaternion / matrix math for tiny_phys
 *
 * Conventions (do not change these without changing everything):
 *   - Right-handed coordinate system, Y up.
 *   - Quaternions are stored (x, y, z, w) with w scalar LAST, matching glTF.
 *   - tp_mat3 is stored as three COLUMN vectors (c0, c1, c2).
 *   - Units are SI: metres, kilograms, seconds, radians.
 *
 * Everything here is `static inline` and header-only. This file is included
 * from C++ (tiny_engine), so it deliberately avoids compound literals and
 * designated initialisers, which are not portable C++.
 */
#ifndef TPHYS_MATH_H
#define TPHYS_MATH_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TP_PI       3.14159265358979323846f
#define TP_EPSILON  1e-6f

/* ------------------------------------------------------------------ types */

typedef struct tp_vec3 {
    float x, y, z;
} tp_vec3;

typedef struct tp_quat {
    float x, y, z, w;   /* w is the scalar part, stored last (glTF order) */
} tp_quat;

typedef struct tp_mat3 {
    tp_vec3 c0, c1, c2; /* columns */
} tp_mat3;

typedef struct tp_transform {
    tp_vec3 position;
    tp_quat rotation;
} tp_transform;

/* ------------------------------------------------------------------ vec3 */

static inline tp_vec3 tp_v3(float x, float y, float z) {
    tp_vec3 v; v.x = x; v.y = y; v.z = z; return v;
}
static inline tp_vec3 tp_v3_zero(void)     { return tp_v3(0.0f, 0.0f, 0.0f); }
static inline tp_vec3 tp_v3_splat(float s) { return tp_v3(s, s, s); }

static inline tp_vec3 tp_v3_add(tp_vec3 a, tp_vec3 b) {
    return tp_v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline tp_vec3 tp_v3_sub(tp_vec3 a, tp_vec3 b) {
    return tp_v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline tp_vec3 tp_v3_mul(tp_vec3 a, tp_vec3 b) {   /* component-wise */
    return tp_v3(a.x * b.x, a.y * b.y, a.z * b.z);
}
static inline tp_vec3 tp_v3_scale(tp_vec3 a, float s) {
    return tp_v3(a.x * s, a.y * s, a.z * s);
}
static inline tp_vec3 tp_v3_neg(tp_vec3 a) {
    return tp_v3(-a.x, -a.y, -a.z);
}
/* a + b * s -- the single most common operation in an integrator. */
static inline tp_vec3 tp_v3_mad(tp_vec3 a, tp_vec3 b, float s) {
    return tp_v3(a.x + b.x * s, a.y + b.y * s, a.z + b.z * s);
}
static inline float tp_v3_dot(tp_vec3 a, tp_vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline tp_vec3 tp_v3_cross(tp_vec3 a, tp_vec3 b) {
    return tp_v3(a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x);
}
static inline float tp_v3_length_sq(tp_vec3 a) { return tp_v3_dot(a, a); }
static inline float tp_v3_length(tp_vec3 a)    { return sqrtf(tp_v3_dot(a, a)); }

static inline float tp_v3_distance(tp_vec3 a, tp_vec3 b) {
    return tp_v3_length(tp_v3_sub(a, b));
}
/* Returns the zero vector for degenerate input rather than NaN. */
static inline tp_vec3 tp_v3_normalize(tp_vec3 a) {
    float len_sq = tp_v3_length_sq(a);
    if (len_sq < TP_EPSILON * TP_EPSILON) return tp_v3_zero();
    return tp_v3_scale(a, 1.0f / sqrtf(len_sq));
}
static inline tp_vec3 tp_v3_lerp(tp_vec3 a, tp_vec3 b, float t) {
    return tp_v3(a.x + (b.x - a.x) * t,
                 a.y + (b.y - a.y) * t,
                 a.z + (b.z - a.z) * t);
}
static inline tp_vec3 tp_v3_min(tp_vec3 a, tp_vec3 b) {
    return tp_v3(a.x < b.x ? a.x : b.x,
                 a.y < b.y ? a.y : b.y,
                 a.z < b.z ? a.z : b.z);
}
static inline tp_vec3 tp_v3_max(tp_vec3 a, tp_vec3 b) {
    return tp_v3(a.x > b.x ? a.x : b.x,
                 a.y > b.y ? a.y : b.y,
                 a.z > b.z ? a.z : b.z);
}
static inline tp_vec3 tp_v3_abs(tp_vec3 a) {
    return tp_v3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
}

/* ------------------------------------------------------------------ quat */

static inline tp_quat tp_quat_make(float x, float y, float z, float w) {
    tp_quat q; q.x = x; q.y = y; q.z = z; q.w = w; return q;
}
static inline tp_quat tp_quat_identity(void) {
    return tp_quat_make(0.0f, 0.0f, 0.0f, 1.0f);
}
static inline tp_quat tp_quat_from_axis_angle(tp_vec3 axis, float radians) {
    tp_vec3 n = tp_v3_normalize(axis);
    float h = radians * 0.5f;
    float s = sinf(h);
    return tp_quat_make(n.x * s, n.y * s, n.z * s, cosf(h));
}
/* Result applies b first, then a  (same order as matrix multiplication). */
static inline tp_quat tp_quat_mul(tp_quat a, tp_quat b) {
    return tp_quat_make(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}
static inline tp_quat tp_quat_conjugate(tp_quat q) {
    return tp_quat_make(-q.x, -q.y, -q.z, q.w);
}
static inline tp_quat tp_quat_normalize(tp_quat q) {
    float len_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (len_sq < TP_EPSILON * TP_EPSILON) return tp_quat_identity();
    float inv = 1.0f / sqrtf(len_sq);
    return tp_quat_make(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
}
/* v' = q * v * q^-1, using the two-cross-product form (fewer ops). */
static inline tp_vec3 tp_quat_rotate(tp_quat q, tp_vec3 v) {
    tp_vec3 u = tp_v3(q.x, q.y, q.z);
    tp_vec3 t = tp_v3_scale(tp_v3_cross(u, v), 2.0f);
    return tp_v3_add(tp_v3_mad(v, t, q.w), tp_v3_cross(u, t));
}
/* Inverse rotation. Assumes q is unit-length, which it always is here. */
static inline tp_vec3 tp_quat_rotate_inv(tp_quat q, tp_vec3 v) {
    return tp_quat_rotate(tp_quat_conjugate(q), v);
}
/*
 * Integrate an orientation by angular velocity w (rad/s) over dt.
 *   dq/dt = 0.5 * (w as pure quaternion) * q
 * First-order accurate; renormalising each step keeps drift bounded.
 */
static inline tp_quat tp_quat_integrate(tp_quat q, tp_vec3 w, float dt) {
    tp_quat wq = tp_quat_make(w.x, w.y, w.z, 0.0f);
    tp_quat dq = tp_quat_mul(wq, q);
    float h = 0.5f * dt;
    return tp_quat_normalize(tp_quat_make(q.x + dq.x * h,
                                          q.y + dq.y * h,
                                          q.z + dq.z * h,
                                          q.w + dq.w * h));
}

/* ------------------------------------------------------------------ mat3 */

static inline tp_mat3 tp_mat3_cols(tp_vec3 c0, tp_vec3 c1, tp_vec3 c2) {
    tp_mat3 m; m.c0 = c0; m.c1 = c1; m.c2 = c2; return m;
}
static inline tp_mat3 tp_mat3_identity(void) {
    return tp_mat3_cols(tp_v3(1.0f, 0.0f, 0.0f),
                        tp_v3(0.0f, 1.0f, 0.0f),
                        tp_v3(0.0f, 0.0f, 1.0f));
}
static inline tp_mat3 tp_mat3_diagonal(tp_vec3 d) {
    return tp_mat3_cols(tp_v3(d.x, 0.0f, 0.0f),
                        tp_v3(0.0f, d.y, 0.0f),
                        tp_v3(0.0f, 0.0f, d.z));
}
static inline tp_mat3 tp_mat3_zero(void) {
    return tp_mat3_cols(tp_v3_zero(), tp_v3_zero(), tp_v3_zero());
}
static inline tp_vec3 tp_mat3_mul_v3(tp_mat3 m, tp_vec3 v) {
    tp_vec3 r = tp_v3_scale(m.c0, v.x);
    r = tp_v3_mad(r, m.c1, v.y);
    r = tp_v3_mad(r, m.c2, v.z);
    return r;
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
/* Returns the zero matrix if m is singular. Check with tp_mat3_determinant. */
static inline tp_mat3 tp_mat3_inverse(tp_mat3 m) {
    float det = tp_mat3_determinant(m);
    if (fabsf(det) < TP_EPSILON) return tp_mat3_zero();
    float inv_det = 1.0f / det;
    /* Rows of the inverse are the scaled cross products of the columns. */
    tp_vec3 r0 = tp_v3_scale(tp_v3_cross(m.c1, m.c2), inv_det);
    tp_vec3 r1 = tp_v3_scale(tp_v3_cross(m.c2, m.c0), inv_det);
    tp_vec3 r2 = tp_v3_scale(tp_v3_cross(m.c0, m.c1), inv_det);
    return tp_mat3_cols(tp_v3(r0.x, r1.x, r2.x),
                        tp_v3(r0.y, r1.y, r2.y),
                        tp_v3(r0.z, r1.z, r2.z));
}
static inline tp_mat3 tp_mat3_from_quat(tp_quat q) {
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    return tp_mat3_cols(
        tp_v3(1.0f - 2.0f * (yy + zz),        2.0f * (xy + wz),        2.0f * (xz - wy)),
        tp_v3(       2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz),        2.0f * (yz + wx)),
        tp_v3(       2.0f * (xz + wy),        2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy)));
}

/* ------------------------------------------------------------- transform */

static inline tp_transform tp_transform_identity(void) {
    tp_transform t;
    t.position = tp_v3_zero();
    t.rotation = tp_quat_identity();
    return t;
}
static inline tp_transform tp_transform_make(tp_vec3 position, tp_quat rotation) {
    tp_transform t; t.position = position; t.rotation = rotation; return t;
}
/* Local point -> world space. */
static inline tp_vec3 tp_transform_point(tp_transform t, tp_vec3 p) {
    return tp_v3_add(tp_quat_rotate(t.rotation, p), t.position);
}
/* Local direction -> world space (no translation). */
static inline tp_vec3 tp_transform_dir(tp_transform t, tp_vec3 d) {
    return tp_quat_rotate(t.rotation, d);
}
/* World point -> local space. */
static inline tp_vec3 tp_transform_point_inv(tp_transform t, tp_vec3 p) {
    return tp_quat_rotate_inv(t.rotation, tp_v3_sub(p, t.position));
}
/* Apply `b` first, then `a`. */
static inline tp_transform tp_transform_combine(tp_transform a, tp_transform b) {
    return tp_transform_make(tp_transform_point(a, b.position),
                             tp_quat_normalize(tp_quat_mul(a.rotation, b.rotation)));
}
static inline tp_transform tp_transform_inverse(tp_transform t) {
    tp_quat inv_r = tp_quat_conjugate(t.rotation);
    return tp_transform_make(tp_quat_rotate(inv_r, tp_v3_neg(t.position)), inv_r);
}

#ifdef __cplusplus
}
#endif

#endif /* TPHYS_MATH_H */
