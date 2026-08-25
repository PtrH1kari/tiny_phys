#include "tp_test.h"
#include "tphys/tphys_math.h"

int main(void) {
    const float eps = 1e-5f;

    TP_TEST("cross product is right-handed: x cross y = z") {
        tp_vec3 c = tp_v3_cross(tp_v3(1, 0, 0), tp_v3(0, 1, 0));
        TP_CHECK_V3_NEAR(c, 0.0f, 0.0f, 1.0f, eps);
    }

    TP_TEST("normalize gives unit length, and zero for degenerate input") {
        tp_vec3 n = tp_v3_normalize(tp_v3(3, 4, 0));
        TP_CHECK_NEAR(tp_v3_length(n), 1.0f, eps);
        TP_CHECK_NEAR(tp_v3_length(tp_v3_normalize(tp_v3_zero())), 0.0f, eps);
    }

    TP_TEST("90 degrees about Y maps +Z to +X") {
        tp_quat q = tp_quat_from_axis_angle(tp_v3(0, 1, 0), TP_PI * 0.5f);
        tp_vec3 v = tp_quat_rotate(q, tp_v3(0, 0, 1));
        TP_CHECK_V3_NEAR(v, 1.0f, 0.0f, 0.0f, eps);
    }

    TP_TEST("rotate then rotate_inv is the identity") {
        tp_quat q = tp_quat_from_axis_angle(tp_v3(1, 2, 3), 0.7f);
        tp_vec3 v = tp_v3(0.3f, -1.2f, 5.0f);
        tp_vec3 r = tp_quat_rotate_inv(q, tp_quat_rotate(q, v));
        TP_CHECK_V3_NEAR(r, v.x, v.y, v.z, eps);
    }

    TP_TEST("quaternion composition matches sequential rotation") {
        tp_quat a = tp_quat_from_axis_angle(tp_v3(0, 1, 0), 0.4f);
        tp_quat b = tp_quat_from_axis_angle(tp_v3(1, 0, 0), 0.9f);
        tp_vec3 v = tp_v3(1, 2, 3);
        tp_vec3 sequential = tp_quat_rotate(a, tp_quat_rotate(b, v));
        tp_vec3 composed   = tp_quat_rotate(tp_quat_mul(a, b), v);
        TP_CHECK_V3_NEAR(composed, sequential.x, sequential.y, sequential.z, eps);
    }

    TP_TEST("mat3_from_quat agrees with quat_rotate") {
        tp_quat q = tp_quat_from_axis_angle(tp_v3(-2, 5, 1), 1.3f);
        tp_mat3 m = tp_mat3_from_quat(q);
        tp_vec3 v = tp_v3(0.5f, -3.0f, 2.25f);
        tp_vec3 by_quat = tp_quat_rotate(q, v);
        tp_vec3 by_mat  = tp_mat3_mul_v3(m, v);
        TP_CHECK_V3_NEAR(by_mat, by_quat.x, by_quat.y, by_quat.z, eps);
        /* A rotation matrix has determinant +1. If this fails, a sign in
         * mat3_from_quat is wrong and inertia tensors will be garbage. */
        TP_CHECK_NEAR(tp_mat3_determinant(m), 1.0f, eps);
    }

    TP_TEST("mat3 inverse times original is the identity") {
        tp_mat3 m = tp_mat3_cols(tp_v3(2, 1, 0), tp_v3(0, 3, 1), tp_v3(1, 0, 4));
        tp_mat3 p = tp_mat3_mul(m, tp_mat3_inverse(m));
        TP_CHECK_V3_NEAR(p.c0, 1.0f, 0.0f, 0.0f, eps);
        TP_CHECK_V3_NEAR(p.c1, 0.0f, 1.0f, 0.0f, eps);
        TP_CHECK_V3_NEAR(p.c2, 0.0f, 0.0f, 1.0f, eps);
    }

    TP_TEST("singular matrix inverts to zero instead of NaN") {
        tp_mat3 m = tp_mat3_cols(tp_v3(1, 0, 0), tp_v3(2, 0, 0), tp_v3(3, 0, 0));
        tp_mat3 inv = tp_mat3_inverse(m);
        TP_CHECK_V3_NEAR(inv.c0, 0.0f, 0.0f, 0.0f, eps);
    }

    TP_TEST("integrating angular velocity for a full turn returns to start") {
        tp_quat q = tp_quat_identity();
        tp_vec3 w = tp_v3(0, TP_PI * 2.0f, 0);   /* one revolution per second */
        for (int i = 0; i < 2000; ++i) q = tp_quat_integrate(q, w, 1.0f / 2000.0f);
        tp_vec3 v = tp_quat_rotate(q, tp_v3(1, 0, 0));
        TP_CHECK_V3_NEAR(v, 1.0f, 0.0f, 0.0f, 1e-3f);
    }

    TP_TEST("transform round-trips a world point through local space") {
        tp_transform t = tp_transform_make(
            tp_v3(3, -2, 7), tp_quat_from_axis_angle(tp_v3(0, 0, 1), 1.1f));
        tp_vec3 p = tp_v3(1, 1, 1);
        tp_vec3 r = tp_transform_point(t, tp_transform_point_inv(t, p));
        TP_CHECK_V3_NEAR(r, p.x, p.y, p.z, eps);
    }

    TP_TEST("transform_inverse undoes transform_combine") {
        tp_transform t = tp_transform_make(
            tp_v3(1, 2, 3), tp_quat_from_axis_angle(tp_v3(1, 1, 0), 0.6f));
        tp_transform id = tp_transform_combine(t, tp_transform_inverse(t));
        TP_CHECK_V3_NEAR(id.position, 0.0f, 0.0f, 0.0f, eps);
        TP_CHECK_NEAR(fabsf(id.rotation.w), 1.0f, eps);
    }

    return tp_test_summary();
}
