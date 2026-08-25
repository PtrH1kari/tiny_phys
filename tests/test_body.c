#include "tp_test.h"
#include "tphys/tphys.h"

int main(void) {
    TP_TEST("world creates and destroys cleanly") {
        tp_world_desc wd = tp_world_desc_default();
        tp_world* w = tp_world_create(&wd);
        TP_CHECK(w != NULL);
        TP_CHECK_NEAR(tp_world_get_gravity(w).y, -9.81f, 1e-6f);
        TP_CHECK(tp_world_get_body_count(w) == 0);
        tp_world_destroy(w);
    }

    TP_TEST("a destroyed handle stops being valid, even after slot reuse") {
        tp_world_desc wd = tp_world_desc_default();
        tp_world* w = tp_world_create(&wd);
        tp_body_desc bd = tp_body_desc_default();

        tp_body_id a = tp_body_create(w, &bd);
        TP_CHECK(tp_body_is_valid(w, a));
        tp_body_destroy(w, a);
        TP_CHECK(!tp_body_is_valid(w, a));

        /* The next create reuses the same slot -- the old handle must still
         * be rejected. This is the whole point of the generation counter. */
        tp_body_id b = tp_body_create(w, &bd);
        TP_CHECK(b.index == a.index);
        TP_CHECK(!tp_body_id_equal(a, b));
        TP_CHECK(!tp_body_is_valid(w, a));
        TP_CHECK(tp_body_is_valid(w, b));

        tp_world_destroy(w);
    }

    TP_TEST("the pool grows past its initial capacity") {
        tp_world_desc wd = tp_world_desc_default();
        wd.max_bodies = 4;
        tp_world* w = tp_world_create(&wd);
        tp_body_desc bd = tp_body_desc_default();
        for (int i = 0; i < 100; ++i) TP_CHECK(tp_body_is_valid(w, tp_body_create(w, &bd)));
        TP_CHECK(tp_world_get_body_count(w) == 100);
        tp_world_destroy(w);
    }

    TP_TEST("free fall matches the analytic solution") {
        tp_world_desc wd = tp_world_desc_default();
        tp_world* w = tp_world_create(&wd);

        tp_body_desc bd = tp_body_desc_default();
        bd.linear_damping = 0.0f;          /* compare against pure gravity */
        tp_body_id body = tp_body_create(w, &bd);

        const float dt = 1.0f / 240.0f;
        const int steps = 240;             /* one second */
        for (int i = 0; i < steps; ++i) tp_world_step(w, dt);

        /* Semi-implicit Euler overshoots the exact solution by g*dt*t/2,
         * which is ~0.02 m here. Anything larger means a real bug. */
        float y = tp_body_get_position(w, body).y;
        TP_CHECK_NEAR(y, -0.5f * 9.81f, 0.05f);
        TP_CHECK_NEAR(tp_body_get_linear_velocity(w, body).y, -9.81f, 1e-3f);

        tp_world_destroy(w);
    }

    TP_TEST("static bodies ignore gravity and forces") {
        tp_world_desc wd = tp_world_desc_default();
        tp_world* w = tp_world_create(&wd);
        tp_body_desc bd = tp_body_desc_default();
        bd.type = TP_BODY_STATIC;
        tp_body_id body = tp_body_create(w, &bd);

        tp_body_apply_impulse(w, body, tp_v3(0, 1000, 0));
        for (int i = 0; i < 60; ++i) tp_world_step(w, 1.0f / 60.0f);

        TP_CHECK_V3_NEAR(tp_body_get_position(w, body), 0.0f, 0.0f, 0.0f, 1e-6f);
        TP_CHECK_NEAR(tp_body_get_mass(w, body), 0.0f, 1e-6f);
        tp_world_destroy(w);
    }

    TP_TEST("an impulse produces v = J / m") {
        tp_world_desc wd = tp_world_desc_default();
        wd.gravity = tp_v3_zero();
        tp_world* w = tp_world_create(&wd);

        tp_body_desc bd = tp_body_desc_default();
        bd.linear_damping = 0.0f;
        tp_body_id body = tp_body_create(w, &bd);
        tp_body_set_mass(w, body, 2.0f);

        tp_body_apply_impulse(w, body, tp_v3(10, 0, 0));
        TP_CHECK_NEAR(tp_body_get_linear_velocity(w, body).x, 5.0f, 1e-5f);

        tp_world_destroy(w);
    }

    TP_TEST("angular momentum is conserved with no torque") {
        tp_world_desc wd = tp_world_desc_default();
        wd.gravity = tp_v3_zero();
        tp_world* w = tp_world_create(&wd);

        tp_body_desc bd = tp_body_desc_default();
        bd.angular_damping = 0.0f;
        bd.angular_velocity = tp_v3(0, 3.0f, 0);
        tp_body_id body = tp_body_create(w, &bd);

        for (int i = 0; i < 600; ++i) tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK_V3_NEAR(tp_body_get_angular_velocity(w, body), 0.0f, 3.0f, 0.0f, 1e-4f);

        /* The orientation must stay unit-length after 600 integrations. */
        tp_quat q = tp_body_get_rotation(w, body);
        TP_CHECK_NEAR(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0f, 1e-5f);

        tp_world_destroy(w);
    }

    TP_TEST("calls on a stale handle are ignored rather than crashing") {
        tp_world_desc wd = tp_world_desc_default();
        tp_world* w = tp_world_create(&wd);
        tp_body_desc bd = tp_body_desc_default();
        tp_body_id body = tp_body_create(w, &bd);
        tp_body_destroy(w, body);

        tp_body_set_linear_velocity(w, body, tp_v3(1, 2, 3));
        tp_body_apply_force(w, body, tp_v3(1, 2, 3));
        TP_CHECK_V3_NEAR(tp_body_get_linear_velocity(w, body), 0.0f, 0.0f, 0.0f, 1e-6f);
        TP_CHECK(tp_body_get_user_data(w, body) == NULL);

        tp_world_destroy(w);
    }

    TP_TEST("a body comes to rest and falls asleep") {
        tp_world_desc wd = tp_world_desc_default();
        wd.gravity = tp_v3_zero();
        tp_world* w = tp_world_create(&wd);
        tp_body_desc bd = tp_body_desc_default();
        tp_body_id body = tp_body_create(w, &bd);

        TP_CHECK(tp_body_is_awake(w, body));
        for (int i = 0; i < 120; ++i) tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK(!tp_body_is_awake(w, body));

        tp_body_wake(w, body);
        TP_CHECK(tp_body_is_awake(w, body));
        tp_world_destroy(w);
    }

    return tp_test_summary();
}
