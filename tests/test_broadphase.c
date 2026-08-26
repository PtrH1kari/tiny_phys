#include "tp_test.h"
#include "tphys/tphys.h"

static tp_world* make_world(void) {
    tp_world_desc wd = tp_world_desc_default();
    wd.gravity = tp_v3_zero();
    return tp_world_create(&wd);
}

int main(void) {
    TP_TEST("two bodies 10 m apart produce no pairs") {
        tp_world* w = make_world();

        tp_body_desc bd = tp_body_desc_default();
        bd.transform.position = tp_v3(0.0f, 0.0f, 0.0f);
        tp_body_create(w, &bd);

        bd.transform.position = tp_v3(10.0f, 0.0f, 0.0f);
        tp_body_create(w, &bd);
        
        tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK(tp_world_get_pair_count(w) == 0);

        tp_world_destroy(w);
    }

    TP_TEST("two bodies in one point produce 1 pair") {
        tp_world* w = make_world();

        tp_body_desc bd = tp_body_desc_default();
        bd.transform.position = tp_v3(0.0f, 0.0f, 0.0f);
        tp_body_create(w, &bd);
        tp_body_create(w, &bd);
        
        tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK(tp_world_get_pair_count(w) == 1);

        tp_world_destroy(w);
    }

    TP_TEST("three bodies in one point produce 3 pairs") {
        tp_world* w = make_world();

        tp_body_desc bd = tp_body_desc_default();
        bd.transform.position = tp_v3(0.0f, 0.0f, 0.0f);
        tp_body_create(w, &bd);
        tp_body_create(w, &bd);
        tp_body_create(w, &bd);
        
        tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK(tp_world_get_pair_count(w) == 3);

        tp_world_destroy(w);
    }

    TP_TEST("two STATIC bodies in one point produce no pairs") {
        tp_world* w = make_world();

        tp_body_desc bd = tp_body_desc_default();
        bd.type = TP_BODY_STATIC;

        bd.transform.position = tp_v3(0.0f, 0.0f, 0.0f);
        tp_body_create(w, &bd);
        tp_body_create(w, &bd);
        
        tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK(tp_world_get_pair_count(w) == 0);

        tp_world_destroy(w);
    }

    TP_TEST("STATIC and DYNAMIC bodies in one point produce 1 pair") {
        tp_world* w = make_world();

        tp_body_desc bd = tp_body_desc_default();
        bd.type = TP_BODY_STATIC;
        bd.transform.position = tp_v3(0.0f, 0.0f, 0.0f);
        tp_body_create(w, &bd);
        
        bd.type = TP_BODY_DYNAMIC;
        tp_body_create(w, &bd);
        
        tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK(tp_world_get_pair_count(w) == 1);

        tp_world_destroy(w);
    }

    TP_TEST("one body at (0,0,0) produce no pairs") {
        tp_world* w = make_world();

        tp_body_desc bd = tp_body_desc_default();
        bd.transform.position = tp_v3(0.0f, 0.0f, 0.0f);
        tp_body_create(w, &bd);
        
        tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK(tp_world_get_pair_count(w) == 0);

        tp_world_destroy(w);
    }

    TP_TEST("one destroyed body produce no pairs") {
        tp_world* w = make_world();

        tp_body_desc bd = tp_body_desc_default();
        bd.transform.position = tp_v3(0.0f, 0.0f, 0.0f);
        tp_body_id id = tp_body_create(w, &bd);
        tp_body_destroy(w, id);

        tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK(tp_world_get_pair_count(w) == 0);

        tp_world_destroy(w);
    }

    TP_TEST("50 bodies at the same point produce 1225 pairs") {
        tp_world* w = make_world();

        tp_body_desc bd = tp_body_desc_default();
        for (int i = 0; i < 50; ++i) tp_body_create(w, &bd);

        tp_world_step(w, 1.0f / 60.0f);
        TP_CHECK(tp_world_get_pair_count(w) == 1225);

        tp_world_destroy(w);
    }

    return tp_test_summary();
}
