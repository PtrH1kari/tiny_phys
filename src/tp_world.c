#include "tp_internal.h"
#include <stdlib.h>
#include <string.h>


static void* tp_default_alloc(size_t size, void* ctx) {
    (void)ctx;
    return malloc(size);
}
static void tp_default_free(void* ptr, void* ctx) {
    (void)ctx;
    free(ptr);
}

void* tp_alloc(tp_world* world, size_t size) {
    return world->alloc.alloc_fn(size, world->alloc.ctx);
}
void tp_free(tp_world* world, void* ptr) {
    if (ptr != NULL) {
        world->alloc.free_fn(ptr, world->alloc.ctx);
    }
}

tp_material tp_material_default(void) {
    tp_material m;
    m.friction = 0.5f;
    m.restitution = 0.0f;
    return m;
}

tp_body_id tp_body_id_null(void) {
    tp_body_id id;
    id.index = TP_INVALID_INDEX;
    id.generation = 0;
    return id;
}

tp_shape_id tp_shape_id_null(void) {
    tp_shape_id id;
    id.index = TP_INVALID_INDEX;
    id.generation = 0;
    return id;
}

bool tp_body_id_equal(tp_body_id a, tp_body_id b) {
    return a.index == b.index && a.generation == b.generation;
}

tp_query_filter tp_query_filter_default(void) {
    tp_query_filter f;
    f.layer_mask = TP_LAYER_ALL;
    f.hit_sensors = false;
    f.hit_static = true;
    f.hit_dynamic = true;
    return f;
}

tp_body_desc tp_body_desc_default(void) {
    tp_body_desc d;
    memset(&d, 0, sizeof d);
    d.type = TP_BODY_DYNAMIC;
    d.transform = tp_transform_identity();
    d.linear_damping = 0.05f;
    d.angular_damping = 0.05f;
    d.gravity_scale  = 1.0f;
    return d;
}

tp_world_desc tp_world_desc_default(void) {
    tp_world_desc d;
    memset(&d, 0, sizeof d);
    d.gravity =  tp_v3(0.0f, -9.81f, 0.0f);
    d.max_bodies = 256;
    d.velocity_iterations = 8;
    d.position_iterations = 3;
    return d;
}

tp_shape_desc tp_shape_desc_default(void) {
    tp_shape_desc d;
    memset(&d, 0, sizeof d);
    d.type =  TP_SHAPE_SPHERE;
    d.data.sphere.radius = 0.5f;
    d.local = tp_transform_identity();
    d.density = 1000.0f;
    d.material = tp_material_default();
    d.layer = 1u;
    d.mask = TP_LAYER_ALL;
    return d;
}

const char* tp_version_string(void) {
    return "tiny_phys 0.1.0-dev";
}
