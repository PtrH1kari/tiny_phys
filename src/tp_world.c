#include "tp_internal.h"
#include "tphys/tphys.h"
#include <stdint.h>
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

static void tp_body_pool_init(tp_body* bodies, uint32_t from, uint32_t to) {
    for (uint32_t i = from; i < to; i++) {
        memset(&bodies[i], 0, sizeof bodies[i]);
        bodies[i].generation = 1;
        bodies[i].in_use = false;
        bodies[i].next_free = (i + 1 < to) ? (i + 1) : TP_INVALID_INDEX;
    }
}

static bool tp_body_pool_grow(tp_world* world) {
    uint32_t new_capacity = (world->body_capacity != 0) ? (world->body_capacity * 2) : 64;
    tp_body* new_bodies = tp_alloc(world, new_capacity * sizeof(tp_body));
    if (new_bodies == NULL) return false;
    uint32_t old_body_capacity =  world->body_capacity;
    memcpy(new_bodies, world->bodies, world->body_capacity * sizeof(tp_body));
    tp_body_pool_init(new_bodies, world->body_capacity, new_capacity);
    tp_free(world, world->bodies);
    world->bodies = new_bodies;
    world->body_capacity = new_capacity;
    world->body_free_head = old_body_capacity;
    return true;
}

tp_body* tp_body_lookup(tp_world* world, tp_body_id id) {
    if (world == NULL || id.index >= world->body_capacity) return NULL;
    tp_body* b = &world->bodies[id.index];
    if (!b->in_use) return NULL;
    if (b->generation != id.generation) return NULL;
    return b;
}

const tp_body* tp_body_lookup_const(const tp_world* world, tp_body_id id)  {
    return tp_body_lookup((tp_world*)world, id);
}

void tp_body_update_inertia_world(tp_body *body) {
    if (body->lock_rotation || body->type != TP_BODY_DYNAMIC) {
        body->inv_inertia_world = tp_mat3_zero();
    } else {
        tp_mat3 r = tp_mat3_from_quat(body->xform.rotation);
        body->inv_inertia_world = tp_mat3_mul(tp_mat3_mul(r, body->inv_inertia_local), tp_mat3_transpose(r));
    }
}

tp_world* tp_world_create(const tp_world_desc* desc) {
    tp_world_desc d = desc ? *desc : tp_world_desc_default();
    tp_allocator alloc = d.allocator;
    if (alloc.alloc_fn == NULL || alloc.free_fn == NULL) {
        alloc.alloc_fn = tp_default_alloc;
        alloc.free_fn = tp_default_free;
        alloc.ctx = NULL;
    }
    tp_world* world = alloc.alloc_fn(sizeof(tp_world), alloc.ctx);
    if (world == NULL) return NULL;
    memset(world, 0, sizeof *world);
    world->alloc = alloc;
    world->gravity = d.gravity;
    world->velocity_iterations = d.velocity_iterations ? d.velocity_iterations : 8;
    world->position_iterations = d.position_iterations ? d.position_iterations : 3;
    uint32_t capacity = d.max_bodies ? d.max_bodies : 256;
    world->bodies = tp_alloc(world, capacity * sizeof(tp_body));
    if (world->bodies == NULL) {
        alloc.free_fn(world, alloc.ctx);
        return NULL;
    }
    tp_body_pool_init(world->bodies, 0, capacity);
    world->body_capacity = capacity;
    world->body_free_head = 0;
    world->body_count = 0;
    return world;
}

void tp_world_destroy(tp_world *world) {
    if (world == NULL) return;
    tp_allocator alloc = world->alloc;
    tp_free(world, world->bodies);
    alloc.free_fn(world, alloc.ctx);
}

const char* tp_version_string(void) {
    return "tiny_phys 0.1.0-dev";
}
