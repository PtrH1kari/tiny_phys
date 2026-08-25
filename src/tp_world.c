#include "tp_internal.h"

#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------- allocator */

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
    if (ptr) world->alloc.free_fn(ptr, world->alloc.ctx);
}

/* ------------------------------------------------------------- defaults */

tp_body_id tp_body_id_null(void) {
    tp_body_id id; id.index = TP_INVALID_INDEX; id.generation = 0; return id;
}
tp_shape_id tp_shape_id_null(void) {
    tp_shape_id id; id.index = TP_INVALID_INDEX; id.generation = 0; return id;
}
bool tp_body_id_equal(tp_body_id a, tp_body_id b) {
    return a.index == b.index && a.generation == b.generation;
}

tp_material tp_material_default(void) {
    tp_material m; m.friction = 0.5f; m.restitution = 0.0f; return m;
}

tp_shape_desc tp_shape_desc_default(void) {
    tp_shape_desc d;
    memset(&d, 0, sizeof d);
    d.type            = TP_SHAPE_SPHERE;
    d.data.sphere.radius = 0.5f;
    d.local           = tp_transform_identity();
    d.density         = 1000.0f;          /* roughly water / soft plastic */
    d.material        = tp_material_default();
    d.layer           = 1u;
    d.mask            = 0xFFFFFFFFu;
    d.is_sensor       = false;
    return d;
}

tp_body_desc tp_body_desc_default(void) {
    tp_body_desc d;
    memset(&d, 0, sizeof d);
    d.type             = TP_BODY_DYNAMIC;
    d.transform        = tp_transform_identity();
    d.linear_velocity  = tp_v3_zero();
    d.angular_velocity = tp_v3_zero();
    d.linear_damping   = 0.05f;
    d.angular_damping  = 0.05f;
    d.gravity_scale    = 1.0f;
    d.lock_rotation    = false;
    d.start_asleep     = false;
    d.user_data        = NULL;
    return d;
}

tp_world_desc tp_world_desc_default(void) {
    tp_world_desc d;
    memset(&d, 0, sizeof d);
    d.gravity             = tp_v3(0.0f, -9.81f, 0.0f);
    d.max_bodies          = 256;
    d.velocity_iterations = 8;
    d.position_iterations = 3;
    return d;
}

tp_query_filter tp_query_filter_default(void) {
    tp_query_filter f;
    f.layer_mask  = 0xFFFFFFFFu;
    f.hit_sensors = false;
    f.hit_static  = true;
    f.hit_dynamic = true;
    return f;
}

/* ------------------------------------------------------------ body pool */

/*
 * Slots are recycled through an intrusive free list. `generation` starts at 1
 * and increments on every destroy, so a handle held across a destroy/create
 * pair no longer matches and tp_body_lookup returns NULL.
 */
static void tp_body_pool_init(tp_body* bodies, uint32_t from, uint32_t to) {
    for (uint32_t i = from; i < to; ++i) {
        memset(&bodies[i], 0, sizeof bodies[i]);
        bodies[i].generation = 1;
        bodies[i].in_use     = false;
        bodies[i].next_free  = (i + 1 < to) ? (i + 1) : TP_INVALID_INDEX;
    }
}

static bool tp_body_pool_grow(tp_world* world) {
    uint32_t new_capacity = world->body_capacity ? world->body_capacity * 2 : 64;
    tp_body* new_bodies = (tp_body*)tp_alloc(world, new_capacity * sizeof(tp_body));
    if (!new_bodies) return false;

    memcpy(new_bodies, world->bodies, world->body_capacity * sizeof(tp_body));
    tp_body_pool_init(new_bodies, world->body_capacity, new_capacity);

    tp_free(world, world->bodies);
    world->bodies         = new_bodies;
    world->body_free_head = world->body_capacity;
    world->body_capacity  = new_capacity;
    return true;
}

tp_body* tp_body_lookup(tp_world* world, tp_body_id id) {
    if (!world || id.index >= world->body_capacity) return NULL;
    tp_body* b = &world->bodies[id.index];
    if (!b->in_use || b->generation != id.generation) return NULL;
    return b;
}
const tp_body* tp_body_lookup_const(const tp_world* world, tp_body_id id) {
    return tp_body_lookup((tp_world*)world, id);
}

/* --------------------------------------------------------------- world */

tp_world* tp_world_create(const tp_world_desc* desc) {
    tp_world_desc d = desc ? *desc : tp_world_desc_default();

    tp_allocator alloc = d.allocator;
    if (!alloc.alloc_fn || !alloc.free_fn) {
        alloc.alloc_fn = tp_default_alloc;
        alloc.free_fn  = tp_default_free;
        alloc.ctx      = NULL;
    }

    tp_world* world = (tp_world*)alloc.alloc_fn(sizeof(tp_world), alloc.ctx);
    if (!world) return NULL;
    memset(world, 0, sizeof *world);
    world->alloc = alloc;

    world->gravity             = d.gravity;
    world->velocity_iterations = d.velocity_iterations ? d.velocity_iterations : 8;
    world->position_iterations = d.position_iterations ? d.position_iterations : 3;

    uint32_t capacity = d.max_bodies ? d.max_bodies : 256;
    world->bodies = (tp_body*)tp_alloc(world, capacity * sizeof(tp_body));
    if (!world->bodies) {
        alloc.free_fn(world, alloc.ctx);
        return NULL;
    }
    tp_body_pool_init(world->bodies, 0, capacity);
    world->body_capacity  = capacity;
    world->body_free_head = 0;
    world->body_count     = 0;

    return world;
}

void tp_world_destroy(tp_world* world) {
    if (!world) return;
    tp_allocator alloc = world->alloc;
    tp_free(world, world->bodies);
    tp_free(world, world->shapes);
    tp_free(world, world->contact_events);
    alloc.free_fn(world, alloc.ctx);
}

void tp_world_set_gravity(tp_world* world, tp_vec3 gravity) {
    if (world) world->gravity = gravity;
}
tp_vec3 tp_world_get_gravity(const tp_world* world) {
    return world ? world->gravity : tp_v3_zero();
}
uint32_t tp_world_get_body_count(const tp_world* world) {
    return world ? world->body_count : 0u;
}

/* ---------------------------------------------------------------- body */

tp_body_id tp_body_create(tp_world* world, const tp_body_desc* desc) {
    if (!world) return tp_body_id_null();
    if (world->body_free_head == TP_INVALID_INDEX && !tp_body_pool_grow(world))
        return tp_body_id_null();

    uint32_t index = world->body_free_head;
    tp_body* b = &world->bodies[index];
    world->body_free_head = b->next_free;

    tp_body_desc d = desc ? *desc : tp_body_desc_default();

    uint32_t generation = b->generation;
    memset(b, 0, sizeof *b);
    b->generation = generation;
    b->in_use     = true;
    b->next_free  = TP_INVALID_INDEX;

    b->xform            = d.transform;
    b->linear_velocity  = d.linear_velocity;
    b->angular_velocity = d.angular_velocity;
    b->type             = d.type;
    b->linear_damping   = d.linear_damping;
    b->angular_damping  = d.angular_damping;
    b->gravity_scale    = d.gravity_scale;
    b->lock_rotation    = d.lock_rotation;
    b->user_data        = d.user_data;
    b->awake            = !d.start_asleep;
    b->sleep_timer      = 0.0f;

    if (d.type == TP_BODY_DYNAMIC) {
        /* Provisional unit mass. Replaced once shapes are added (M4) or by
         * an explicit tp_body_set_mass call. */
        b->inv_mass          = 1.0f;
        b->inv_inertia_local = tp_mat3_identity();
    } else {
        b->inv_mass          = 0.0f;
        b->inv_inertia_local = tp_mat3_zero();
    }
    tp_body_update_inertia_world(b);

    world->body_count++;

    tp_body_id id;
    id.index      = index;
    id.generation = generation;
    return id;
}

void tp_body_destroy(tp_world* world, tp_body_id id) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b) return;

    b->in_use    = false;
    b->generation++;                    /* invalidates every existing handle */
    if (b->generation == 0) b->generation = 1;   /* keep 0 reserved as null  */
    b->next_free = world->body_free_head;
    world->body_free_head = id.index;
    world->body_count--;
}

bool tp_body_is_valid(const tp_world* world, tp_body_id id) {
    return tp_body_lookup_const(world, id) != NULL;
}

tp_shape_id tp_body_add_shape(tp_world* world, tp_body_id body,
                              const tp_shape_desc* desc) {
    (void)world; (void)body; (void)desc;
    /* TODO(M4): allocate from the shape pool, derive mass properties from
     * density and volume, then rebuild the body's inverse inertia tensor. */
    return tp_shape_id_null();
}

void tp_body_update_inertia_world(tp_body* body) {
    if (body->lock_rotation || body->type != TP_BODY_DYNAMIC) {
        body->inv_inertia_world = tp_mat3_zero();
        return;
    }
    tp_mat3 r = tp_mat3_from_quat(body->xform.rotation);
    body->inv_inertia_world =
        tp_mat3_mul(tp_mat3_mul(r, body->inv_inertia_local), tp_mat3_transpose(r));
}

/* --------------------------------------------------------- body access */

#define TP_GET_OR(world, id, field, fallback)                       \
    do {                                                            \
        const tp_body* b_ = tp_body_lookup_const((world), (id));    \
        return b_ ? b_->field : (fallback);                         \
    } while (0)

tp_transform tp_body_get_transform(const tp_world* world, tp_body_id id) {
    TP_GET_OR(world, id, xform, tp_transform_identity());
}
tp_vec3 tp_body_get_position(const tp_world* world, tp_body_id id) {
    return tp_body_get_transform(world, id).position;
}
tp_quat tp_body_get_rotation(const tp_world* world, tp_body_id id) {
    return tp_body_get_transform(world, id).rotation;
}
tp_vec3 tp_body_get_linear_velocity(const tp_world* world, tp_body_id id) {
    TP_GET_OR(world, id, linear_velocity, tp_v3_zero());
}
tp_vec3 tp_body_get_angular_velocity(const tp_world* world, tp_body_id id) {
    TP_GET_OR(world, id, angular_velocity, tp_v3_zero());
}
void* tp_body_get_user_data(const tp_world* world, tp_body_id id) {
    TP_GET_OR(world, id, user_data, NULL);
}
bool tp_body_is_awake(const tp_world* world, tp_body_id id) {
    TP_GET_OR(world, id, awake, false);
}

void tp_body_set_transform(tp_world* world, tp_body_id id, tp_transform xform) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b) return;
    b->xform = xform;
    b->xform.rotation = tp_quat_normalize(b->xform.rotation);
    tp_body_update_inertia_world(b);
    b->awake = true;
    b->sleep_timer = 0.0f;
}
void tp_body_set_linear_velocity(tp_world* world, tp_body_id id, tp_vec3 v) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b || b->type == TP_BODY_STATIC) return;
    b->linear_velocity = v;
    b->awake = true; b->sleep_timer = 0.0f;
}
void tp_body_set_angular_velocity(tp_world* world, tp_body_id id, tp_vec3 w) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b || b->type == TP_BODY_STATIC) return;
    b->angular_velocity = w;
    b->awake = true; b->sleep_timer = 0.0f;
}
void tp_body_set_user_data(tp_world* world, tp_body_id id, void* user_data) {
    tp_body* b = tp_body_lookup(world, id);
    if (b) b->user_data = user_data;
}
void tp_body_wake(tp_world* world, tp_body_id id) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b) return;
    b->awake = true;
    b->sleep_timer = 0.0f;
}

float tp_body_get_mass(const tp_world* world, tp_body_id id) {
    const tp_body* b = tp_body_lookup_const(world, id);
    if (!b || b->inv_mass <= 0.0f) return 0.0f;   /* 0 reported as infinite */
    return 1.0f / b->inv_mass;
}
void tp_body_set_mass(tp_world* world, tp_body_id id, float mass) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b || b->type != TP_BODY_DYNAMIC) return;
    b->inv_mass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
}

/* --------------------------------------------------------- body forces */

void tp_body_apply_force(tp_world* world, tp_body_id id, tp_vec3 force) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b || b->type != TP_BODY_DYNAMIC) return;
    b->force_accum = tp_v3_add(b->force_accum, force);
    b->awake = true; b->sleep_timer = 0.0f;
}
void tp_body_apply_torque(tp_world* world, tp_body_id id, tp_vec3 torque) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b || b->type != TP_BODY_DYNAMIC) return;
    b->torque_accum = tp_v3_add(b->torque_accum, torque);
    b->awake = true; b->sleep_timer = 0.0f;
}
void tp_body_apply_force_at(tp_world* world, tp_body_id id,
                            tp_vec3 force, tp_vec3 world_point) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b || b->type != TP_BODY_DYNAMIC) return;
    tp_vec3 r = tp_v3_sub(world_point, b->xform.position);
    b->force_accum  = tp_v3_add(b->force_accum, force);
    b->torque_accum = tp_v3_add(b->torque_accum, tp_v3_cross(r, force));
    b->awake = true; b->sleep_timer = 0.0f;
}
void tp_body_apply_impulse(tp_world* world, tp_body_id id, tp_vec3 impulse) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b || b->type != TP_BODY_DYNAMIC) return;
    b->linear_velocity = tp_v3_mad(b->linear_velocity, impulse, b->inv_mass);
    b->awake = true; b->sleep_timer = 0.0f;
}
void tp_body_apply_impulse_at(tp_world* world, tp_body_id id,
                              tp_vec3 impulse, tp_vec3 world_point) {
    tp_body* b = tp_body_lookup(world, id);
    if (!b || b->type != TP_BODY_DYNAMIC) return;
    tp_vec3 r = tp_v3_sub(world_point, b->xform.position);
    b->linear_velocity = tp_v3_mad(b->linear_velocity, impulse, b->inv_mass);
    b->angular_velocity = tp_v3_add(
        b->angular_velocity,
        tp_mat3_mul_v3(b->inv_inertia_world, tp_v3_cross(r, impulse)));
    b->awake = true; b->sleep_timer = 0.0f;
}

/* ---------------------------------------------------------------- step */

/*
 * Semi-implicit (symplectic) Euler: integrate velocity first, then use the
 * NEW velocity to integrate position. One line different from explicit Euler,
 * and the difference between a stable simulation and one that gains energy
 * until it explodes.
 */
static void tp_integrate_velocities(tp_world* world, float dt) {
    for (uint32_t i = 0; i < world->body_capacity; ++i) {
        tp_body* b = &world->bodies[i];
        if (!b->in_use || b->type != TP_BODY_DYNAMIC || !b->awake) continue;

        tp_vec3 accel = tp_v3_scale(world->gravity, b->gravity_scale);
        accel = tp_v3_mad(accel, b->force_accum, b->inv_mass);
        b->linear_velocity = tp_v3_mad(b->linear_velocity, accel, dt);

        if (!b->lock_rotation) {
            tp_vec3 angular_accel =
                tp_mat3_mul_v3(b->inv_inertia_world, b->torque_accum);
            b->angular_velocity =
                tp_v3_mad(b->angular_velocity, angular_accel, dt);
        }

        /* Exponential damping: frame-rate independent, unlike v *= 0.99f. */
        float lin_factor = 1.0f / (1.0f + dt * b->linear_damping);
        float ang_factor = 1.0f / (1.0f + dt * b->angular_damping);
        b->linear_velocity  = tp_v3_scale(b->linear_velocity,  lin_factor);
        b->angular_velocity = tp_v3_scale(b->angular_velocity, ang_factor);
    }
}

static void tp_integrate_positions(tp_world* world, float dt) {
    for (uint32_t i = 0; i < world->body_capacity; ++i) {
        tp_body* b = &world->bodies[i];
        if (!b->in_use || b->type == TP_BODY_STATIC || !b->awake) continue;

        b->xform.position = tp_v3_mad(b->xform.position, b->linear_velocity, dt);
        if (!b->lock_rotation) {
            b->xform.rotation =
                tp_quat_integrate(b->xform.rotation, b->angular_velocity, dt);
        }
        tp_body_update_inertia_world(b);
    }
}

static void tp_update_sleeping(tp_world* world, float dt) {
    for (uint32_t i = 0; i < world->body_capacity; ++i) {
        tp_body* b = &world->bodies[i];
        if (!b->in_use || b->type != TP_BODY_DYNAMIC || !b->awake) continue;

        bool slow =
            tp_v3_length_sq(b->linear_velocity) <
                TP_SLEEP_LINEAR_THRESHOLD * TP_SLEEP_LINEAR_THRESHOLD &&
            tp_v3_length_sq(b->angular_velocity) <
                TP_SLEEP_ANGULAR_THRESHOLD * TP_SLEEP_ANGULAR_THRESHOLD;

        if (slow) {
            b->sleep_timer += dt;
            if (b->sleep_timer >= TP_SLEEP_TIME) {
                b->awake = false;
                b->linear_velocity  = tp_v3_zero();
                b->angular_velocity = tp_v3_zero();
            }
        } else {
            b->sleep_timer = 0.0f;
        }
    }
}

static void tp_clear_forces(tp_world* world) {
    for (uint32_t i = 0; i < world->body_capacity; ++i) {
        tp_body* b = &world->bodies[i];
        if (!b->in_use) continue;
        b->force_accum  = tp_v3_zero();
        b->torque_accum = tp_v3_zero();
    }
}

void tp_world_step(tp_world* world, float dt) {
    if (!world || dt <= 0.0f) return;

    world->contact_event_count = 0;

    /* TODO(M3): broadphase -- build AABBs, query the dynamic tree for pairs. */
    /* TODO(M4): narrowphase -- generate contact manifolds for those pairs.   */

    tp_integrate_velocities(world, dt);

    /* TODO(M5): solve velocity constraints, `velocity_iterations` times.     */

    tp_integrate_positions(world, dt);

    /* TODO(M5): solve position constraints, `position_iterations` times.     */

    tp_update_sleeping(world, dt);
    tp_clear_forces(world);
}

/* -------------------------------------------------------------- queries */

bool tp_world_raycast(const tp_world* world,
                      tp_vec3 origin, tp_vec3 direction,
                      float max_distance, tp_query_filter filter,
                      tp_ray_hit* out_hit) {
    (void)world; (void)origin; (void)direction;
    (void)max_distance; (void)filter; (void)out_hit;
    /* TODO(M6): walk the broadphase tree, then per-shape ray tests. */
    return false;
}

const tp_contact_event* tp_world_get_contact_events(const tp_world* world,
                                                    uint32_t* out_count) {
    if (out_count) *out_count = world ? world->contact_event_count : 0u;
    return world ? world->contact_events : NULL;
}

void tp_world_debug_draw(const tp_world* world, const tp_debug_draw* draw) {
    if (!world || !draw || !draw->line_fn) return;
    /* TODO(M4): draw shape wireframes, AABBs and contact normals. */
}

/* ----------------------------------------------------------------- misc */

const char* tp_version_string(void) {
    return "tiny_phys 0.1.0-dev";
}

const char* tp_result_string(tp_result result) {
    switch (result) {
        case TP_OK:                   return "ok";
        case TP_ERR_INVALID_HANDLE:   return "invalid handle";
        case TP_ERR_OUT_OF_MEMORY:    return "out of memory";
        case TP_ERR_INVALID_ARGUMENT: return "invalid argument";
        case TP_ERR_UNSUPPORTED:      return "unsupported";
        default:                      return "unknown";
    }
}
