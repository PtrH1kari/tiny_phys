#ifndef TPHYS_INTERNAL_H
#define TPHYS_INTERNAL_H

#include "tphys/tphys.h"

#define TP_INVALID_INDEX 0xFFFFFFFFu

typedef struct tp_body {
    /* --- integrator state --- */
    tp_transform xform;
    tp_vec3 linear_velocity, angular_velocity, force_accum, torque_accum;
    /* --- mass  properties --- */
    float inv_mass;
    tp_mat3 inv_inertia_local, inv_inertia_world;
    /* ---- configuration ----- */
    tp_body_type type;
    float linear_damping, angular_damping, gravity_scale;
    bool lock_rotation;
    void* user_data;
    /* --- slot bookkeeping --- */
    uint32_t generation;
    bool in_use;
    uint32_t next_free;
    /* ------- sleeping ------- */
    bool awake;
    float sleep_timer;
} tp_body;
struct tp_world {
    tp_vec3 gravity;
    uint32_t velocity_iterations, position_iterations;
    tp_body* bodies;
    uint32_t body_capacity, body_count, body_free_head;
    tp_allocator alloc;
};

void* tp_alloc(tp_world* world, size_t size);
void tp_free(tp_world* world, void* ptr);
tp_body* tp_body_lookup(tp_world* world, tp_body_id id);
const tp_body* tp_body_lookup_const(const tp_world* world, tp_body_id id);
void tp_body_update_inertia_world(tp_body* body);

#endif // TPHYS_INTERNAL_H
