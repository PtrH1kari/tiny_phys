/*
 * tp_internal.h -- private to the library. Never installed, never included
 * by users. Change anything in here freely; the public ABI does not depend
 * on it because tp_world is opaque in tphys.h.
 */
#ifndef TP_INTERNAL_H
#define TP_INTERNAL_H

#include "tphys/tphys.h"

#define TP_INVALID_INDEX  0xFFFFFFFFu
#define TP_MAX_SHAPES_PER_BODY 8
#define TP_SLEEP_LINEAR_THRESHOLD   0.01f   /* m/s   */
#define TP_SLEEP_ANGULAR_THRESHOLD  0.05f   /* rad/s */
#define TP_SLEEP_TIME               0.5f    /* s     */

typedef struct tp_shape {
    tp_shape_desc desc;
    tp_body_id    body;
    uint32_t      generation;
    bool          in_use;
    uint32_t      next_free;
} tp_shape;

typedef struct tp_body {
    /* --- state the solver reads and writes every step ------------------- */
    tp_transform xform;
    tp_vec3      linear_velocity;
    tp_vec3      angular_velocity;
    tp_vec3      force_accum;
    tp_vec3      torque_accum;

    /* --- mass properties ------------------------------------------------ */
    float   inv_mass;            /* 0 means infinite mass (static)          */
    tp_mat3 inv_inertia_local;   /* in body space, constant                 */
    tp_mat3 inv_inertia_world;   /* recomputed each step from orientation   */

    /* --- configuration -------------------------------------------------- */
    tp_body_type type;
    float        linear_damping;
    float        angular_damping;
    float        gravity_scale;
    bool         lock_rotation;
    void*        user_data;

    /* --- shapes --------------------------------------------------------- */
    uint32_t shapes[TP_MAX_SHAPES_PER_BODY];
    uint32_t shape_count;

    /* --- sleeping ------------------------------------------------------- */
    bool  awake;
    float sleep_timer;

    /* --- slot bookkeeping ----------------------------------------------- */
    uint32_t generation;
    bool     in_use;
    uint32_t next_free;
} tp_body;

struct tp_world {
    tp_vec3  gravity;
    uint32_t velocity_iterations;
    uint32_t position_iterations;

    tp_body* bodies;
    uint32_t body_capacity;
    uint32_t body_count;
    uint32_t body_free_head;

    tp_shape* shapes;
    uint32_t  shape_capacity;
    uint32_t  shape_count;
    uint32_t  shape_free_head;

    tp_contact_event* contact_events;
    uint32_t          contact_event_count;
    uint32_t          contact_event_capacity;

    tp_allocator alloc;
};

/* Allocation goes through the world so a custom allocator is always honoured. */
void* tp_alloc(tp_world* world, size_t size);
void  tp_free(tp_world* world, void* ptr);

/* Returns NULL for a stale or out-of-range handle. Callers MUST check. */
tp_body*       tp_body_lookup(tp_world* world, tp_body_id id);
const tp_body* tp_body_lookup_const(const tp_world* world, tp_body_id id);

/* Recompute inv_inertia_world = R * inv_inertia_local * R^T. */
void tp_body_update_inertia_world(tp_body* body);

#endif /* TP_INTERNAL_H */
