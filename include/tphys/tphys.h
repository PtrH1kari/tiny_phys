/*
 * tphys.h -- tiny_phys public API
 *
 * A small 3D rigid body physics engine in C11, designed to be linked into
 * C++ engines (see https://github.com/zen4xx/tiny_engine).
 *
 * Design rules for this header -- keep them:
 *   1. No renderer, window, or platform types appear here. Ever.
 *   2. Objects are referred to by generational handles, never raw pointers,
 *      so a stale handle is detectable instead of undefined behaviour.
 *   3. Internal structs stay opaque (`tp_world` is forward-declared only),
 *      so the ABI does not break when internals change.
 *   4. Valid C11 *and* valid C++: no compound literals, no anonymous structs
 *      outside unions, no designated initialisers.
 *
 * Units are SI throughout: metres, kilograms, seconds, radians.
 */
#ifndef TPHYS_H
#define TPHYS_H

#include <stdint.h>
#include <stddef.h>
#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "tphys_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TPHYS_VERSION_MAJOR 0
#define TPHYS_VERSION_MINOR 1
#define TPHYS_VERSION_PATCH 0


/* Export macro. Static builds need nothing; shared builds define TPHYS_SHARED. */
#if defined(_WIN32) && defined(TPHYS_SHARED)
  #if defined(TPHYS_BUILDING)
    #define TP_API __declspec(dllexport)
  #else
    #define TP_API __declspec(dllimport)
  #endif
#else
  #define TP_API
#endif

#define TP_LAYER_ALL 0xFFFFFFFFu

/* ---------------------------------------------------------------- handles */

/*
 * A generational handle. `index` is a slot in an internal array; `generation`
 * is bumped every time that slot is reused. A handle to a destroyed body
 * therefore fails tp_body_is_valid() instead of silently aliasing a new body.
 * generation == 0 is reserved as "never valid", so a zeroed struct is null.
 */
typedef struct tp_body_id {
    uint32_t index;
    uint32_t generation;
} tp_body_id;

typedef struct tp_shape_id {
    uint32_t index;
    uint32_t generation;
} tp_shape_id;

TP_API tp_body_id  tp_body_id_null(void);
TP_API tp_shape_id tp_shape_id_null(void);
TP_API bool        tp_body_id_equal(tp_body_id a, tp_body_id b);

/* ------------------------------------------------------------------ enums */

typedef enum tp_body_type {
    TP_BODY_STATIC = 0,   /* never moves; infinite mass (walls, terrain)     */
    TP_BODY_KINEMATIC,    /* moved by you; pushes dynamics, ignores forces   */
    TP_BODY_DYNAMIC       /* fully simulated                                 */
} tp_body_type;

typedef enum tp_shape_type {
    TP_SHAPE_SPHERE = 0,
    TP_SHAPE_BOX,
    TP_SHAPE_CAPSULE,     /* aligned to local Y axis                        */
    TP_SHAPE_PLANE        /* infinite half-space; static bodies only        */
} tp_shape_type;

typedef enum tp_result {
    TP_OK = 0,
    TP_ERR_INVALID_HANDLE,
    TP_ERR_OUT_OF_MEMORY,
    TP_ERR_INVALID_ARGUMENT,
    TP_ERR_UNSUPPORTED
} tp_result;

/* -------------------------------------------------------------- allocator */

/*
 * Optional custom allocator. Leave zeroed in tp_world_desc to use malloc/free.
 * `ctx` is passed back to you untouched -- use it for an arena or a pool.
 */
typedef struct tp_allocator {
    void* (*alloc_fn)(size_t size, void* ctx);
    void  (*free_fn)(void* ptr, void* ctx);
    void*  ctx;
} tp_allocator;

/* --------------------------------------------------------------- material */

typedef struct tp_material {
    float friction;     /* Coulomb coefficient, 0 = ice, 1 = rubber. Default 0.5 */
    float restitution;  /* bounciness, 0 = clay, 1 = perfectly elastic. Default 0 */
} tp_material;

TP_API tp_material tp_material_default(void);

/* ------------------------------------------------------------------ shape */

typedef struct tp_shape_desc {
    tp_shape_type type;
    union {
        struct { float radius; }                     sphere;
        struct { tp_vec3 half_extents; }             box;
        struct { float radius; float half_height; }  capsule;  /* local +Y */
        struct { tp_vec3 normal; float distance; }   plane;
    } data;

    tp_transform local;     /* offset of the shape inside its body           */
    float        density;   /* kg/m^3; used to derive mass. Default 1000     */
    tp_material  material;
    uint32_t     layer;     /* which layer this shape belongs to (bitmask)   */
    uint32_t     mask;      /* which layers it collides with (bitmask)       */
    bool         is_sensor; /* reports overlaps but generates no response    */
} tp_shape_desc;

/* Fills in sane defaults; set `type` and `data` afterwards. Always use this
 * rather than a zeroed struct, so new fields added later stay safe. */
TP_API tp_shape_desc tp_shape_desc_default(void);

/* ------------------------------------------------------------------- body */

typedef struct tp_body_desc {
    tp_body_type type;
    tp_transform transform;
    tp_vec3      linear_velocity;
    tp_vec3      angular_velocity;
    float        linear_damping;    /* per-second velocity decay. Default 0.05 */
    float        angular_damping;   /* default 0.05                            */
    float        gravity_scale;     /* default 1.0                             */
    bool         lock_rotation;     /* useful for characters and pickups       */
    bool         start_asleep;
    void*        user_data;         /* your scene node pointer, entity id, ... */
} tp_body_desc;

TP_API tp_body_desc tp_body_desc_default(void);

/* ------------------------------------------------------------------ world */

typedef struct tp_world tp_world;   /* opaque */

typedef struct tp_world_desc {
    tp_vec3      gravity;              /* default (0, -9.81, 0)               */
    uint32_t     max_bodies;           /* initial capacity; grows as needed   */
    uint32_t     velocity_iterations;  /* solver iterations. Default 8        */
    uint32_t     position_iterations;  /* default 3                           */
    tp_allocator allocator;            /* zeroed => malloc/free               */
} tp_world_desc;

TP_API tp_world_desc tp_world_desc_default(void);

TP_API tp_world* tp_world_create(const tp_world_desc* desc);
TP_API void      tp_world_destroy(tp_world* world);

/*
 * Advance the simulation by exactly `dt` seconds.
 *
 * Call this with a FIXED dt (1/60 f is a good default) from an accumulator.
 * Passing a variable frame time makes the solver behave differently at every
 * frame rate and destroys reproducibility.
 */
TP_API void tp_world_step(tp_world* world, float dt);

TP_API void    tp_world_set_gravity(tp_world* world, tp_vec3 gravity);
TP_API tp_vec3 tp_world_get_gravity(const tp_world* world);
TP_API uint32_t tp_world_get_body_count(const tp_world* world);

/* ---------------------------------------------------------- body lifetime */

TP_API tp_body_id tp_body_create(tp_world* world, const tp_body_desc* desc);
TP_API void       tp_body_destroy(tp_world* world, tp_body_id body);
TP_API bool       tp_body_is_valid(const tp_world* world, tp_body_id body);

TP_API tp_shape_id tp_body_add_shape(tp_world* world, tp_body_id body,
                                     const tp_shape_desc* desc);

/* ------------------------------------------------------------ body access */

TP_API tp_transform tp_body_get_transform(const tp_world* world, tp_body_id body);
TP_API void         tp_body_set_transform(tp_world* world, tp_body_id body,
                                          tp_transform transform);

TP_API tp_vec3 tp_body_get_position(const tp_world* world, tp_body_id body);
TP_API tp_quat tp_body_get_rotation(const tp_world* world, tp_body_id body);

TP_API tp_vec3 tp_body_get_linear_velocity(const tp_world* world, tp_body_id body);
TP_API void    tp_body_set_linear_velocity(tp_world* world, tp_body_id body, tp_vec3 v);
TP_API tp_vec3 tp_body_get_angular_velocity(const tp_world* world, tp_body_id body);
TP_API void    tp_body_set_angular_velocity(tp_world* world, tp_body_id body, tp_vec3 w);

TP_API float tp_body_get_mass(const tp_world* world, tp_body_id body);
TP_API void  tp_body_set_mass(tp_world* world, tp_body_id body, float mass);

TP_API void* tp_body_get_user_data(const tp_world* world, tp_body_id body);
TP_API void  tp_body_set_user_data(tp_world* world, tp_body_id body, void* user_data);

TP_API bool tp_body_is_awake(const tp_world* world, tp_body_id body);
TP_API void tp_body_wake(tp_world* world, tp_body_id body);

/* ------------------------------------------------------------ body forces */

/* Continuous forces (N) and torques (N*m). Cleared at the end of every step. */
TP_API void tp_body_apply_force(tp_world* world, tp_body_id body, tp_vec3 force);
TP_API void tp_body_apply_force_at(tp_world* world, tp_body_id body,
                                   tp_vec3 force, tp_vec3 world_point);
TP_API void tp_body_apply_torque(tp_world* world, tp_body_id body, tp_vec3 torque);

/* Instantaneous impulses (N*s). Use these for jumps, hits, explosions. */
TP_API void tp_body_apply_impulse(tp_world* world, tp_body_id body, tp_vec3 impulse);
TP_API void tp_body_apply_impulse_at(tp_world* world, tp_body_id body,
                                     tp_vec3 impulse, tp_vec3 world_point);

/* ---------------------------------------------------------------- queries */

typedef struct tp_query_filter {
    uint32_t layer_mask;      /* only hit shapes whose layer & this != 0     */
    bool     hit_sensors;
    bool     hit_static;
    bool     hit_dynamic;
} tp_query_filter;

TP_API tp_query_filter tp_query_filter_default(void);

typedef struct tp_ray_hit {
    tp_body_id body;
    tp_vec3    point;      /* world-space intersection point                */
    tp_vec3    normal;     /* world-space surface normal at the hit          */
    float      distance;   /* along the ray, in metres                       */
} tp_ray_hit;

/* Returns true and fills `out_hit` on the closest hit within `max_distance`. */
TP_API bool tp_world_raycast(const tp_world* world,
                             tp_vec3 origin, tp_vec3 direction,
                             float max_distance,
                             tp_query_filter filter,
                             tp_ray_hit* out_hit);

/* ----------------------------------------------------------------- events */

typedef enum tp_contact_event_type {
    TP_CONTACT_BEGIN = 0,
    TP_CONTACT_END
} tp_contact_event_type;

typedef struct tp_contact_event {
    tp_contact_event_type type;
    tp_body_id            body_a;
    tp_body_id            body_b;
    tp_vec3               point;         /* representative contact point     */
    tp_vec3               normal;        /* points from A towards B          */
    float                 normal_impulse;/* how hard they hit, in N*s        */
} tp_contact_event;

/*
 * Events are buffered during tp_world_step and remain valid until the NEXT
 * call to tp_world_step. Polling rather than calling back keeps ordering
 * deterministic and stops you mutating the world mid-solve.
 */
TP_API const tp_contact_event* tp_world_get_contact_events(const tp_world* world,
                                                           uint32_t* out_count);

/* ------------------------------------------------------------ debug draw */

/* The engine never draws anything itself -- it hands you line segments and
 * you feed them to whatever renderer you have. */
typedef struct tp_debug_draw {
    void (*line_fn)(tp_vec3 from, tp_vec3 to, tp_vec3 color_rgb, void* ctx);
    void*  ctx;
    bool   draw_shapes;
    bool   draw_aabbs;
    bool   draw_contacts;
} tp_debug_draw;

TP_API void tp_world_debug_draw(const tp_world* world, const tp_debug_draw* draw);

/* ------------------------------------------------------------------ misc */

TP_API const char* tp_version_string(void);
TP_API const char* tp_result_string(tp_result result);

#ifdef __cplusplus
}
#endif

#endif /* TPHYS_H */
