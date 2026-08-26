# tiny_phys

A small 3D rigid body physics engine written in **C11**, built to drop into
C++ engines such as [tiny_engine](https://github.com/zen4xx/tiny_engine).

> **Status: pre-alpha.** The public API in `include/tphys/tphys.h` is settled,
> and bodies, handles and the integrator work. Collision detection and the
> constraint solver are not implemented yet — see the roadmap below.

- SI units throughout: metres, kilograms, seconds, radians
- Right-handed, Y up, quaternions stored `(x, y, z, w)` to match glTF
- Generational handles, not raw pointers — a stale handle is detectable
- No dependencies beyond the C standard library
- Optional custom allocator; the library never calls `malloc` behind your back

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
ctest --test-dir build --output-on-failure
```

With sanitizers, which is how you should develop:

```sh
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DTPHYS_SANITIZE=ON
cmake --build build-asan -j && ctest --test-dir build-asan --output-on-failure
```

## Use it in your project

```cmake
include(FetchContent)
FetchContent_Declare(
    tiny_phys
    GIT_REPOSITORY https://github.com/PtrH1kari/tiny_phys.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(tiny_phys)

target_link_libraries(myApp PRIVATE tphys::tphys)
```

## Example

```c
#include <tphys/tphys.h>

tp_world_desc wd = tp_world_desc_default();
wd.gravity = tp_v3(0.0f, -9.81f, 0.0f);
tp_world* world = tp_world_create(&wd);

tp_body_desc bd = tp_body_desc_default();
bd.transform.position = tp_v3(0.0f, 10.0f, 0.0f);
tp_body_id crate = tp_body_create(world, &bd);

/* Fixed timestep. Never pass a raw frame time to tp_world_step. */
const float FIXED_DT = 1.0f / 60.0f;
static float accumulator = 0.0f;

accumulator += frame_delta_seconds;
while (accumulator >= FIXED_DT) {
    tp_world_step(world, FIXED_DT);
    accumulator -= FIXED_DT;
}

tp_transform t = tp_body_get_transform(world, crate);

tp_world_destroy(world);
```

## Using it with tiny_engine

Keep the bridge in **one** C++ file in the game, not in either engine.
tiny_phys never learns what a renderer is, and tiny_engine never learns what a
rigid body is.

```cpp
// physics_bridge.cpp
#include <tphys/tphys.h>
#include <tiny_engine.h>
#include <vector>

struct PhysicsLink {
    tp_body_id body;
    SceneNode* node;    // whatever tiny_engine calls its transform node
};

static std::vector<PhysicsLink> g_links;

void SyncPhysicsToScene(tp_world* world, float alpha) {
    for (auto& link : g_links) {
        if (!tp_body_is_valid(world, link.body)) continue;
        tp_transform t = tp_body_get_transform(world, link.body);
        link.node->setPosition({ t.position.x, t.position.y, t.position.z });
        link.node->setRotation({ t.rotation.x, t.rotation.y,
                                 t.rotation.z, t.rotation.w });
    }
}
```

`alpha` is the leftover accumulator fraction — interpolate between the previous
and current transform with it once rendering runs at a different rate than the
fixed physics step.

## Roadmap

| Milestone | Contents | Status |
|---|---|---|
| M0 | Repo, CMake, CI, public API, test harness | done |
| M1 | Vector / quaternion / matrix math | done |
| M2 | Body pool, handles, semi-implicit Euler integrator | done |
| M3 | Broadphase: brute force, then a dynamic AABB tree | 30% |
| M4 | Narrowphase: sphere, box, capsule, plane; contact manifolds | |
| M5 | Sequential-impulse solver with friction and warm starting | |
| M6 | Raycasts, overlap queries, sensors, collision layers | |
| M7 | Islands, sleeping, profiling | |
| M8 | Kinematic character controller | |
| M9 | `v0.1.0-beta` | |

## License

MIT
