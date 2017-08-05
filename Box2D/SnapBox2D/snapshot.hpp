#ifndef SNAPBOX2D_SNAPSHOT_HPP
#define SNAPBOX2D_SNAPSHOT_HPP

#include <cstdint>
#include <memory>

class b2World;

namespace turbo {
namespace memory {

class concurrent_sized_slab;

} // namespace memory
} // nemespace turbo

namespace snapbox2D {

struct snapshot
{
    snapshot() = delete;
    snapshot(const turbo::memory::concurrent_sized_slab& original, const b2World& world);
    std::unique_ptr<turbo::memory::concurrent_sized_slab> heap;
    float step;
};

} // namespace snapbox2D

#endif
