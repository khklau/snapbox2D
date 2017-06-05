#ifndef SNAPBOX2D_SNAPSHOT_HPP
#define SNAPBOX2D_SNAPSHOT_HPP

#include <cstdint>
#include <memory>

class b2World;

namespace turbo {
namespace cinterop {

class untyped_allocator;

} // namespace cinterop
} // nemespace turbo

namespace snapbox2D {

struct snapshot
{
    snapshot() = delete;
    snapshot(const turbo::cinterop::untyped_allocator& original, const b2World& world);
    std::unique_ptr<turbo::cinterop::untyped_allocator> heap;
    float step;
};

} // namespace snapbox2D

#endif
