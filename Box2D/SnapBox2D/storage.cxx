#include <SnapBox2D/storage.hpp>
#include <Box2D/Common/b2Settings.h>
#include <cstddef>
#include <turbo/memory/slab_allocator.hpp>
#include <SnapBox2D/snapshot.hpp>

namespace tme = turbo::memory;

namespace snapbox2D {

extern turbo::memory::concurrent_sized_slab& local_allocator();

} // namespace snapbox2D

void* b2Alloc(std::size_t size)
{
    return snapbox2D::local_allocator().malloc(size);
}

void b2Free(void* mem, std::size_t size)
{
    snapbox2D::local_allocator().free(mem, size);
}

namespace snapbox2D {

std::unique_ptr<snapshot> save(const b2World& world)
{
    return std::move(std::unique_ptr<snapshot>(new snapshot(local_allocator(), world)));
}

void restore(const snapshot& source)
{
    if (source.heap)
    {
	local_allocator() = *(source.heap);
    }
}

} // namespace snapbox2D
