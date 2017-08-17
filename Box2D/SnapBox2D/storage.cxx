#include <SnapBox2D/storage.hpp>
#include <Box2D/Common/b2Settings.h>
#include <cstddef>
#include <turbo/memory/slab_allocator.hpp>
#include <turbo/memory/slab_allocator.hxx>
#include <SnapBox2D/snapshot.hpp>

namespace tme = turbo::memory;

namespace runtime_ {

turbo::memory::concurrent_sized_slab& local_allocator();

} // namespace runtime_

void* b2Alloc(std::size_t size)
{
    return runtime_::local_allocator().malloc(size);
}

void b2Free(void* mem, std::size_t size)
{
    runtime_::local_allocator().free(mem, size);
}

namespace snapbox2D {

std::unique_ptr<snapshot> save(const b2World& world)
{
    return std::move(std::unique_ptr<snapshot>(new snapshot(runtime_::local_allocator(), world)));
}

void restore(const snapshot& source)
{
    if (source.heap)
    {
	runtime_::local_allocator() = *(source.heap);
    }
}

} // namespace snapbox2D
