#include <SnapBox2D/storage.hpp>
#include <Box2D/Common/b2Settings.h>
#include <cstddef>
#include <turbo/cinterop/untyped_allocator.hpp>
#include <SnapBox2D/snapshot.hpp>

namespace tci = turbo::cinterop;
namespace tme = turbo::memory;

namespace {

std::vector<tme::block_config> load_config()
{
    // TODO load the config from a file
    std::vector<tme::block_config> result;
    return std::move(result);
}

tci::untyped_allocator& local_allocator()
{
    thread_local tci::untyped_allocator allocator_(4U, load_config());
    return allocator_;
}

} // anonymous namespace

void* b2Alloc(std::size_t size)
{
    return local_allocator().malloc(size);
}

void b2Free(void* mem)
{
    local_allocator().free(mem);
}

namespace snapbox2d {

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

} // namespace snapbox2d
