#include <SnapBox2D/snapshot.hpp>
#include <Box2D/Dynamics/b2World.h>
#include <turbo/memory/slab_allocator.hpp>

namespace tme = turbo::memory;

namespace snapbox2D {

snapshot::snapshot(const tme::concurrent_sized_slab& original, const b2World& world)
    :
	heap(new tme::concurrent_sized_slab(original)),
	step(world.GetProfile().step)
{ }

} // namespace snapbox2D
