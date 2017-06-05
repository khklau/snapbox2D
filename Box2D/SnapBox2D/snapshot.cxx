#include <SnapBox2D/snapshot.hpp>
#include <Box2D/Dynamics/b2World.h>
#include <turbo/cinterop/untyped_allocator.hpp>

namespace tci = turbo::cinterop;

namespace snapbox2D {

snapshot::snapshot(const tci::untyped_allocator& original, const b2World& world)
    :
	heap(new tci::untyped_allocator(original)),
	step(world.GetProfile().step)
{ }

} // namespace snapbox2D
