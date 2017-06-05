#ifndef SNAPBOX2D_STORAGE_HPP
#define SNAPBOX2D_STORAGE_HPP

#include <memory>

class b2World;

namespace snapbox2D {

class snapshot;

std::unique_ptr<snapshot> save(const b2World& world);

void restore(const snapshot& source);

} // namespace snapbox2D

#endif
