namespace cg {
namespace mapping {

bool pointInMap(float x, float y, size_t width, size_t height, float resolution);
bool indexInMap(size_t x, size_t y, size_t width, size_t height);
bool heightInRange(float height, float minHeight, float maxHeight);
int coordsToCellIndex(float x, float y, size_t width, float resolution);

} // mapping namespace
} // cg namespace