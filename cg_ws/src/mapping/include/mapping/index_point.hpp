#ifndef MAPPING__INDEX_POINT_HPP
#define MAPPING__INDEX_POINT_HPP

namespace cg {
namespace mapping {

struct indexPoint {
  // a struct which simply defines a points position in the "2D" map, with a z-height
    int x;
    int y;
    float z;
};

} // mapping namespace
} // cg namespace

#endif // MAPPING__INDEX_POINT_HPP
