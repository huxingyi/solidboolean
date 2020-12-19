#ifndef AXIS_ALIGNED_BOUNDING_BOX_H
#define AXIS_ALIGNED_BOUNDING_BOX_H
#include "vector3.h"

class AxisAlignedBoudingBox
{
public:
    void update(const Vector3 &vertex);
    const Vector3 &lowerBound() const;
    const Vector3 &upperBound() const;
    Vector3 &lowerBound();
    Vector3 &upperBound();
    bool intersectWith(const AxisAlignedBoudingBox &other) const;
    bool intersectWithAt(const AxisAlignedBoudingBox &other, AxisAlignedBoudingBox *result) const;
    const Vector3 &center() const;
    void updateCenter();
    
private:
    Vector3 m_min = {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
    };
    Vector3 m_max = {
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
    };
    Vector3 m_sum;
    size_t m_num = 0;
    Vector3 m_center;
};

#endif
