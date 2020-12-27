#ifndef AXIS_ALIGNED_BOUNDING_BOX_H
#define AXIS_ALIGNED_BOUNDING_BOX_H
#include <cmath>
#include "vector3.h"

class AxisAlignedBoudingBox
{
public:
    void update(const Vector3 &vertex)
    {
        for (size_t i = 0; i < 3; ++i) {
            if (vertex[i] > m_max[i])
                m_max[i] = vertex[i];
            if (vertex[i] < m_min[i])
                m_min[i] = vertex[i];
            m_sum[i] += vertex[i];
        }
        ++m_num;
    }

    const Vector3 &center() const
    {
        return m_center;
    }

    void updateCenter()
    {
        if (0 == m_num)
            return;
        m_center = m_sum /= (float)m_num;
    }

    const Vector3 &lowerBound() const
    {
        return m_min;
    }

    const Vector3 &upperBound() const
    {
        return m_max;
    }

    Vector3 &lowerBound()
    {
        return m_min;
    }

    Vector3 &upperBound()
    {
        return m_max;
    }

    bool intersectWithAt(const AxisAlignedBoudingBox &other, AxisAlignedBoudingBox *result) const
    {
        const Vector3 &otherMin = other.lowerBound();
        const Vector3 &otherMax = other.upperBound();
        for (size_t i = 0; i < 3; ++i) {
            if (m_min[i] <= otherMax[i] && m_max[i] >= otherMin[i])
                continue;
            return false;
        }
        for (size_t i = 0; i < 3; ++i) {
            std::vector<double> points = {
                m_min[i], otherMax[i], m_max[i], otherMin[i]
            };
            std::sort(points.begin(), points.end());
            result->lowerBound()[i] = points[1];
            result->upperBound()[i] = points[2];
        }
        return true;
    }

    bool intersectWith(const AxisAlignedBoudingBox &other) const
    {
        const Vector3 &otherMin = other.lowerBound();
        const Vector3 &otherMax = other.upperBound();
        for (size_t i = 0; i < 3; ++i) {
            if (m_min[i] <= otherMax[i] && m_max[i] >= otherMin[i])
                continue;
            return false;
        }
        return true;
    }
    
private:
    Vector3 m_min = {
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
    };
    Vector3 m_max = {
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
    };
    Vector3 m_sum;
    size_t m_num = 0;
    Vector3 m_center;
};

#endif
