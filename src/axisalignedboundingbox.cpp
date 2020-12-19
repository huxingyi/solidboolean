#include <cmath>
#include "axisalignedboundingbox.h"

void AxisAlignedBoudingBox::update(const Vector3 &vertex)
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

const Vector3 &AxisAlignedBoudingBox::center() const
{
    return m_center;
}

void AxisAlignedBoudingBox::updateCenter()
{
    if (0 == m_num)
        return;
    m_center = m_sum /= (float)m_num;
}

const Vector3 &AxisAlignedBoudingBox::lowerBound() const
{
    return m_min;
}

const Vector3 &AxisAlignedBoudingBox::upperBound() const
{
    return m_max;
}

Vector3 &AxisAlignedBoudingBox::lowerBound()
{
    return m_min;
}

Vector3 &AxisAlignedBoudingBox::upperBound()
{
    return m_max;
}

bool AxisAlignedBoudingBox::intersectWithAt(const AxisAlignedBoudingBox &other, AxisAlignedBoudingBox *result) const
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
        std::sort(points.begin(), points.end(), [](const float &first, const float &second) {
            return first < second;
        });
        result->lowerBound()[i] = points[1];
        result->upperBound()[i] = points[2];
    }
    return true;
}

bool AxisAlignedBoudingBox::intersectWith(const AxisAlignedBoudingBox &other) const
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

