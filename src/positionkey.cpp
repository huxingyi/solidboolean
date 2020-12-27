#include "positionkey.h"

long PositionKey::m_toIntFactor = 100000;

PositionKey::PositionKey(const Vector3 &v) :
    PositionKey(v.x(), v.y(), v.z())
{
}

PositionKey::PositionKey(double x, double y, double z)
{
    m_intX = (long)(x * m_toIntFactor);
    m_intY = (long)(y * m_toIntFactor);
    m_intZ = (long)(z * m_toIntFactor);
}

bool PositionKey::operator<(const PositionKey &right) const
{
    if (m_intX < right.m_intX)
        return true;
    if (m_intX > right.m_intX)
        return false;
    if (m_intY < right.m_intY)
        return true;
    if (m_intY > right.m_intY)
        return false;
    if (m_intZ < right.m_intZ)
        return true;
    if (m_intZ > right.m_intZ)
        return false;
    return false;
}

bool PositionKey::operator==(const PositionKey &right) const
{
    return m_intX == right.m_intX &&
        m_intY == right.m_intY &&
        m_intZ == right.m_intZ;
}
