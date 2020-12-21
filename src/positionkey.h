#ifndef POSITION_KEY_H
#define POSITION_KEY_H
#include "vector3.h"

class PositionKey
{
public:
    PositionKey(const Vector3 &v);
    PositionKey(double x, double y, double z);
    const Vector3 &position() const;
    bool operator<(const PositionKey &right) const;
    bool operator==(const PositionKey &right) const;

private:
    long m_intX = 0;
    long m_intY = 0;
    long m_intZ = 0;
    Vector3 m_position;

    static long m_toIntFactor;
};

#endif
