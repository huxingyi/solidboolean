#ifndef POSITION_KEY_H
#define POSITION_KEY_H
#include "vector3.h"

class PositionKey
{
public:
    PositionKey(const Vector3 &v);
    PositionKey(double x, double y, double z);
    bool operator<(const PositionKey &right) const;
    bool operator==(const PositionKey &right) const;

private:
    long m_intX;
    long m_intY;
    long m_intZ;

    static long m_toIntFactor;
};

#endif
