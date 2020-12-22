#ifndef RE_TRIANGULATOR_H
#define RE_TRIANGULATOR_H
#include <unordered_map>
#include <unordered_set>
#include "vector3.h"
#include "vector2.h"

class ReTriangulator
{
public:
    ReTriangulator(const std::vector<Vector3> &points, 
        const Vector3 &normal);
    void setEdges(const std::vector<Vector3> &points,
        const std::unordered_map<size_t, std::unordered_set<size_t>> *neighborMapFrom3);
    void triangulate();
    const std::vector<std::vector<size_t>> &polygons() const;
private:
    Vector3 m_projectAxis;
    Vector3 m_projectOrigin;
    Vector3 m_projectNormal;
    std::vector<Vector2> m_points;
    const std::unordered_map<size_t, std::unordered_set<size_t>> *m_neighborMapFrom3 = nullptr;
    std::vector<std::vector<size_t>> m_polylines;
    std::vector<std::vector<size_t>> m_innerPolygons;
    std::vector<std::vector<size_t>> m_polygons;
    
    void lookupPolylinesFromNeighborMap(const std::unordered_map<size_t, std::unordered_set<size_t>> &neighborMap);
    int attachPointToTriangleEdge(const Vector2 &point);
    void buildInnerPolygonHierarchy();
    bool buildPolygons();
};

#endif
