/*
 *  Copyright (c) 2020 Jeremy HU <jeremy-at-dust3d dot org>. All rights reserved. 
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:

 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
 
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
    bool reTriangulate();
    const std::vector<std::vector<size_t>> &polygons() const;
    const std::vector<std::vector<size_t>> &triangles() const;
private:
    Vector3 m_projectAxis;
    Vector3 m_projectOrigin;
    Vector3 m_projectNormal;
    std::vector<Vector2> m_points;
    const std::unordered_map<size_t, std::unordered_set<size_t>> *m_neighborMapFrom3 = nullptr;
    std::vector<std::vector<size_t>> m_polylines;
    std::vector<std::vector<size_t>> m_innerPolygons;
    std::vector<std::vector<size_t>> m_polygons;
    std::unordered_map<size_t, std::unordered_set<size_t>> m_innerParentsMap;
    std::unordered_map<size_t, std::unordered_set<size_t>> m_innerChildrenMap;
    std::unordered_map<size_t, std::vector<size_t>> m_polygonHoles;
    std::vector<std::vector<size_t>> m_triangles;
    
    void lookupPolylinesFromNeighborMap(const std::unordered_map<size_t, std::unordered_set<size_t>> &neighborMap);
    int attachPointToTriangleEdge(const Vector2 &point);
    bool buildPolygons();
    void buildPolygonHierarchy();
    void triangulate();
};

#endif
