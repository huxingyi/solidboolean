#include <queue>
#include <iostream>
#include <earcut.hpp>
#include <array>
#include "retriangulator.h"

ReTriangulator::ReTriangulator(const std::vector<Vector3> &points, 
        const Vector3 &normal) :
    m_projectNormal(normal)
{
    m_projectAxis = (points[1] - points[0]).normalized();
    m_projectOrigin = points[0];
    
    Vector3::project(points, &m_points,
        m_projectNormal, m_projectAxis, m_projectOrigin);
}

void ReTriangulator::setEdges(const std::vector<Vector3> &points,
    const std::unordered_map<size_t, std::unordered_set<size_t>> *neighborMapFrom3)
{
    Vector3::project(points, &m_points,
        m_projectNormal, m_projectAxis, m_projectOrigin);
    m_neighborMapFrom3 = neighborMapFrom3;
}

void ReTriangulator::lookupPolylinesFromNeighborMap(const std::unordered_map<size_t, std::unordered_set<size_t>> &neighborMap)
{
    std::vector<size_t> endpoints;
    endpoints.reserve(neighborMap.size());
    for (const auto &it: neighborMap) {
        if (it.second.size() == 1) {
            //std::cout << "Endpoint:" << it.first << "=>count:" << it.second.size() << std::endl;
            endpoints.push_back(it.first);
        }
    }
    for (const auto &it: neighborMap) {
        if (it.second.size() > 1) {
            //std::cout << "Edge:" << it.first << "=>count:" << it.second.size() << std::endl;
            endpoints.push_back(it.first);
        }
    }
    
    //std::cout << "endpoints:" << endpoints.size() << std::endl;
    
    std::unordered_set<size_t> visited;
    for (const auto &startEndpoint: endpoints) {
        if (visited.find(startEndpoint) != visited.end())
            continue;
        std::queue<size_t> q;
        q.push(startEndpoint);
        std::vector<size_t> polyline;
        while (!q.empty()) {
            size_t loop = q.front();
            visited.insert(loop);
            polyline.push_back(loop);
            q.pop();
            auto neighborIt = neighborMap.find(loop);
            if (neighborIt == neighborMap.end())
                break;
            for (const auto &it: neighborIt->second) {
                if (visited.find(it) == visited.end()) {
                    q.push(it);
                    break;
                }
            }
        }
        if (polyline.size() >= 3) {
            auto neighborOfLast = neighborMap.find(polyline.back());
            if (neighborOfLast->second.find(startEndpoint) != neighborOfLast->second.end()) {
                m_innerPolygons.push_back(polyline);
                continue;
            }
        }
        m_polylines.push_back(polyline);
        
        /*
        std::cout << "polyline:";
        for (const auto &it: polyline)
            std::cout << it << " ";
        if (polyline.front() == polyline.back())
            std::cout << "(RING)";
        std::cout << std::endl;
        */
    }
}

int ReTriangulator::attachPointToTriangleEdge(const Vector2 &point)
{
    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;
        if (point.isOnLine(m_points[i], m_points[j]))
            return i;
    }
    return -1;
}

void ReTriangulator::buildPolygonHierarchy()
{
    for (size_t i = 0; i < m_innerPolygons.size(); ++i) {
        for (size_t j = i + 1; j < m_innerPolygons.size(); ++j) {
            if (m_points[m_innerPolygons[i][0]].isInPolygon(m_points, m_innerPolygons[j])) {
                m_innerChildrenMap[j].insert(i);
                m_innerParentsMap[i].insert(j);
            } else if (m_points[m_innerPolygons[j][0]].isInPolygon(m_points, m_innerPolygons[i])) {
                m_innerChildrenMap[i].insert(j);
                m_innerParentsMap[j].insert(i);
            }
        }
    }
    
    for (const auto &it: m_innerParentsMap)
        std::cout << "innerPolygon[" << it.first << "].parent count:" << it.second.size() << std::endl;

    for (size_t i = 0; i < m_innerPolygons.size(); ++i) {
        const auto &inner = m_innerPolygons[i];
        if (m_innerParentsMap.find(i) != m_innerParentsMap.end())
            continue;
        for (size_t j = 0; j < m_polygons.size(); ++j) {
            if (m_points[inner[0]].isInPolygon(m_points, m_polygons[j])) {
                m_polygonHoles[j].push_back(i);
            }
        }
    }
    
    for (const auto &it: m_polygonHoles) {
        std::cout << "Polygon[" << it.first << "] holes:" << std::endl;
        for (const auto &inner: it.second) {
            std::cout << "  inner:" << inner << std::endl;
        }
    }
}

bool ReTriangulator::buildPolygons()
{
    struct EdgePoint
    {
        size_t pointIndex = 0;
        size_t linkToPointIndex = 0;
        int polylineIndex = -1;
        bool reversed = false;
        double squaredDistance = 0.0;
        int linkTo = -1;
    };
    std::vector<std::vector<EdgePoint>> edgePoints(3);
    for (int polylineIndex = 0; polylineIndex < (int)m_polylines.size(); ++polylineIndex) {
        const auto &polyline = m_polylines[polylineIndex];
        int frontEdge = attachPointToTriangleEdge(m_points[polyline.front()]);
        int backEdge = attachPointToTriangleEdge(m_points[polyline.back()]);
        if (-1 == frontEdge || -1 == backEdge) {
            std::cout << "frontEdge:" << frontEdge << std::endl;
            std::cout << "backEdge:" << backEdge << std::endl;
            continue;
        }
        edgePoints[frontEdge].push_back({
            polyline.front(),
            polyline.back(),
            polylineIndex,
            false,
            (m_points[polyline.front()] - m_points[frontEdge]).lengthSquared()
        });
        edgePoints[backEdge].push_back({
            polyline.back(),
            polyline.front(),
            polylineIndex,
            true,
            (m_points[polyline.back()] - m_points[backEdge]).lengthSquared()
        });
    }
    for (auto &it: edgePoints) {
        std::sort(it.begin(), it.end(), [](const EdgePoint &first, const EdgePoint &second) {
            return first.squaredDistance < second.squaredDistance;
        });
    }
    
    // Turn triangle to ring
    std::vector<EdgePoint> ringPoints;
    for (size_t i = 0; i < 3; ++i) {
        ringPoints.push_back({i});
        for (const auto &it: edgePoints[i])
            ringPoints.push_back(it);
    }
    
    // Make polyline link
    std::unordered_map<size_t, size_t> pointRingPositionMap;
    for (size_t i = 0; i < ringPoints.size(); ++i) {
        const auto &it = ringPoints[i];
        if (-1 == it.polylineIndex)
            continue;
        pointRingPositionMap.insert({it.pointIndex, i});
    }
    for (size_t i = 0; i < ringPoints.size(); ++i) {
        auto &it = ringPoints[i];
        if (-1 == it.polylineIndex)
            continue;
        auto findLinkTo = pointRingPositionMap.find(it.linkToPointIndex);
        if (findLinkTo == pointRingPositionMap.end())
            continue;
        it.linkTo = findLinkTo->second;
    }
    
    //std::cout << "Build polygons..." << std::endl;
    
    std::unordered_set<size_t> visited;
    std::queue<size_t> startQueue;
    startQueue.push(0);
    while (!startQueue.empty()) {
        auto startIndex = startQueue.front();
        startQueue.pop();
        if (visited.find(startIndex) != visited.end())
            continue;
        std::vector<size_t> polygon;
        auto loopIndex = startIndex;
        do {
            auto &it = ringPoints[loopIndex];
            visited.insert(loopIndex);
            if (-1 == it.polylineIndex) {
                polygon.push_back(it.pointIndex);
                loopIndex = (loopIndex + 1) % ringPoints.size();
            } else if (-1 != it.linkTo) {
                const auto &polyline = m_polylines[it.polylineIndex];
                if (it.reversed) {
                    for (int i = (int)polyline.size() - 1; i >= 0; --i)
                        polygon.push_back(polyline[i]);
                } else {
                    for (const auto &pointIndex: polyline)
                        polygon.push_back(pointIndex);
                }
                startQueue.push((loopIndex + 1) % ringPoints.size());
                loopIndex = (it.linkTo + 1) % ringPoints.size();
            } else {
                std::cerr << "linkTo failed" << std::endl;
                return false;
            }
        } while (loopIndex != startIndex);
        m_polygons.push_back(polygon);
    }
    
    //std::cout << "m_polylines:" << m_polylines.size() << std::endl;
    //for (size_t i = 0; i < ringPoints.size(); ++i) {
    //    const auto &it = ringPoints[i];
    //    std::cout << "[" << i << "] point:" << it.pointIndex << " linkPoint:" << it.linkToPointIndex << " polylineIndex:" << it.polylineIndex << " linkTo:" << it.linkTo << std::endl;
    //}
    
    //for (size_t i = 0; i < m_polygons.size(); ++i) {
    //    const auto &it = m_polygons[i];
    //    std::cout << "Polygon[" << i << "]:";
    //    for (const auto &pointIndex: it)
    //        std::cout << pointIndex << "\t";
    //    std::cout << std::endl;
    //}
    
    return true;
}

void ReTriangulator::triangulate()
{
    for (size_t polygonIndex = 0; polygonIndex < m_polygons.size(); ++polygonIndex) {
        const auto &polygon = m_polygons[polygonIndex];
        
        std::vector<std::vector<std::array<double, 2>>> polygonAndHoles;
        std::vector<size_t> pointIndices;

        std::vector<std::array<double, 2>> border;
        for (const auto &it: polygon) {
            pointIndices.push_back(it);
            const auto &v = m_points[it];
            border.push_back(std::array<double, 2> {v.x(), v.y()});
        }
        polygonAndHoles.push_back(border);
        
        auto findHoles = m_polygonHoles.find(polygonIndex);
        if (findHoles != m_polygonHoles.end()) {
            for (const auto &h: findHoles->second) {
                std::vector<std::array<double, 2>> hole;
                for (const auto &it: m_innerPolygons[h]) {
                    pointIndices.push_back(it);
                    const auto &v = m_points[it];
                    hole.push_back(std::array<double, 2> {v.x(), v.y()});
                }
                polygonAndHoles.push_back(hole);
            }
        }

        std::vector<size_t> indices = mapbox::earcut<size_t>(polygonAndHoles);
        m_triangles.reserve(indices.size() / 3);
        for (size_t i = 0; i < indices.size(); i += 3) {
            m_triangles.push_back({
                pointIndices[indices[i]],
                pointIndices[indices[i + 1]],
                pointIndices[indices[i + 2]]
            });
        }
    }
    
    for (size_t polygonIndex = 0; polygonIndex < m_innerPolygons.size(); ++polygonIndex) {
        const auto &polygon = m_innerPolygons[polygonIndex];

        std::vector<std::vector<std::array<double, 2>>> polygonAndHoles;
        std::vector<size_t> pointIndices;

        std::vector<std::array<double, 2>> border;
        for (const auto &it: polygon) {
            pointIndices.push_back(it);
            const auto &v = m_points[it];
            border.push_back(std::array<double, 2> {v.x(), v.y()});
        }
        polygonAndHoles.push_back(border);
        
        auto childrenIt = m_innerChildrenMap.find(polygonIndex);
        if (childrenIt != m_innerChildrenMap.end()) {
            auto children = childrenIt->second;
            for (const auto &child: childrenIt->second) {
                auto grandChildrenIt = m_innerChildrenMap.find(child);
                if (grandChildrenIt != m_innerChildrenMap.end()) {
                    for (const auto &grandChild: grandChildrenIt->second) {
                        std::cout << "Grand child removed:" << grandChild << std::endl;
                        children.erase(grandChild);
                    }
                }
            }
            for (const auto &child: children) {
                std::vector<std::array<double, 2>> hole;
                for (const auto &it: m_innerPolygons[child]) {
                    pointIndices.push_back(it);
                    const auto &v = m_points[it];
                    hole.push_back(std::array<double, 2> {v.x(), v.y()});
                }
                polygonAndHoles.push_back(hole);
            }
        }
        
        std::vector<size_t> indices = mapbox::earcut<size_t>(polygonAndHoles);
        m_triangles.reserve(indices.size() / 3);
        for (size_t i = 0; i < indices.size(); i += 3) {
            m_triangles.push_back({
                pointIndices[indices[i]],
                pointIndices[indices[i + 1]],
                pointIndices[indices[i + 2]]
            });
        }
    }
}

void ReTriangulator::reTriangulate()
{
    lookupPolylinesFromNeighborMap(*m_neighborMapFrom3);
    buildPolygons();
    buildPolygonHierarchy();
    triangulate();
}

const std::vector<std::vector<size_t>> &ReTriangulator::polygons() const
{
    return m_polygons;
}

const std::vector<std::vector<size_t>> &ReTriangulator::triangles() const
{
    return m_triangles;
}

