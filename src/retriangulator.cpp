#include <queue>
#include <iostream>
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
    std::unordered_set<size_t> endpoints;
    for (const auto &it: neighborMap) {
        if (it.second.size() == 1) {
            endpoints.insert(it.first);
        }
    }
    
    std::cout << "endpoints:" << endpoints.size() << std::endl;
    
    while (!endpoints.empty()) {
        size_t startEndpoint = *endpoints.begin();
        endpoints.erase(startEndpoint);
        std::queue<size_t> q;
        q.push(startEndpoint);
        std::unordered_set<size_t> visited;
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
        std::cout << "polyline:";
        for (const auto &it: polyline)
            std::cout << it << " ";
        std::cout << std::endl;
    }
    // TODO:
}

void ReTriangulator::triangulate()
{
    lookupPolylinesFromNeighborMap(*m_neighborMapFrom3);
    // TODO:
}

