#ifndef SOLID_BOOLEAN_H
#define SOLID_BOOLEAN_H
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <chrono>
#include "vector3.h"
#include "solidmesh.h"
#include "axisalignedboundingboxtree.h"
#include "axisalignedboundingbox.h"
#include "positionkey.h"

class SolidBoolean
{
public:
    SolidBoolean(const SolidMesh *firstMesh,
        const SolidMesh *secondMesh);
    ~SolidBoolean();
    bool combine();
    void fetchUnion(std::vector<std::vector<size_t>> &resultTriangles);
    void fetchDiff(std::vector<std::vector<size_t>> &resultTriangles);
    void fetchIntersect(std::vector<std::vector<size_t>> &resultTriangles);
    
    const std::vector<Vector3> &resultVertices();
    std::chrono::time_point<std::chrono::high_resolution_clock> benchBegin_buildTrees;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchEnd_buildTrees;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchBegin_searchPotentialIntersectedPairs;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchEnd_searchPotentialIntersectedPairs;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchBegin_processPotentialIntersectedPairs;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchEnd_processPotentialIntersectedPairs;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchBegin_addUnintersectedTriangles;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchEnd_addUnintersectedTriangles;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchBegin_reTriangulate;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchEnd_reTriangulate;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchBegin_buildPolygonsFromEdges;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchEnd_buildPolygonsFromEdges;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchBegin_buildFaceGroups;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchEnd_buildFaceGroups;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchBegin_decideGroupSide;
    std::chrono::time_point<std::chrono::high_resolution_clock> benchEnd_decideGroupSide;
private:
    const SolidMesh *m_firstMesh = nullptr;
    const SolidMesh *m_secondMesh = nullptr;
    AxisAlignedBoudingBoxTree *m_leftTree = nullptr;
    AxisAlignedBoudingBoxTree *m_rightTree = nullptr;
    std::vector<std::pair<size_t, size_t>> *m_potentialIntersectedPairs = nullptr;
    std::vector<AxisAlignedBoudingBox> m_firstMeshFaceAABBs;
    std::vector<AxisAlignedBoudingBox> m_secondMeshFaceAABBs;
    std::vector<Vector3> m_newVertices;
    std::vector<std::vector<size_t>> m_newTriangles;
    std::map<PositionKey, size_t> m_newPositionMap;
    std::vector<std::vector<size_t>> m_firstTriangleGroups;
    std::vector<std::vector<size_t>> m_secondTriangleGroups;
    std::vector<bool> m_firstGroupSides;
    std::vector<bool> m_secondGroupSides;
    std::unordered_set<size_t> m_firstIntersectedFaces;
    std::unordered_set<size_t> m_secondIntersectedFaces;
    std::unordered_map<size_t, std::vector<size_t>> m_firstFacesAroundVertexMap;
    std::unordered_map<size_t, std::vector<size_t>> m_secondFacesAroundVertexMap;
    
    static inline uint64_t makeHalfEdgeKey(size_t first, size_t second)
    {
        return (first << 32) | second;
    }
    
    void addTriagleToAxisAlignedBoundingBox(const SolidMesh &mesh, const std::vector<size_t> &triangle, AxisAlignedBoudingBox *box)
    {
        for (size_t i = 0; i < 3; ++i)
            box->update((*mesh.vertices())[triangle[i]]);
    }
    
    void searchPotentialIntersectedPairs();
    bool intersectTwoFaces(size_t firstIndex, size_t secondIndex, std::pair<Vector3, Vector3> &newEdge);
    bool buildPolygonsFromEdges(const std::unordered_map<size_t, std::unordered_set<size_t>> &edges,
        std::vector<std::vector<size_t>> &polygons);
    bool isPointInMesh(const Vector3 &testPosition, 
        const SolidMesh *targetMesh,
        AxisAlignedBoudingBoxTree *meshBoxTree,
        const Vector3 &testAxis);
    void buildFaceGroups(const std::vector<std::vector<size_t>> &intersections,
        const std::unordered_map<uint64_t, size_t> &halfEdges,
        const std::vector<std::vector<size_t>> &triangles,
        size_t remainingStartTriangleIndex,
        size_t remainingTriangleCount,
        std::vector<std::vector<size_t>> &triangleGroups);
    size_t addNewPoint(const Vector3 &position);
    bool addUnintersectedTriangles(const SolidMesh *mesh, 
        const std::unordered_set<size_t> &usedFaces,
        std::unordered_map<uint64_t, size_t> *halfEdges);
    void decideGroupSide(const std::vector<std::vector<size_t>> &groups,
        const SolidMesh *mesh,
        AxisAlignedBoudingBoxTree *tree,
        std::vector<bool> &groupSides);
};

#endif
