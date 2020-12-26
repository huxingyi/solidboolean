#ifndef SOLID_BOOLEAN_H
#define SOLID_BOOLEAN_H
#include <unordered_map>
#include <unordered_set>
#include <map>
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
    void doUnion();
    void doDiff();
    void doIntersect();
    const std::vector<Vector3> &resultVertices();
    const std::vector<std::vector<size_t>> &resultTriangles();
    
    static void exportObject(const char *filename, const std::vector<Vector3> &vertices, const std::vector<std::vector<size_t>> &faces);
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
    std::vector<std::vector<size_t>> m_resultTriangles;
    
    void addMeshToAxisAlignedBoundingBox(const SolidMesh &mesh, AxisAlignedBoudingBox *box);
    void addTriagleToAxisAlignedBoundingBox(const SolidMesh &mesh, const std::vector<size_t> &triangle, AxisAlignedBoudingBox *box);
    void searchPotentialIntersectedPairs();
    bool intersectTwoFaces(size_t firstIndex, size_t secondIndex, std::pair<Vector3, Vector3> &newEdge);
    bool buildPolygonsFromEdges(const std::unordered_map<size_t, std::unordered_set<size_t>> &edges,
        std::vector<std::vector<size_t>> &polygons);
    bool isPointInMesh(const Vector3 &testPosition, 
        const SolidMesh *targetMesh,
        AxisAlignedBoudingBoxTree *meshBoxTree,
        const Vector3 &testAxis,
        const char *debugName=nullptr);
    void buildFaceGroups(const std::vector<std::vector<size_t>> &intersections,
        const std::map<std::pair<size_t, size_t>, size_t> &halfEdges,
        const std::vector<std::vector<size_t>> &triangles,
        std::vector<std::vector<size_t>> &triangleGroups);
    size_t addNewPoint(const Vector3 &position);
    void addRemainingTriangles(const SolidMesh *mesh, 
        const std::unordered_set<size_t> &usedFaces,
        std::map<std::pair<size_t, size_t>, size_t> &halfEdges);
    void decideGroupSide(const std::vector<std::vector<size_t>> &groups,
        const SolidMesh *mesh,
        AxisAlignedBoudingBoxTree *tree,
        std::vector<bool> &groupSides);
    void combine();
};

#endif
