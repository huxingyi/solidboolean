#ifndef SOLID_BOOLEAN_H
#define SOLID_BOOLEAN_H
#include <unordered_map>
#include <unordered_set>
#include <map>
#include "vector3.h"
#include "solidmesh.h"
#include "axisalignedboundingboxtree.h"
#include "axisalignedboundingbox.h"

class SolidBoolean
{
public:
    SolidBoolean(const SolidMesh *firstMesh,
        const SolidMesh *secondMesh);
    ~SolidBoolean();
    void combine();
private:
    const SolidMesh *m_firstMesh = nullptr;
    const SolidMesh *m_secondMesh = nullptr;
    std::vector<std::pair<size_t, size_t>> *m_potentialIntersectedPairs = nullptr;
    
    void addMeshToAxisAlignedBoundingBox(const SolidMesh &mesh, AxisAlignedBoudingBox *box);
    void addTriagleToAxisAlignedBoundingBox(const SolidMesh &mesh, const std::vector<size_t> &triangle, AxisAlignedBoudingBox *box);
    void searchPotentialIntersectedPairs();
    bool intersectTwoFaces(size_t firstIndex, size_t secondIndex, std::pair<Vector3, Vector3> &newEdge);
    bool buildPolygonsFromEdges(const std::unordered_map<size_t, std::unordered_set<size_t>> &edges,
        std::vector<std::vector<size_t>> &polygons);
    bool isPolygonInward(const std::vector<size_t> &polygon, 
        const std::vector<Vector3> &vertices,
        const std::vector<Vector3> &triangleNormals, 
        const std::map<std::pair<size_t, size_t>, size_t> &halfEdges);
    
    void exportObject(const char *filename, const std::vector<Vector3> &vertices, const std::vector<std::vector<size_t>> &faces);
};

#endif
