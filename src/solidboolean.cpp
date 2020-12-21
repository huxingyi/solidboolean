#include <stdio.h>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include "solidboolean.h"
#include "tri_tri_intersect.h"
#include "retriangulator.h"
#include "positionkey.h"

SolidBoolean::SolidBoolean(const SolidMesh *m_firstMesh,
        const SolidMesh *m_secondMesh) :
    m_firstMesh(m_firstMesh),
    m_secondMesh(m_secondMesh)
{
}

SolidBoolean::~SolidBoolean()
{
    delete m_potentialIntersectedPairs;
}

void SolidBoolean::addMeshToAxisAlignedBoundingBox(const SolidMesh &mesh, AxisAlignedBoudingBox *box)
{
    for (const auto &vertex: *mesh.vertices())
        box->update(vertex);
}

void SolidBoolean::addTriagleToAxisAlignedBoundingBox(const SolidMesh &mesh, const std::vector<size_t> &triangle, AxisAlignedBoudingBox *box)
{
    for (size_t i = 0; i < 3; ++i)
        box->update((*mesh.vertices())[triangle[i]]);
}

void SolidBoolean::searchPotentialIntersectedPairs()
{
    std::vector<AxisAlignedBoudingBox> firstMeshFaceAABBs;
    std::vector<AxisAlignedBoudingBox> secondMeshFaceAABBs;
    firstMeshFaceAABBs.resize(m_firstMesh->triangles()->size());
    secondMeshFaceAABBs.resize(m_secondMesh->triangles()->size());
    for (size_t i = 0; i < firstMeshFaceAABBs.size(); ++i) {
        addTriagleToAxisAlignedBoundingBox(*m_firstMesh, (*m_firstMesh->triangles())[i], &firstMeshFaceAABBs[i]);
        firstMeshFaceAABBs[i].updateCenter();
    }
    for (size_t i = 0; i < secondMeshFaceAABBs.size(); ++i) {
        addTriagleToAxisAlignedBoundingBox(*m_secondMesh, (*m_secondMesh->triangles())[i], &secondMeshFaceAABBs[i]);
        secondMeshFaceAABBs[i].updateCenter();
    }
    
    AxisAlignedBoudingBox firstBox;
    AxisAlignedBoudingBox secondBox;
    AxisAlignedBoudingBox intersectedBox;
    addMeshToAxisAlignedBoundingBox(*m_firstMesh, &firstBox);
    addMeshToAxisAlignedBoundingBox(*m_secondMesh, &secondBox);
    firstBox.intersectWithAt(secondBox, &intersectedBox);
    std::vector<size_t> firstGroupOfFacesIn;
    std::vector<size_t> secondGroupOfFacesIn;
    for (size_t i = 0; i < firstMeshFaceAABBs.size(); ++i) {
        if (intersectedBox.intersectWith(firstMeshFaceAABBs[i])) {
            firstGroupOfFacesIn.push_back(i);
        }
    }
    for (size_t i = 0; i < secondMeshFaceAABBs.size(); ++i) {
        if (intersectedBox.intersectWith(secondMeshFaceAABBs[i])) {
            secondGroupOfFacesIn.push_back(i);
        }
    }

    AxisAlignedBoudingBox firstGroupBox;
    for (const auto &i: firstGroupOfFacesIn) {
        addTriagleToAxisAlignedBoundingBox(*m_firstMesh, (*m_firstMesh->triangles())[i], &firstGroupBox);
    }
    firstGroupBox.updateCenter();
    
    AxisAlignedBoudingBox secondGroupBox;
    for (const auto &i: secondGroupOfFacesIn) {
        addTriagleToAxisAlignedBoundingBox(*m_secondMesh, (*m_secondMesh->triangles())[i], &secondGroupBox);
    }
    secondGroupBox.updateCenter();
    
    AxisAlignedBoudingBoxTree leftTree(&firstMeshFaceAABBs, firstGroupOfFacesIn, firstGroupBox);
    AxisAlignedBoudingBoxTree rightTree(&secondMeshFaceAABBs, secondGroupOfFacesIn, secondGroupBox);
    
    m_potentialIntersectedPairs = leftTree.test(leftTree.root(), rightTree.root(), &secondMeshFaceAABBs);
}

bool SolidBoolean::intersectTwoFaces(size_t firstIndex, size_t secondIndex, std::pair<Vector3, Vector3> &newEdge)
{
    const auto &firstFace = (*m_firstMesh->triangles())[firstIndex];
    const auto &secondFace = (*m_secondMesh->triangles())[secondIndex];
    int coplanar = 0;
    if (!tri_tri_intersection_test_3d((double *)(*m_firstMesh->vertices())[firstFace[0]].constData(),
            (double *)(*m_firstMesh->vertices())[firstFace[1]].constData(),
            (double *)(*m_firstMesh->vertices())[firstFace[2]].constData(),
            (double *)(*m_secondMesh->vertices())[secondFace[0]].constData(),
            (double *)(*m_secondMesh->vertices())[secondFace[1]].constData(),
            (double *)(*m_secondMesh->vertices())[secondFace[2]].constData(),
            &coplanar,
            (double *)newEdge.first.constData(),
            (double *)newEdge.second.constData())) {
        return false;
    }
    if (coplanar)
        return false;
    return true;
}

void SolidBoolean::exportObject(const char *filename, const std::vector<Vector3> &vertices, const std::vector<std::vector<size_t>> &triangles)
{
    FILE *fp = fopen(filename, "wb");
    for (const auto &it: vertices) {
        fprintf(fp, "v %f %f %f\n", it.x(), it.y(), it.z());
    }
    for (const auto &it: triangles) {
        fprintf(fp, "f %zu %zu %zu\n", it[0] + 1, it[1] + 1, it[2] + 1);
    }
    fclose(fp);
}

void SolidBoolean::combine()
{
    searchPotentialIntersectedPairs();
    
    std::vector<std::vector<size_t>> firstBoundaryTriangles;
    std::vector<std::vector<size_t>> secondBoundaryTriangles;
    
    std::vector<Vector3> debugPoints;
    std::vector<std::vector<size_t>> debugFaces;
    
    struct IntersectedContext
    {
        std::vector<Vector3> points;
        std::map<PositionKey, size_t> positionMap;
        std::unordered_map<size_t, std::unordered_set<size_t>> neighborMap;
    };
    
    std::map<size_t, IntersectedContext> firstTriangleIntersectedContext;
    std::map<size_t, IntersectedContext> secondTriangleIntersectedContext;
    
    auto addIntersectedPoint = [](IntersectedContext &context, const Vector3 &position) {
        auto insertResult = context.positionMap.insert({PositionKey(position), context.points.size()});
        if (insertResult.second) {
            //std::cout << "[" << (3 + context.points.size()) << "] New position:" << position << std::endl;
            context.points.push_back(position);
        } //else {
            //std::cout << "[" << (3 + insertResult.first->second) << "] Repeated position:" << position << std::endl;
        //}
        return insertResult.first->second;
    };
    
    for (const auto &pair: *m_potentialIntersectedPairs) {
        std::pair<Vector3, Vector3> newEdge;
        if (intersectTwoFaces(pair.first, pair.second, newEdge)) {
            {
                auto &context = firstTriangleIntersectedContext[pair.first];
                size_t firstPointIndex = 3 + addIntersectedPoint(context, newEdge.first);
                size_t secondPointIndex = 3 + addIntersectedPoint(context, newEdge.second);
                if (firstPointIndex != secondPointIndex) {
                    context.neighborMap[firstPointIndex].insert(secondPointIndex);
                    context.neighborMap[secondPointIndex].insert(firstPointIndex);
                }
            }
            
            /*
            {
                auto &context = secondTriangleIntersectedContext[pair.second];
                size_t firstPointIndex = 3 + addIntersectedPoint(context, newEdge.first);
                size_t secondPointIndex = 3 + addIntersectedPoint(context, newEdge.second);
                if (firstPointIndex != secondPointIndex) {
                    context.neighborMap[firstPointIndex].insert(secondPointIndex);
                    context.neighborMap[secondPointIndex].insert(firstPointIndex);
                }
            }
            */
            
            debugFaces.push_back({debugPoints.size(), debugPoints.size() + 1});
            debugPoints.push_back(newEdge.first);
            debugPoints.push_back(newEdge.second);
            firstBoundaryTriangles.push_back((*m_firstMesh->triangles())[pair.first]);
            secondBoundaryTriangles.push_back((*m_secondMesh->triangles())[pair.second]);
        }
    }
    
    for (const auto &it: firstTriangleIntersectedContext) {
        const auto &triangle = (*m_firstMesh->triangles())[it.first];
        ReTriangulator reTriangulator({
                (*m_firstMesh->vertices())[triangle[0]],
                (*m_firstMesh->vertices())[triangle[1]],
                (*m_firstMesh->vertices())[triangle[2]]
            }, 
            (*m_firstMesh->triangleNormals())[it.first]
        );
        reTriangulator.setEdges(it.second.points,
            &it.second.neighborMap);
        reTriangulator.triangulate();
    }
    
    exportObject("debug-first.obj", *m_firstMesh->vertices(), firstBoundaryTriangles);
    exportObject("debug-second.obj", *m_secondMesh->vertices(), secondBoundaryTriangles);
   
    {
        FILE *fp = fopen("debug.obj", "wb");
        for (const auto &it: debugPoints) {
            fprintf(fp, "v %f %f %f\n", it.x(), it.y(), it.z());
        }
        for (const auto &it: debugFaces) {
            fprintf(fp, "l %zu %zu\n", it[0] + 1, it[1] + 1);
        }
        fclose(fp);
    }
}
