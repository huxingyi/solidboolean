#include <stdio.h>
#include <queue>
#include <set>
#include "solidboolean.h"
#include "tri_tri_intersect.h"
#include "retriangulator.h"
#include "positionkey.h"

static const std::vector<Vector3> g_testAxisList = {
    {std::numeric_limits<double>::max(), std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::epsilon()},
    {std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::max(), std::numeric_limits<double>::epsilon()},
    {std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::max()},
};

SolidBoolean::SolidBoolean(const SolidMesh *m_firstMesh,
        const SolidMesh *m_secondMesh) :
    m_firstMesh(m_firstMesh),
    m_secondMesh(m_secondMesh)
{
}

SolidBoolean::~SolidBoolean()
{
    delete m_potentialIntersectedPairs;
    delete m_leftTree;
    delete m_rightTree;
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

bool SolidBoolean::isPointInMesh(const Vector3 &testPosition, 
    const SolidMesh *targetMesh,
    AxisAlignedBoudingBoxTree *meshBoxTree,
    const Vector3 &testAxis,
    const char *debugName)
{
    Vector3 testEnd = testPosition + testAxis;
    bool inside = false;
    std::vector<AxisAlignedBoudingBox> rayBox(1);
    auto &box = rayBox[0];
    box.update(testPosition);
    box.update(testEnd);
    AxisAlignedBoudingBoxTree testTree(&rayBox,
        {0},
        rayBox[0]);
    std::vector<std::pair<size_t, size_t>> *pairs = meshBoxTree->test(meshBoxTree->root(), testTree.root(), &rayBox);
    std::set<PositionKey> hits;
    
    std::vector<Vector3> debugVertices;
    std::vector<std::vector<size_t>> debugLines;
    
    //std::cout << "=====================" << std::endl;
    for (const auto &it: *pairs) {
        //std::cout << "Test pair:" << it.first << "~" << it.second;
        const auto &triangle = (*targetMesh->triangles())[it.first];
        std::vector<Vector3> trianglePositions = {
            (*targetMesh->vertices())[triangle[0]],
            (*targetMesh->vertices())[triangle[1]],
            (*targetMesh->vertices())[triangle[2]]
        };
        Vector3 intersection;
        if (Vector3::intersectSegmentAndPlane(testPosition, testEnd,
                trianglePositions[0], 
                (*targetMesh->triangleNormals())[it.first],
                &intersection)) {
            std::vector<Vector3> normals;
            for (size_t i = 0; i < 3; ++i) {
                size_t j = (i + 1) % 3;
                normals.push_back(Vector3::normal(intersection, trianglePositions[i], trianglePositions[j]));
            }
            if (Vector3::dotProduct(normals[0], normals[1]) > 0 == Vector3::dotProduct(normals[0], normals[2]) > 0) {
                debugLines.push_back({debugVertices.size(), debugVertices.size() + 1});
                debugVertices.push_back(testPosition);
                debugVertices.push_back(intersection);
                
                hits.insert(PositionKey(intersection));
                //std::cout << " hits[" << hits.size() << "]: intersection:" << intersection;
            }
        }
        //std::cout << std::endl;
    }
    inside = 0 != hits.size() % 2;
    delete pairs;
    
    if (nullptr != debugName) {
        exportObject(debugName, debugVertices, debugLines);
    }
    
    return inside;
}

void SolidBoolean::searchPotentialIntersectedPairs()
{
    m_firstMeshFaceAABBs.resize(m_firstMesh->triangles()->size());
    m_secondMeshFaceAABBs.resize(m_secondMesh->triangles()->size());
    for (size_t i = 0; i < m_firstMeshFaceAABBs.size(); ++i) {
        addTriagleToAxisAlignedBoundingBox(*m_firstMesh, (*m_firstMesh->triangles())[i], &m_firstMeshFaceAABBs[i]);
        m_firstMeshFaceAABBs[i].updateCenter();
    }
    for (size_t i = 0; i < m_secondMeshFaceAABBs.size(); ++i) {
        addTriagleToAxisAlignedBoundingBox(*m_secondMesh, (*m_secondMesh->triangles())[i], &m_secondMeshFaceAABBs[i]);
        m_secondMeshFaceAABBs[i].updateCenter();
    }
    
    AxisAlignedBoudingBox firstBox;
    AxisAlignedBoudingBox secondBox;
    AxisAlignedBoudingBox intersectedBox;
    addMeshToAxisAlignedBoundingBox(*m_firstMesh, &firstBox);
    addMeshToAxisAlignedBoundingBox(*m_secondMesh, &secondBox);
    firstBox.intersectWithAt(secondBox, &intersectedBox);
    std::vector<size_t> firstGroupOfFacesIn;
    std::vector<size_t> secondGroupOfFacesIn;
    for (size_t i = 0; i < m_firstMeshFaceAABBs.size(); ++i) {
        //if (intersectedBox.intersectWith(m_firstMeshFaceAABBs[i])) {
            firstGroupOfFacesIn.push_back(i);
        //}
    }
    for (size_t i = 0; i < m_secondMeshFaceAABBs.size(); ++i) {
        //if (intersectedBox.intersectWith(m_secondMeshFaceAABBs[i])) {
            secondGroupOfFacesIn.push_back(i);
        //}
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
    
    m_leftTree = new AxisAlignedBoudingBoxTree(&m_firstMeshFaceAABBs, firstGroupOfFacesIn, firstGroupBox);
    m_rightTree = new AxisAlignedBoudingBoxTree(&m_secondMeshFaceAABBs, secondGroupOfFacesIn, secondGroupBox);
    
    m_potentialIntersectedPairs = m_leftTree->test(m_leftTree->root(), m_rightTree->root(), &m_secondMeshFaceAABBs);
    
    //m_leftTree->exportObject("debug-lefttree.obj");
    //m_rightTree->exportObject("debug-righttree.obj");
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

void SolidBoolean::exportObject(const char *filename, const std::vector<Vector3> &vertices, const std::vector<std::vector<size_t>> &faces)
{
    FILE *fp = fopen(filename, "wb");
    for (const auto &it: vertices) {
        fprintf(fp, "v %f %f %f\n", it.x(), it.y(), it.z());
    }
    for (const auto &it: faces) {
        if (it.size() == 2) {
            fprintf(fp, "l");
            for (const auto &v: it)
                fprintf(fp, " %zu", v + 1);
            fprintf(fp, "\n");
            continue;
        }
        fprintf(fp, "f");
        for (const auto &v: it)
            fprintf(fp, " %zu", v + 1);
        fprintf(fp, "\n");
    }
    fclose(fp);
}

bool SolidBoolean::buildPolygonsFromEdges(const std::unordered_map<size_t, std::unordered_set<size_t>> &edges,
        std::vector<std::vector<size_t>> &polygons)
{
    std::unordered_set<size_t> visited;
    for (const auto &edge: edges) {
        const auto &startEndpoint = edge.first;
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
            auto neighborIt = edges.find(loop);
            if (neighborIt == edges.end())
                break;
            for (const auto &it: neighborIt->second) {
                if (visited.find(it) == visited.end()) {
                    q.push(it);
                    break;
                }
            }
        }
        if (polyline.size() <= 2) {
            std::cout << "buildPolygonsFromEdges failed, too short" << std::endl;
            return false;
        }
        
        auto neighborOfLast = edges.find(polyline.back());
        if (neighborOfLast->second.find(startEndpoint) == neighborOfLast->second.end()) {
            std::cout << "buildPolygonsFromEdges failed, could not form a ring" << std::endl;
            return false;
        }
        
        polygons.push_back(polyline);
    }
    
    return true;
}

void SolidBoolean::buildFaceGroups(const std::vector<std::vector<size_t>> &intersections,
        const std::map<std::pair<size_t, size_t>, size_t> &halfEdges,
        const std::vector<std::vector<size_t>> &triangles,
        std::vector<std::vector<size_t>> &triangleGroups)
{
    std::map<std::pair<size_t, size_t>, size_t> halfEdgeGroupMap;
    size_t groupIndex = 0;
    std::queue<std::pair<size_t, size_t>> q;
    for (const auto &intersection: intersections) {
        for (size_t i = 0; i < intersection.size(); ++i) {
            size_t j = (i + 1) % intersection.size();
            {
                std::pair<size_t, size_t> halfEdge = {intersection[i], intersection[j]};
                halfEdgeGroupMap.insert({halfEdge, groupIndex});
                auto halfEdgeIt = halfEdges.find(halfEdge);
                if (halfEdgeIt != halfEdges.end()) {
                    q.push({halfEdgeIt->second, groupIndex});
                }
            }
            {
                std::pair<size_t, size_t> halfEdge = {intersection[j], intersection[i]};
                halfEdgeGroupMap.insert({halfEdge, groupIndex + 1});
                auto halfEdgeIt = halfEdges.find(halfEdge);
                if (halfEdgeIt != halfEdges.end()) {
                    q.push({halfEdgeIt->second, groupIndex + 1});
                }
            }
        }
        groupIndex += 2;
    }
    
    triangleGroups.resize(groupIndex);
    std::unordered_set<size_t> visitedTriangles;
    while (!q.empty()) {
        auto triangleAndGroupIndex = q.front();
        q.pop();
        if (visitedTriangles.find(triangleAndGroupIndex.first) != visitedTriangles.end())
            continue;
        visitedTriangles.insert(triangleAndGroupIndex.first);
        triangleGroups[triangleAndGroupIndex.second].push_back(triangleAndGroupIndex.first);
        const auto &indicies = triangles[triangleAndGroupIndex.first];
        for (size_t i = 0; i < 3; ++i) {
            size_t j = (i + 1) % 3;
            std::pair<size_t, size_t> halfEdge = {indicies[i], indicies[j]};
            if (halfEdgeGroupMap.find(halfEdge) != halfEdgeGroupMap.end())
                continue;
            halfEdgeGroupMap.insert({halfEdge, triangleAndGroupIndex.second});
            auto halfEdgeIt = halfEdges.find({halfEdge.second, halfEdge.first});
            if (halfEdgeIt != halfEdges.end()) {
                q.push({halfEdgeIt->second, triangleAndGroupIndex.second});
            }
        }
    }
}

size_t SolidBoolean::addNewPoint(const Vector3 &position) 
{
    auto insertResult = m_newPositionMap.insert({PositionKey(position), m_newVertices.size()});
    if (insertResult.second) {
        m_newVertices.push_back(position);
    }
    return insertResult.first->second;
}

void SolidBoolean::addRemainingTriangles(const SolidMesh *mesh, 
    const std::unordered_set<size_t> &usedFaces,
    std::map<std::pair<size_t, size_t>, size_t> &halfEdges) 
{
    size_t triangleCount = mesh->triangles()->size();
    for (size_t i = 0; i < triangleCount; ++i) {
        if (usedFaces.find(i) != usedFaces.end())
            continue;
        const auto &oldTriangle = (*mesh->triangles())[i];
        size_t newInsertedIndex = m_newTriangles.size();
        m_newTriangles.push_back({
            addNewPoint((*mesh->vertices())[oldTriangle[0]]),
            addNewPoint((*mesh->vertices())[oldTriangle[1]]),
            addNewPoint((*mesh->vertices())[oldTriangle[2]])
        });
        const auto &newInsertedTriangle = m_newTriangles.back();
        halfEdges.insert({{newInsertedTriangle[0], newInsertedTriangle[1]}, newInsertedIndex});
        halfEdges.insert({{newInsertedTriangle[1], newInsertedTriangle[2]}, newInsertedIndex});
        halfEdges.insert({{newInsertedTriangle[2], newInsertedTriangle[0]}, newInsertedIndex});
    }
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
        if (insertResult.second)
            context.points.push_back(position);
        return insertResult.first->second;
    };
    
    std::unordered_set<size_t> firstIntersectedFaces;
    std::unordered_set<size_t> secondIntersectedFaces;
    for (const auto &pair: *m_potentialIntersectedPairs) {
        std::pair<Vector3, Vector3> newEdge;
        if (intersectTwoFaces(pair.first, pair.second, newEdge)) {
            firstIntersectedFaces.insert(pair.first);
            secondIntersectedFaces.insert(pair.second);
            
            {
                auto &context = firstTriangleIntersectedContext[pair.first];
                size_t firstPointIndex = 3 + addIntersectedPoint(context, newEdge.first);
                size_t secondPointIndex = 3 + addIntersectedPoint(context, newEdge.second);
                if (firstPointIndex != secondPointIndex) {
                    context.neighborMap[firstPointIndex].insert(secondPointIndex);
                    context.neighborMap[secondPointIndex].insert(firstPointIndex);
                }
            }
            
            {
                auto &context = secondTriangleIntersectedContext[pair.second];
                size_t firstPointIndex = 3 + addIntersectedPoint(context, newEdge.first);
                size_t secondPointIndex = 3 + addIntersectedPoint(context, newEdge.second);
                if (firstPointIndex != secondPointIndex) {
                    context.neighborMap[firstPointIndex].insert(secondPointIndex);
                    context.neighborMap[secondPointIndex].insert(firstPointIndex);
                }
            }

            debugFaces.push_back({debugPoints.size(), debugPoints.size() + 1});
            debugPoints.push_back(newEdge.first);
            debugPoints.push_back(newEdge.second);
            firstBoundaryTriangles.push_back((*m_firstMesh->triangles())[pair.first]);
            secondBoundaryTriangles.push_back((*m_secondMesh->triangles())[pair.second]);
        }
    }
    
    std::unordered_map<size_t, std::unordered_set<size_t>> firstEdges;
    std::unordered_map<size_t, std::unordered_set<size_t>> secondEdges;
    std::vector<std::vector<size_t>> firstIntersections;
    std::vector<std::vector<size_t>> secondIntersections;
    std::map<std::pair<size_t, size_t>, size_t> firstHalfEdges;
    std::map<std::pair<size_t, size_t>, size_t> secondHalfEdges;
    
    auto reTriangulate = [&](const std::map<size_t, IntersectedContext> &context,
            const SolidMesh *mesh, 
            std::unordered_map<size_t, std::unordered_set<size_t>> &edges,
            std::map<std::pair<size_t, size_t>, size_t> &halfEdges) {
        for (const auto &it: context) {
            const auto &triangle = (*mesh->triangles())[it.first];
            ReTriangulator reTriangulator({
                    (*mesh->vertices())[triangle[0]],
                    (*mesh->vertices())[triangle[1]],
                    (*mesh->vertices())[triangle[2]]
                }, 
                (*mesh->triangleNormals())[it.first]
            );
            reTriangulator.setEdges(it.second.points,
                &it.second.neighborMap);
            reTriangulator.reTriangulate();
            std::vector<size_t> newIndices;
            newIndices.push_back(addNewPoint((*mesh->vertices())[triangle[0]]));
            newIndices.push_back(addNewPoint((*mesh->vertices())[triangle[1]]));
            newIndices.push_back(addNewPoint((*mesh->vertices())[triangle[2]]));
            for (const auto &point: it.second.points)
                newIndices.push_back(addNewPoint(point));
            for (const auto &triangle: reTriangulator.triangles()) {
                //newTriangleNormals.push_back((*mesh->triangleNormals())[it.first]);
                size_t newInsertedIndex = m_newTriangles.size();
                m_newTriangles.push_back({
                    newIndices[triangle[0]],
                    newIndices[triangle[1]],
                    newIndices[triangle[2]]
                });
                const auto &newInsertedTriangle = m_newTriangles.back();
                halfEdges.insert({{newInsertedTriangle[0], newInsertedTriangle[1]}, newInsertedIndex});
                halfEdges.insert({{newInsertedTriangle[1], newInsertedTriangle[2]}, newInsertedIndex});
                halfEdges.insert({{newInsertedTriangle[2], newInsertedTriangle[0]}, newInsertedIndex});
            }
            for (const auto &it: it.second.neighborMap) {
                auto from = newIndices[it.first];
                for (const auto &it2: it.second) {
                    auto to = newIndices[it2];
                    edges[from].insert(to);
                    edges[to].insert(from);
                }
            }
        }
    };
    reTriangulate(firstTriangleIntersectedContext, m_firstMesh, firstEdges, firstHalfEdges);
    reTriangulate(secondTriangleIntersectedContext, m_secondMesh, secondEdges, secondHalfEdges);
    
    buildPolygonsFromEdges(firstEdges, firstIntersections);
    
    addRemainingTriangles(m_firstMesh, firstIntersectedFaces, firstHalfEdges);
    addRemainingTriangles(m_secondMesh, secondIntersectedFaces, secondHalfEdges);

    buildFaceGroups(firstIntersections,
        firstHalfEdges,
        m_newTriangles,
        m_firstTriangleGroups);
    buildFaceGroups(firstIntersections,
        secondHalfEdges,
        m_newTriangles,
        m_secondTriangleGroups);
        
    decideGroupSide(m_firstTriangleGroups,
        m_secondMesh,
        m_rightTree,
        m_firstGroupSides);
    decideGroupSide(m_secondTriangleGroups,
        m_firstMesh,
        m_leftTree,
        m_secondGroupSides);
    
    /*
    std::vector<Vector3> g_testAxisList = {
        {std::numeric_limits<double>::max(), std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::epsilon()},
        {std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::max(), std::numeric_limits<double>::epsilon()},
        {std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::max()},
    };
    size_t groupIndex = 0;
    for (size_t i = 0; i < m_firstTriangleGroups.size(); ++i) {
        const auto &group = m_firstTriangleGroups[i];
        if (group.empty())
            continue;
        size_t insideCount = 0;
        size_t totalCount = 0;
        for (size_t pickIndex = 0; pickIndex < 6 && pickIndex < group.size(); ++pickIndex) {
            for (size_t axisIndex = 0; axisIndex < g_testAxisList.size(); ++axisIndex) {
                //char debugName[200];
                //sprintf(debugName, "debug-line-group%zu-axis-%zu.obj", groupIndex, axisIndex);
                const auto &pickedTriangle = m_newTriangles[group[pickIndex]];
                bool inside = isPointInMesh((m_newVertices[pickedTriangle[0]] +
                        m_newVertices[pickedTriangle[1]] +
                        m_newVertices[pickedTriangle[2]]) / 3.0, 
                    m_secondMesh,
                    m_rightTree,
                    g_testAxisList[axisIndex],
                    nullptr); //debugName);
                if (inside)
                    ++insideCount;
                ++totalCount;
            }
        }
        
        std::cout << "Group[" << groupIndex << "]:";
        std::cout << "Decide on " << insideCount << " result:" << (((float)insideCount / totalCount > 0.5) ? "inside" : "outside");
        std::cout << std::endl;
        
        std::vector<std::vector<size_t>> groupTriangles;
        for (const auto &it: group)
            groupTriangles.push_back(m_newTriangles[it]);
        char filename[200];
        sprintf(filename, "debug-group-%zu.obj", groupIndex);
        exportObject(filename, m_newVertices, groupTriangles);
        ++groupIndex;
    }
    */
    
    exportObject("debug-triangles.obj", m_newVertices, m_newTriangles);
    
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

void SolidBoolean::decideGroupSide(const std::vector<std::vector<size_t>> &groups,
        const SolidMesh *mesh,
        AxisAlignedBoudingBoxTree *tree,
        std::vector<bool> &groupSides)
{
    groupSides.resize(groups.size());
    for (size_t i = 0; i < groups.size(); ++i) {
        const auto &group = groups[i];
        if (group.empty())
            continue;
        size_t insideCount = 0;
        size_t totalCount = 0;
        for (size_t pickIndex = 0; pickIndex < 6 && pickIndex < group.size(); ++pickIndex) {
            for (size_t axisIndex = 0; axisIndex < g_testAxisList.size(); ++axisIndex) {
                //char debugName[200];
                //sprintf(debugName, "debug-line-group%zu-axis-%zu.obj", groupIndex, axisIndex);
                const auto &pickedTriangle = m_newTriangles[group[pickIndex]];
                bool inside = isPointInMesh((m_newVertices[pickedTriangle[0]] +
                        m_newVertices[pickedTriangle[1]] +
                        m_newVertices[pickedTriangle[2]]) / 3.0, 
                    mesh,
                    tree,
                    g_testAxisList[axisIndex],
                    nullptr); //debugName);
                if (inside)
                    ++insideCount;
                ++totalCount;
            }
        }
        groupSides[i] = (float)insideCount / totalCount > 0.5;
    }
}

void SolidBoolean::doUnion()
{
    combine();
    
    for (size_t i = 0; i < m_firstGroupSides.size(); ++i) {
        if (m_firstGroupSides[i])
            continue;
        for (const auto &it: m_firstTriangleGroups[i])
            m_resultTriangles.push_back(m_newTriangles[it]);
    }
    
    for (size_t i = 0; i < m_secondGroupSides.size(); ++i) {
        if (m_secondGroupSides[i])
            continue;
        for (const auto &it: m_secondTriangleGroups[i])
            m_resultTriangles.push_back(m_newTriangles[it]);
    }
}

void SolidBoolean::doDiff()
{
    combine();
    
    for (size_t i = 0; i < m_firstGroupSides.size(); ++i) {
        if (m_firstGroupSides[i])
            continue;
        for (const auto &it: m_firstTriangleGroups[i])
            m_resultTriangles.push_back(m_newTriangles[it]);
    }
    
    for (size_t i = 0; i < m_secondGroupSides.size(); ++i) {
        if (!m_secondGroupSides[i])
            continue;
        for (const auto &it: m_secondTriangleGroups[i]) {
            auto triangle = m_newTriangles[it];
            m_resultTriangles.push_back({
                triangle[2], triangle[1], triangle[0]
            });
        }
    }
}

void SolidBoolean::doIntersect()
{
    combine();
    
    for (size_t i = 0; i < m_firstGroupSides.size(); ++i) {
        if (!m_firstGroupSides[i])
            continue;
        for (const auto &it: m_firstTriangleGroups[i])
            m_resultTriangles.push_back(m_newTriangles[it]);
    }
    
    for (size_t i = 0; i < m_secondGroupSides.size(); ++i) {
        if (!m_secondGroupSides[i])
            continue;
        for (const auto &it: m_secondTriangleGroups[i])
            m_resultTriangles.push_back(m_newTriangles[it]);
    }
}

const std::vector<Vector3> &SolidBoolean::resultVertices()
{
    return m_newVertices;
}

const std::vector<std::vector<size_t>> &SolidBoolean::resultTriangles()
{
    return m_resultTriangles;
}
