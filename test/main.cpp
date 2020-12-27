#include <string>
#include <iostream>
#include <chrono>
#include "vector3.h"
#include "solidboolean.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

static void exportObject(const char *filename, const std::vector<Vector3> &vertices, const std::vector<std::vector<size_t>> &faces)
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

static bool loadObj(const std::string &filename, 
    std::vector<Vector3> &outputVertices, 
    std::vector<std::vector<size_t>> &outputTriangles)
{
    tinyobj::attrib_t attributes;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;
    
    bool loadSuccess = tinyobj::LoadObj(&attributes, &shapes, &materials, &warn, &err, filename.c_str());
    if (!warn.empty()) {
        std::cout << "WARN:" << warn.c_str() << std::endl;
    }
    if (!err.empty()) {
        std::cout << err.c_str() << std::endl;
    }
    if (!loadSuccess) {
        return false;
    }
    
    outputVertices.resize(attributes.vertices.size() / 3);
    for (size_t i = 0, j = 0; i < outputVertices.size(); ++i) {
        auto &dest = outputVertices[i];
        dest.setX(attributes.vertices[j++]);
        dest.setY(attributes.vertices[j++]);
        dest.setZ(attributes.vertices[j++]);
    }
    
    outputTriangles.clear();
    for (const auto &shape: shapes) {
        for (size_t i = 0; i < shape.mesh.indices.size(); i += 3) {
            outputTriangles.push_back(std::vector<size_t> {
                (size_t)shape.mesh.indices[i + 0].vertex_index,
                (size_t)shape.mesh.indices[i + 1].vertex_index,
                (size_t)shape.mesh.indices[i + 2].vertex_index
            });
        }
    }
    
    return true;
}

int main(int argc, char ** argv)
{
    std::vector<Vector3> firstVertices;
    std::vector<std::vector<size_t>> firstTriangles;
    
    std::vector<Vector3> secondVertices; 
    std::vector<std::vector<size_t>> secondTriangles;
    
    loadObj("../../cases/addax-and-meerkat/a.obj", firstVertices, firstTriangles);
    loadObj("../../cases/addax-and-meerkat/b.obj", secondVertices, secondTriangles);
    
    //loadObj("../../cases/cube-sphere/a.obj", firstVertices, firstTriangles);
    //loadObj("../../cases/cube-sphere/b.obj", secondVertices, secondTriangles);
    
    auto t1 = std::chrono::high_resolution_clock::now();
    
    SolidMesh firstMesh;
    firstMesh.setVertices(&firstVertices);
    firstMesh.setTriangles(&firstTriangles);
    firstMesh.calculateTriangleNormals();
    
    SolidMesh secondMesh;
    secondMesh.setVertices(&secondVertices);
    secondMesh.setTriangles(&secondTriangles);
    secondMesh.calculateTriangleNormals();
    
    SolidBoolean solidBoolean(&firstMesh, &secondMesh);
    solidBoolean.combine();
    
    auto t2 = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    
    std::cout << "Duration:" << duration << std::endl;
    
    std::cout << "\tsearchPotentialIntersectedPairs:" << std::chrono::duration_cast<std::chrono::milliseconds>(solidBoolean.benchEnd_searchPotentialIntersectedPairs - solidBoolean.benchBegin_searchPotentialIntersectedPairs).count() << std::endl;
    std::cout << "\t\tbuildTrees:" << std::chrono::duration_cast<std::chrono::milliseconds>(solidBoolean.benchEnd_buildTrees - solidBoolean.benchBegin_buildTrees).count() << std::endl;
    std::cout << "\tprocessPotentialIntersectedPairs:" << std::chrono::duration_cast<std::chrono::milliseconds>(solidBoolean.benchEnd_processPotentialIntersectedPairs - solidBoolean.benchBegin_processPotentialIntersectedPairs).count() << std::endl;
    std::cout << "\taddUnintersectedTriangles:" << std::chrono::duration_cast<std::chrono::milliseconds>(solidBoolean.benchEnd_addUnintersectedTriangles - solidBoolean.benchBegin_addUnintersectedTriangles).count() << std::endl;
    std::cout << "\treTriangulate:" << std::chrono::duration_cast<std::chrono::milliseconds>(solidBoolean.benchEnd_reTriangulate - solidBoolean.benchBegin_reTriangulate).count() << std::endl;
    std::cout << "\tbuildPolygonsFromEdges:" << std::chrono::duration_cast<std::chrono::milliseconds>(solidBoolean.benchEnd_buildPolygonsFromEdges - solidBoolean.benchBegin_buildPolygonsFromEdges).count() << std::endl;
    std::cout << "\tbuildFaceGroups:" << std::chrono::duration_cast<std::chrono::milliseconds>(solidBoolean.benchEnd_buildFaceGroups - solidBoolean.benchBegin_buildFaceGroups).count() << std::endl;
    std::cout << "\tdecideGroupSide:" << std::chrono::duration_cast<std::chrono::milliseconds>(solidBoolean.benchEnd_decideGroupSide - solidBoolean.benchBegin_decideGroupSide).count() << std::endl;
    
    std::vector<Vector3> mergedVertices;
    std::vector<std::vector<size_t>> mergedTriangles;
    
    auto mergeMesh = [&](const std::vector<Vector3> &vertices, 
            const std::vector<std::vector<size_t>> &triangles,
            const Vector3 &offset) {
        size_t oldSize = mergedVertices.size();
        for (const auto &it: triangles) {
            mergedTriangles.push_back({
                it[0] + oldSize,
                it[1] + oldSize,
                it[2] + oldSize
            });
        }
        for (const auto &it: vertices) {
            mergedVertices.push_back(it + offset);
        }
    };
    
    {
        std::vector<std::vector<size_t>> resultTriangles;
        solidBoolean.fetchUnion(resultTriangles);
        mergeMesh(solidBoolean.resultVertices(), resultTriangles, Vector3(0.0, 0.0, 0.0));
    }
    {
        std::vector<std::vector<size_t>> resultTriangles;
        solidBoolean.fetchDiff(resultTriangles);
        mergeMesh(solidBoolean.resultVertices(), resultTriangles, Vector3(-1.0, 0.0, 0.0));
    }
    {
        std::vector<std::vector<size_t>> resultTriangles;
        solidBoolean.fetchIntersect(resultTriangles);
        mergeMesh(solidBoolean.resultVertices(), resultTriangles, Vector3(1.0, 0.0, 0.0));
    }
    
    exportObject("debug-merged-result.obj", 
        mergedVertices, mergedTriangles);
    
    return 0;
}
