#include "solidmesh.h"

void SolidMesh::setVertices(const std::vector<Vector3> *vertices)
{
    m_vertices = vertices;
}

void SolidMesh::setTriangles(const std::vector<std::vector<size_t>> *triangles)
{
    m_triangles = triangles;
}

const std::vector<Vector3> *SolidMesh::vertices() const
{
    return m_vertices;
}

const std::vector<std::vector<size_t>> *SolidMesh::triangles() const
{
    return m_triangles;
}
