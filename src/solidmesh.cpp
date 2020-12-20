#include "solidmesh.h"

SolidMesh::~SolidMesh()
{
    delete m_trangleNormals;
}

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

const std::vector<Vector3> *SolidMesh::triangleNormals() const
{
    return m_trangleNormals;
}

void SolidMesh::calculateTriangleNormals()
{
    if (nullptr == m_triangles)
        return;
    
    delete m_trangleNormals;
    m_trangleNormals = new std::vector<Vector3>;
    m_trangleNormals->reserve(m_triangles->size());
    for (const auto &it: *m_triangles) {
        m_trangleNormals->push_back(
            Vector3::normal((*m_vertices)[it[0]], 
                (*m_vertices)[it[1]], 
                (*m_vertices)[it[2]])
        );
    }
}
