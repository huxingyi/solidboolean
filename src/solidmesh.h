#ifndef SOLID_MESH_H
#define SOLID_MESH_H
#include "vector3.h"

class SolidMesh
{
public:
    void setVertices(const std::vector<Vector3> *vertices);
    void setTriangles(const std::vector<std::vector<size_t>> *triangles);
    const std::vector<Vector3> *vertices() const;
    const std::vector<std::vector<size_t>> *triangles() const;
private:
    const std::vector<Vector3> *m_vertices = nullptr;
    const std::vector<std::vector<size_t>> *m_triangles = nullptr;
};

#endif
