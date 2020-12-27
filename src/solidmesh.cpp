/*
 *  Copyright (c) 2020 Jeremy HU <jeremy-at-dust3d dot org>. All rights reserved. 
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:

 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
 
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
