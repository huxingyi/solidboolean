#include "axisalignedboundingboxtree.h"

AxisAlignedBoudingBoxTree::AxisAlignedBoudingBoxTree(const std::vector<AxisAlignedBoudingBox> *boxes,
        const std::vector<size_t> &boxIndices,
        const AxisAlignedBoudingBox &outterBox)
{
    m_boxes = boxes;
    
    m_root = new Node;
    m_root->boundingBox = outterBox;
    m_root->boxIndices = boxIndices;
    
    if (!boxIndices.empty()) {
        for (const auto &boxIndex: boxIndices) {
            m_root->center += (*boxes)[boxIndex].center();
        }
        m_root->center /= (float)boxIndices.size();
    }
    
    splitNode(m_root);
}

AxisAlignedBoudingBoxTree::Node *AxisAlignedBoudingBoxTree::root()
{
    return m_root;
}

AxisAlignedBoudingBoxTree::~AxisAlignedBoudingBoxTree()
{
    deleteNode(m_root);
    delete m_testPairs;
}

void AxisAlignedBoudingBoxTree::collectNodeBoxMesh(const Node *node, std::vector<Vector3> &vertices, 
    std::vector<std::vector<size_t>> &faces) const
{
    if (nullptr == node)
        return;
    
    const Vector3 &lowerBound = node->boundingBox.lowerBound();
    const Vector3 &upperBound = node->boundingBox.upperBound();
    
    std::vector<size_t> topFace;
    topFace.push_back(vertices.size() + 0);
    topFace.push_back(vertices.size() + 1);
    topFace.push_back(vertices.size() + 2);
    topFace.push_back(vertices.size() + 3);
    
    vertices.push_back({lowerBound.x(), lowerBound.y(), upperBound.z()});
    vertices.push_back({upperBound.x(), lowerBound.y(), upperBound.z()});
    vertices.push_back({upperBound.x(), lowerBound.y(), lowerBound.z()});
    vertices.push_back({lowerBound.x(), lowerBound.y(), lowerBound.z()});
    
    std::vector<size_t> bottomFace;
    bottomFace.push_back(vertices.size() + 0);
    bottomFace.push_back(vertices.size() + 1);
    bottomFace.push_back(vertices.size() + 2);
    bottomFace.push_back(vertices.size() + 3);
    
    vertices.push_back({lowerBound.x(), upperBound.y(), upperBound.z()});
    vertices.push_back({upperBound.x(), upperBound.y(), upperBound.z()});
    vertices.push_back({upperBound.x(), upperBound.y(), lowerBound.z()});
    vertices.push_back({lowerBound.x(), upperBound.y(), lowerBound.z()});
    
    faces.push_back(topFace);
    for (size_t i = 0; i < topFace.size(); ++i) {
        size_t j = (i + 1) % topFace.size();
        faces.push_back({topFace[j], topFace[i], bottomFace[i], bottomFace[j]});
    }
    std::reverse(bottomFace.begin(), bottomFace.end());
    faces.push_back(bottomFace);
    
    collectNodeBoxMesh(node->left, vertices, faces);
    collectNodeBoxMesh(node->right, vertices, faces);
}

void AxisAlignedBoudingBoxTree::exportObject(const char *filename) const
{
    std::vector<Vector3> vertices;
    std::vector<std::vector<size_t>> faces;
    collectNodeBoxMesh(m_root, vertices, faces);
    
    FILE *fp = fopen(filename, "wb");
    for (const auto &it: vertices) {
        fprintf(fp, "v %f %f %f\n", it.x(), it.y(), it.z());
    }
    for (const auto &it: faces) {
        fprintf(fp, "f");
        for (const auto &v: it)
            fprintf(fp, " %zu", v + 1);
        fprintf(fp, "\n");
    }
    fclose(fp);
}

void AxisAlignedBoudingBoxTree::testNodes(const Node *first, const Node *second)
{
    if (first->boundingBox.intersectWith(second->boundingBox)) {
        if (first->isLeaf()) {
            if (second->isLeaf()) {
                if ((*m_boxes)[first->boxIndices.front()].intersectWith(
                        (*m_secondBoxes)[second->boxIndices.front()])) {
                    m_testPairs->push_back(std::make_pair(first->boxIndices.front(),
                        second->boxIndices.front()));
                }
            } else {
                testNodes(first, second->left);
                testNodes(first, second->right);
            }
        } else {
            if (second->isLeaf()) {
                testNodes(first->left, second);
                testNodes(first->right, second);
            } else {
                if (first->boxIndices.size() < second->boxIndices.size()) {
                    testNodes(first, second->left);
                    testNodes(first, second->right);
                } else {
                    testNodes(first->left, second);
                    testNodes(first->right, second);
                }
            }
        }
    }
}

std::vector<std::pair<size_t, size_t>> *AxisAlignedBoudingBoxTree::test(const Node *first, const Node *second,
    const std::vector<AxisAlignedBoudingBox> *secondBoxes)
{
    m_secondBoxes = secondBoxes;
    m_testPairs = new std::vector<std::pair<size_t, size_t>>;
    testNodes(first, second);
    auto testPairs = m_testPairs;
    m_testPairs = nullptr;
    return testPairs;
}

void AxisAlignedBoudingBoxTree::deleteNode(Node *node)
{
    if (nullptr == node)
        return;
    deleteNode(node->left);
    deleteNode(node->right);
    delete node;
}

void AxisAlignedBoudingBoxTree::splitNode(Node *node)
{
    const auto &boxIndices = node->boxIndices;
    if (boxIndices.size() < 2)
        return;
    const auto &splitBox = node->boundingBox;
    const Vector3 &lower = splitBox.lowerBound();
    const Vector3 &upper = splitBox.upperBound();
    std::vector<std::pair<size_t, float>> spans(3);
    for (size_t i = 0; i < 3; ++i)
        spans[i] = {i, upper[i] - lower[i]};
    std::sort(spans.begin(), spans.end(), [](const std::pair<size_t, float> &first,
            const std::pair<size_t, float> &second) {
        return first.second < second.second;
    });
    size_t longestAxis = spans[spans.size() - 1].first;
    auto splitPoint = node->center[longestAxis];
    node->left = new Node;
    node->right = new Node;
    std::vector<size_t> leftBoxIndices;
    std::vector<size_t> rightBoxIndices;
    for (size_t i = 0; i < boxIndices.size(); ++i) {
        const auto &boxIndex = boxIndices[i];
        const AxisAlignedBoudingBox &box = (*m_boxes)[boxIndex];
        const auto &center = box.center()[longestAxis];
        if (center < splitPoint)
            leftBoxIndices.push_back(boxIndex);
        else
            rightBoxIndices.push_back(boxIndex);
    }
    if (0 == leftBoxIndices.size()) {
        while (rightBoxIndices.size() > leftBoxIndices.size()) {
            leftBoxIndices.push_back(rightBoxIndices.back());
            rightBoxIndices.pop_back();
        }
    } else if (0 == rightBoxIndices.size()) {
        while (leftBoxIndices.size() > rightBoxIndices.size()) {
            rightBoxIndices.push_back(leftBoxIndices.back());
            leftBoxIndices.pop_back();
        }
    }
    for (const auto &boxIndex: leftBoxIndices) {
        const AxisAlignedBoudingBox &box = (*m_boxes)[boxIndex];
        node->left->boundingBox.update(box.lowerBound());
        node->left->boundingBox.update(box.upperBound());
        node->left->boxIndices.push_back(boxIndex);
        node->left->center += box.center();
    }
    node->left->center /= leftBoxIndices.size();
    for (const auto &boxIndex: rightBoxIndices) {
        const AxisAlignedBoudingBox &box = (*m_boxes)[boxIndex];
        node->right->boundingBox.update(box.lowerBound());
        node->right->boundingBox.update(box.upperBound());
        node->right->boxIndices.push_back(boxIndex);
        node->right->center += box.center();
    }
    node->right->center /= rightBoxIndices.size();

    node->left->center /= (float)node->left->boxIndices.size();
    splitNode(node->left);

    node->right->center /= (float)node->right->boxIndices.size();
    splitNode(node->right);
}
