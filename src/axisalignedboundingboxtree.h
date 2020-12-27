#ifndef AXIS_ALIGNED_BOUNDING_BOX_TREE_H
#define AXIS_ALIGNED_BOUNDING_BOX_TREE_H
#include <vector>
#include "axisalignedboundingbox.h"

class AxisAlignedBoudingBoxTree
{
public:
    struct Node
    {
        AxisAlignedBoudingBox boundingBox;
        Vector3 center;
        std::vector<size_t> boxIndices;
        Node *left = nullptr;
        Node *right = nullptr;
        
        bool isLeaf() const
        {
            return nullptr == left && nullptr == right;
        };
    };
    
    AxisAlignedBoudingBoxTree(const std::vector<AxisAlignedBoudingBox> *boxes,
        const std::vector<size_t> &boxIndices,
        const AxisAlignedBoudingBox &outterBox);
    Node *root();
    ~AxisAlignedBoudingBoxTree();
    void splitNode(Node *node);
    void deleteNode(Node *node);
    
    void testNodes(const Node *first, const Node *second)
    {
        if (first->boundingBox.intersectWith(second->boundingBox)) {
            if (first->isLeaf()) {
                if (second->isLeaf()) {
                    for (const auto &a: first->boxIndices) {
                        for (const auto &b: second->boxIndices) {
                            if ((*m_boxes)[a].intersectWith((*m_secondBoxes)[b])) {
                                m_testPairs->push_back(std::make_pair(a, b));
                            }
                        }
                    }
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
    
    std::vector<std::pair<size_t, size_t>> *test(const Node *first, const Node *second,
        const std::vector<AxisAlignedBoudingBox> *secondBoxes);
private:
    const std::vector<AxisAlignedBoudingBox> *m_boxes = nullptr;
    const std::vector<AxisAlignedBoudingBox> *m_secondBoxes = nullptr;
    Node *m_root = nullptr;
    std::vector<std::pair<size_t, size_t>> *m_testPairs = nullptr;
    std::vector<size_t> m_boxIndicesOrderList;
    std::vector<std::pair<size_t, float>> m_spans = std::vector<std::pair<size_t, float>>(3);
    
    static const size_t m_leafMaxNodeSize;
};

#endif
