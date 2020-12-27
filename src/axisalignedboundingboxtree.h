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
    void testNodes(const Node *first, const Node *second);
    std::vector<std::pair<size_t, size_t>> *test(const Node *first, const Node *second,
        const std::vector<AxisAlignedBoudingBox> *secondBoxes);
private:
    const std::vector<AxisAlignedBoudingBox> *m_boxes = nullptr;
    const std::vector<AxisAlignedBoudingBox> *m_secondBoxes = nullptr;
    Node *m_root = nullptr;
    std::vector<std::pair<size_t, size_t>> *m_testPairs = nullptr;
};

#endif
