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
    size_t longestAxis = std::max_element(spans.begin(), spans.end(), [](const std::pair<size_t, float> &first,
            const std::pair<size_t, float> &second) {
        return first.second < second.second;
    })->first;
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
