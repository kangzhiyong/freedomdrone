#pragma once

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <random>

#include "Point.hpp"

/**
 * C++ k-d tree implementation, based on the C version at rosettacode.org.
 */
template<typename coordinate_type, size_t dimensions>
class KDTree
{
public:
    typedef Point<coordinate_type, dimensions> point_type;
private:
    struct node
    {
        node(const point_type& pt) : point_(pt), left_(nullptr), right_(nullptr)
        {
        }
        coordinate_type get(size_t index) const
        {
            return point_.get(index);
        }
        float distance(const point_type& pt) const
        {
            return point_.distance(pt);
        }

        point_type point_;
        node* left_;
        node* right_;
        long pos_index;
        float nearest_dist_{0};
    };
    typedef std::vector<node> VNodeType;
    node* root_;
    node* best_;
    float best_dist_;
    size_t visited_;
    VNodeType nodes_;
    struct node_cmp
    {
        node_cmp(size_t index) : index_(index)
        {
        }
        bool operator()(const node& n1, const node& n2) const
        {
            return n1.point_.get(index_) < n2.point_.get(index_);
        }
        size_t index_;
    };

    node* make_tree(size_t begin, size_t end, size_t index)
    {
        if (end <= begin)
            return nullptr;
        size_t n = begin + (end - begin)/2;
        std::nth_element(&nodes_[begin], &nodes_[n], &nodes_[0] + end, node_cmp(index));
        index = (index + 1) % dimensions;
        nodes_[n].left_ = make_tree(begin, n, index);
        nodes_[n].right_ = make_tree(n + 1, end, index);
        return &nodes_[n];
    }

    void checkNearestMore(node* root, float d, int k)
    {
        root->nearest_dist_ = d;
        m_vNbrNodes.emplace_back(*root);
        std::sort(m_vNbrNodes.begin(), m_vNbrNodes.end(), [](node n1, node n2){return n1.nearest_dist_ < n2.nearest_dist_;});
        if (m_vNbrNodes.size() > k)
        {
            m_vNbrNodes.erase(m_vNbrNodes.end() - 1);
        }
        best_dist_ = m_vNbrNodes[m_vNbrNodes.size() - 1].nearest_dist_;
        best_ = &m_vNbrNodes[m_vNbrNodes.size() - 1];
    }
    void nearest(node* root, const point_type& p, size_t index, int k)
    {
        if (root == nullptr)
        {
            return;
        }
        ++visited_;
        double d = root->distance(p);
        if (best_ == nullptr || d < best_dist_) {
            checkNearestMore(root, d, k);
        }
        if (best_dist_ == 0)
        {
            return;
        }
        double dx = root->get(index) - p.get(index);
        index = (index + 1) % dimensions;
        nearest(dx > 0 ? root->left_ : root->right_, p, index, k);
        if (dx * dx >= best_dist_)
        {
            return;
        }
        nearest(dx > 0 ? root->right_ : root->left_, p, index, k);
    }

    void nearest(node* root, const point_type& p, size_t index) {
        if (root == nullptr)
            return;
        ++visited_;
        double d = root->distance(p);
        if (best_ == nullptr || d < best_dist_) {
            best_dist_ = d;
            best_ = root;
        }
        if (best_dist_ == 0)
            return;
        double dx = root->get(index) - p.get(index);
        index = (index + 1) % dimensions;
        nearest(dx > 0 ? root->left_ : root->right_, p, index);
        if (dx * dx >= best_dist_)
            return;
        nearest(dx > 0 ? root->right_ : root->left_, p, index);
    }

    void nearest_radius(node* root, const point_type& p, size_t index, float radius)
    {
        if (root == nullptr)
            return;
        ++visited_;
        float d = root->distance(p);
        if (d < radius)
        {
            m_vNbrNodes.emplace_back(*root);
        }

        float dx = root->get(index) - p.get(index);
        index = (index + 1) % dimensions;
        nearest_radius(dx > 0 ? root->left_ : root->right_, p, index, radius);
        if (dx * dx >= radius)
            return;
        nearest_radius(dx > 0 ? root->right_ : root->left_, p, index, radius);
    }

public:
    KDTree(const KDTree&) = delete;
    KDTree& operator=(const KDTree&) = delete;
    KDTree()
    {
        best_ = nullptr;
        best_dist_ = 0;
        visited_ = 0;
    }

    void addNode(point_type p)
    {
        node n(p);
        n.pos_index = nodes_.size();
        nodes_.emplace_back(n);
    }

    void generate_tree()
    {
        root_ = make_tree(0, nodes_.size() - 1, 0);
    }
    /**
     * Constructor taking a pair of iterators. Adds each
     * point in the range [begin, end) to the tree.
     *
     * @param begin start of range
     * @param end end of range
     */
    template<typename iterator>
    KDTree(iterator begin, iterator end)
    {
        best_ = nullptr;
        best_dist_ = 0;
        visited_ = 0;
        nodes_.reserve(std::distance(begin, end));
        for (auto i = begin; i != end; ++i)
        {
            addNode(*i);
        }
        generate_tree();
    }

    /**
     * Constructor taking a function object that generates
     * points. The function object will be called n times
     * to populate the tree.
     *
     * @param f function that returns a point
     * @param n number of points to add
     */
    template<typename func>
    KDTree(func&& f, size_t n)
    {
        best_ = nullptr;
        best_dist_ = 0;
        visited_ = 0;
        nodes_.reserve(n);
        for (size_t i = 0; i < n; ++i)
            addNode(f());
        generate_tree();
    }

    /**
     * Returns true if the tree is empty, false otherwise.
     */
    bool empty() const
    {
        return nodes_.empty();
    }

    /**
     * Returns the number of nodes visited by the last call
     * to nearest().
     */
    size_t visited() const
    {
        return visited_;
    }

    /**
     * Returns the distance between the input point and return value
     * from the last call to nearest().
     */
    float distance() const
    {
        return std::sqrt(best_dist_);
    }

    /**
     * Finds the nearest point in the tree to the given point.
     * It is not valid to call this function if the tree is empty.
     *
     * @param pt a point
     * @param the nearest point in the tree to the given point
     */
    const void nearest(const point_type& pt, int k)
    {
        if (root_ == nullptr)
            throw std::logic_error("tree is empty");
        best_ = nullptr;
        best_dist_ = 0;
        visited_ = 0;
        m_vNbrNodes.clear();
        nearest(root_, pt, 0, k);
    }

    /**
     * Finds the nearest point in the tree to the given point.
     * It is not valid to call this function if the tree is empty.
     *
     * @param pt a point
     * @return the nearest point in the tree to the given point
     */
    const point_type& nearest(const point_type& pt) {
        if (root_ == nullptr)
            throw std::logic_error("tree is empty");
        best_ = nullptr;
        visited_ = 0;
        best_dist_ = 0;
        nearest(root_, pt, 0);
        return best_->point_;
    }

    /**
     * Finds the nearest point in the tree to the given point by radius
     * It is not valid to call this function if the tree is empty.
     *
     * @param pt a point
     * @param the nearest point in the tree to the given point
     */
    const void nearest_radius(const point_type& pt, float radius)
    {
        if (root_ == nullptr)
            throw std::logic_error("tree is empty");
        m_vNbrNodes.clear();
        visited_ = 0;
        nearest_radius(root_, pt, 0, radius);
    }

    VNodeType m_vNbrNodes;
};

typedef KDTree<float, 2> KDT2D;
typedef KDTree<float, 3> KDT3D;
