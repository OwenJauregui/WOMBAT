#include "slam/kd_tree.h"
#include <iostream>

KD_Tree::KD_Tree()
{
    this->head = nullptr;
}

KD_Tree::KD_Tree(const Eigen::MatrixX2d& b)
{
    this->head = nullptr;
    this->set_tree(b);
}

KD_Tree::~KD_Tree()
{   
    // Start deleting process
    this->delete_tree(this->head);
}

bool KD_Tree::initialized()
{
    return this->init;
}

void KD_Tree::set_tree(const Eigen::MatrixX2d& b)
{
    this->init = true;
        
    if(this->head != nullptr) this->delete_tree(this->head);

    // Copy point cloud
    this->points = b;

    // Generate indices
    std::vector<int> indices(b.rows());
    std::iota(indices.begin(), indices.end(), 0);

    // Generate tree
    this->head = this->create_tree(indices.begin(), indices.end(), 0);
}

void KD_Tree::delete_tree(kd_node* node)
{
    if(node != nullptr) {
        // Delete all child nodes
        this->delete_tree(node->left);
        this->delete_tree(node->right);
        
        // Delete current node
        delete(node);
    }
}

kd_node* KD_Tree::create_tree(const std::vector<int>::iterator& start, const std::vector<int>::iterator& end, int level)
{
    // Find the median for the current set of points
    int iter_diff = static_cast<int>(end - start) - 1;
    int median    = iter_diff/2;

    // Declare current node
    kd_node* current;

    if(iter_diff > 0) {
        // Sort the current set of points by the axis "level"
        this->sort_points(start, end, level);
        std::vector<int>::iterator median_it = start + median;

        // Assing values to current node and create childs
        current = this->new_node(*median_it);
        int new_level = (level+1) % 2;
        
        // If median is 0 we can be sure there is only 2 elements
        if (median != 0) {
            current->left  = this->create_tree(start, median_it, new_level);
            current->right = this->create_tree(median_it + 1, end, new_level);
        } else {
            current->right = this->new_node(*(end-1));        
        }
    
    } else {
        // Only one element left
        current = this->new_node(*start);
    }

    return current;
}

kd_node* KD_Tree::new_node(int index)
{
    kd_node* new_node = new kd_node();
    new_node->index = index;
    new_node->left  = nullptr; 
    new_node->right = nullptr;

    return new_node;
}

void KD_Tree::sort_points(const std::vector<int>::iterator& start, const std::vector<int>::iterator& end, int axis)
{
    std::sort(start, end, [this, axis](int a, int b) {
        return compare_nodes(a, b, axis);
    });
}

bool KD_Tree::compare_nodes(int a, int b, int axis)
{
    return this->points(a, axis) < this->points(b, axis);
}

Eigen::RowVector2d KD_Tree::closest_point(const Eigen::RowVector2d& point)
{
    Eigen::RowVector2d output;

    if (this->head != nullptr) {
        // Look for the closest point
        int recs = 0;
        kd_node* closest = this->search(this->head, point, 0, recs);

        std::cout << "Recursions: " <<recs<< std::endl;

        // Return index and distance
        output(0, 0) = closest->index;
        output(0, 1) = utils::euclidean_distance(point, this->points.row(closest->index));
    }

    return output;
}

kd_node* KD_Tree::closest(const Eigen::RowVector2d& point, kd_node* a, kd_node* b)
{
    // Check if any of the nodes is null

    if(a == nullptr) return b;
    if(b == nullptr) return a;

    // Calculate distance to nodes
    double dist_a, dist_b;
    
    dist_a = utils::euclidean_distance(point, this->points.row(a->index));
    dist_b = utils::euclidean_distance(point, this->points.row(b->index));

    // Return closest point
    if (dist_a > dist_b) {
        return b;
    } else {
        return a;
    }
}

kd_node* KD_Tree::search(kd_node* node, const Eigen::RowVector2d& point, int level, int& rec)
{
    if (node == nullptr) return nullptr;

    rec++;

    // Set next level
    int next_level = level + 1;
    int axis       = level%2; 

    // Create auxiliar node pointers
    kd_node *next, *other;    

    if(point(0, axis) >= this->points(node->index, axis)) {
        next  = node->right;
        other = node->left;
    } else {
        other = node->right;
        next  = node->left;
    }

    // Recurse on next branch
    kd_node* temp = search(next, point, next_level, rec);
    kd_node* best = closest(point, temp, node); 

    unsigned long radius = utils::euclidean_distance(point, this->points.row(best->index));
    unsigned long dist   = point(0, axis) - this->points(node->index, axis);   
    
    if(radius >= dist) {
        // Recurse on other branch and update radius
        temp = this->search(other, point, next_level, rec);
        best = closest(point, temp, best);
    }

    return best;
}
