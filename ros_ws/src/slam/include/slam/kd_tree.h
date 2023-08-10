#ifndef KD_TREE_H
#define KD_TREE_H

#include "slam/utils.h"

#include <Eigen/Core>
#include <vector>
#include <numeric>

struct kd_node {

    int index; // Index in point cloud
    
    // Node's children
    kd_node* left;
    kd_node* right;
    
};

class KD_Tree {
    
    private:
    
        // Set of point with indices
        Eigen::MatrixX2d points;
        
        // Start of tree
        kd_node* head;
        bool init;    

    public: 
        KD_Tree();
        KD_Tree(const Eigen::MatrixX2d& b);
        ~KD_Tree();

        bool initialized();

        void delete_tree(kd_node* node);
        void set_tree(const Eigen::MatrixX2d& b);

        kd_node* create_tree(const std::vector<int>::iterator& start, const std::vector<int>::iterator& end, int level);

        kd_node* new_node(int index);
        void sort_points(const std::vector<int>::iterator& start, const std::vector<int>::iterator& end, int axis);
        bool compare_nodes(int a, int b, int axis);        

        Eigen::RowVector2d closest_point(const Eigen::RowVector2d& point);    
        kd_node* closest(const Eigen::RowVector2d& point, kd_node* a, kd_node* b);
        kd_node* search(kd_node* node, const Eigen::RowVector2d& point, int level, int& rec);
};

#endif
