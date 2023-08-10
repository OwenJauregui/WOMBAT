#include "slam/kd_tree.h"
#include <iostream>

int main() {

    Eigen::MatrixX2d a(13, 2);

    a << 2, 3,
         -9, 10,
         -4, -2,
         5, -4,
         -8, 2,
         2, 6,
         -4, 4,
         -6, 8,
         -2, 3,
         -2, 7,
         -2.5, 4,
         0, 8,
         -3, 2;
    
    KD_Tree points(a);

    Eigen::RowVector2d point(1, 2);

    point << -3, 6;
    
    std::cout << points.closest_point(point);
    
    return 0;
}
