#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <iostream>

using namespace Eigen;
using namespace std;

int main() {
    // problem 1
    Vector4f v(1, 1.5, 2, 3);
    Matrix4f i, j;
    i << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
    j << 4, 3, 2, 1, 8, 7, 6, 5, 12, 11, 10, 9, 16, 15, 14, 13;
    // problem 2
    cout << "[problem 2]" << endl;
    cout << i + j << endl;
    // problem 3
    cout << "[problem 3]" << endl;
    cout << i * j << endl;
    // problem 4
    cout << "[problem 4]" << endl;
    cout << i * v << endl;
    return 0;
}