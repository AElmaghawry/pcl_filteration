#include <iostream>
#include <Eigen/Dense>

int main()
{
    // 0.00743252, -0.781734,0.860328
    // -0.68065694, -0.02066882,  0.73231067
    Eigen::Vector3d v(0.00743252, -0.781734, 0.860328);
    Eigen::Vector3d w(-0.68065694, -0.02066882, 0.73231067);
    Eigen::Vector3d q(-0.929, -0.02821, 0.9995);
    std::cout << "Dot product: " << v.dot(w) << std::endl;
    double dp = v.adjoint() * w; // automatic conversion of the inner product to a scalar
    std::cout << "Dot product via a matrix product: " << dp << std::endl;
    std::cout << "Cross product:\n"
              << v.cross(w) << std::endl;
    std::cout << "The normalized vector" << (q.normalized()).transpose() << std::endl ; 
}
