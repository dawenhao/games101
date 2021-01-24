#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

int main()
{
     // Basic Example of cpp
     std::cout << "Example of cpp \n";
     float a = 1.0, b = 2.0;
     std::cout << a << std::endl;
     std::cout << a / b << std::endl;
     std::cout << std::sqrt(b) << std::endl;
     std::cout << std::acos(-1) << std::endl;
     std::cout << std::sin(30.0 / 180.0 * acos(-1)) << std::endl;

     // Example of vector
     std::cout << "Example of vector \n";
     // vector definition
     Eigen::Vector3f v(1.0f, 1.0f, 3.0f);
     Eigen::Vector3f w(1.0f, 1.0f, 0.0f);
     // vector output
     std::cout << "Example of output \n";
     std::cout << v << std::endl;
     // vector add
     std::cout << "Example of add \n";
     std::cout << v + w << std::endl;
     //vector scalar multiply
     std::cout << "Example of scalar multiplay \n";
     std::cout << v * 3.0f << std::endl;
     std::cout << 2.0f * v << std::endl;

     // Example of matrix
     std::cout << "Example of matrix \n";

     return 0;
}