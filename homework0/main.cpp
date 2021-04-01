#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

int main()
{

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << "a:" << a << std::endl;
    std::cout << "a/b:" << a / b << std::endl;
    std::cout << "sqrt(b):" << std::sqrt(b) << std::endl;
    std::cout << "acos(-1):" << std::acos(-1) << std::endl;
    // C++ 中的正弦值需要使用弧度
    std::cout << "sin(30.0 / 180.0 * acos(-1)):" << std::sin(30.0 / 180.0 * acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f w(1.0f, 0.0f, 0.0f);

    std::cout << "向量V\n"
              << v << "\n"
              << "向量W\n"
              << w << "\n"
              << std::endl;

    // vector add
    std::cout << "Example of add \n";
    std::cout << "v+w:\n"
              << v + w << std::endl;

    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << "v*3\n"
              << v * 3.0f << std::endl;
    std::cout << "2*v\n"
              << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i, j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output  I矩阵 \n";
    std::cout << i << "\n"
              << std::endl;

    std::cout << "Example of output  J矩阵 \n";
    std::cout << j << "\n"
              << std::endl;

    // matrix add i + j
    Eigen::Matrix3f iPlusj = i + j;
    std ::cout << "I矩阵加J矩阵:\n"
               << iPlusj << "\n"
               << std::endl;

    // matrix scalar multiply i * 2.0
    Eigen::Matrix3f iMul2 = i * 2.0f;
    std ::cout << "I矩阵×2:\n"
               << iMul2 << "\n"
               << std::endl;

    // matrix multiply i * j

    Eigen::Matrix3f iMulj = i * j;
    std::cout << "I矩阵 × J矩阵\n"
              << iMulj << "\n"
              << std::endl;

    // matrix multiply vector i * v
    Eigen::Vector3f iMulv = i * v;
    std::cout << "I矩阵 × v向量\n"
              << iMulv << "\n"
              << std::endl;

    // 给定一个点P=(2,1)，将该点先逆时针旋转45度，再平移(1,2),计算出变换后的坐标（要求用齐次坐标进行计算）
    Eigen::Vector3f p(2.0f, 1.0f, 1.0f); // 齐次坐标 点 第三个是1
    Eigen::Matrix3f tranMatrix;
    tranMatrix << 1, 0, 1, 0, 1, 2, 0, 0, 1;

    Eigen::Matrix3f rotMatrix;
    float theta = 45.0 / 180.0 * acos(-1); // acos(-1) = pi
    float cos45 = std::cos(theta);
    float sin45 = std::sin(theta);
    rotMatrix << cos45, -sin45, 0, sin45, cos45, 0, 0, 0, 1;
    Eigen::Vector3f result = tranMatrix * rotMatrix * p;
    std ::cout << "变换P点的结果\n"
               << result << std::endl;
    return 0;
}