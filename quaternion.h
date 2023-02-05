#pragma once
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

class QuaternionS
{
public:
    double w, x, y, z;
    QuaternionS();
    QuaternionS(double w_, double x_, double y_, double z_);
    QuaternionS(Eigen::Vector3d axis, double angle);

    QuaternionS operator*(const QuaternionS& q);
    QuaternionS operator*=(const QuaternionS& q);
    QuaternionS operator+(const QuaternionS& q);
    QuaternionS operator*(double t);

    friend std::ostream& operator<<(std::ostream& output, const QuaternionS& q)
    {
        output << q.w << "+" << q.x << "i+" << q.y << "j+" << q.z << "k";
        return output;
    }

    double norm();
    double norm(const QuaternionS& q);
    void normalize();
    void conjugate();

    Eigen::Matrix3d toMatrix3d();
    Eigen::Matrix4d toMatrix4d();

    QuaternionS fromFixedXYZ(double alpha, double beta, double gamma);
    Eigen::Vector3d toFixedXYZ();
    Eigen::Vector4d toAxisAngle();
};