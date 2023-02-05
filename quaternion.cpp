#include <cmath>
#include <vector>
#include <iostream>
#include "quaternion.h"
#define PI 3.1415926

// initiate the quaternion
QuaternionS::QuaternionS(): w(1), x(0), y(0), z(0) {}
QuaternionS::QuaternionS(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
QuaternionS::QuaternionS(Eigen::Vector3d axis, double angle)
{
    double length = sqrt(axis(0) * axis(0) + axis(1) * axis(1) + axis(2) * axis(2));
    if (length > 0)
    {
        this->x = axis(0) / length * sin(angle * 0.5);
        this->y = axis(1) / length * sin(angle * 0.5);
        this->z = axis(2) / length * sin(angle * 0.5);
    }
    else
    {
        this->x = 0;
        this->y = 0;
        this->z = 0;
    }
    this->w = cos(angle * 0.5);
}

// overload the operator* with another quaternion
QuaternionS QuaternionS::operator*(const QuaternionS& q)
{
    QuaternionS p;
    p.x = this->w * q.x + this->x * q.w + this->y * q.z - this->z * q.y;
    p.y = this->w * q.y - this->x * q.z + this->y * q.w + this->z * q.x;
    p.z = this->w * q.z + this->x * q.y - this->y * q.x + this->z * q.w;
    p.w = this->w * q.w - this->x * q.x - this->y * q.y - this->z * q.z;
    return p;
}

// overload the operator*=
QuaternionS QuaternionS::operator*=(const QuaternionS& q)
{
    this->x = this->w * q.x + this->x * q.w + this->y * q.z - this->z * q.y;
    this->y = this->w * q.y - this->x * q.z + this->y * q.w + this->z * q.x;
    this->z = this->w * q.z + this->x * q.y - this->y * q.x + this->z * q.w;
    this->w = this->w * q.w - this->x * q.x - this->y * q.y - this->z * q.z;
    return *this;
}

// overload the operator+
QuaternionS QuaternionS::operator+(const QuaternionS& q)
{
    this->x = this->x + q.x;
    this->y = this->y + q.y;
    this->z = this->z + q.z;
    this->w = this->w + q.w;
    return *this;
}

// overload the operator* with a double
QuaternionS QuaternionS::operator*(double t)
{
    QuaternionS p;
    p.x = this->x * t;
    p.y = this->y * t;
    p.z = this->z * t;
    p.w = this->w * t;
    return p;
}

// get the norm
double QuaternionS::norm()
{
    return sqrt(this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w);
}

// get the norm
double QuaternionS::norm(const QuaternionS& q)
{
    return sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

// normalize the quaternion
void QuaternionS::normalize()
{
    double length = this->norm();
    this->x /= length;
    this->y /= length;
    this->z /= length;
    this->w /= length;
}

// get the conjugate
void QuaternionS::conjugate()
{
    this->x = -this->x;
    this->y = -this->y;
    this->z = -this->z;
}

// convert to a rotation matrix of 3x3
Eigen::Matrix3d QuaternionS::toMatrix3d()
{
    Eigen::Matrix3d rotateMat;
    rotateMat << 1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y,
                2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x,
                2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y;
}

// convert to a rotation matrix of 4x4
Eigen::Matrix4d QuaternionS::toMatrix4d()
{
    Eigen::Matrix4d rotateMat;
    rotateMat << 1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y, 0,
                2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x, 0,
                2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y, 0,
                0, 0, 0, 1;
}

// convert from fixed angles
QuaternionS QuaternionS::fromFixedXYZ(double alpha, double beta, double gamma)
{
    return QuaternionS(cos(alpha * 0.5) * cos(beta * 0.5) * cos(gamma * 0.5) + sin(alpha * 0.5) * sin(beta * 0.5) * sin(gamma * 0.5),
        sin(alpha * 0.5) * cos(beta * 0.5) * cos(gamma * 0.5) - cos(alpha * 0.5) * sin(beta * 0.5) * sin(gamma * 0.5),
        cos(alpha * 0.5) * sin(beta * 0.5) * cos(gamma * 0.5) + sin(alpha * 0.5) * cos(beta * 0.5) * sin(gamma * 0.5),
        cos(alpha * 0.5) * cos(beta * 0.5) * sin(gamma * 0.5) - sin(alpha * 0.5) * sin(beta * 0.5) * cos(gamma * 0.5));
}

// convert the quaternion to fixed angles
Eigen::Vector3d QuaternionS::toFixedXYZ()
{
    Eigen::Vector3d XYZ;
    XYZ << atan(2 * (w * x + y * z) / (1 - 2 * (x * x + y * y))),
        asin(2 * (w * y - x * z)),
        atan(2 * (w * z + x * y) / (1 - 2 * (y * y + z * z)));
    return XYZ;
}

// convert the quaternion to (angle, axis)
Eigen::Vector4d QuaternionS::toAxisAngle()
{
    double theta = 2 * acos(w), n_x = 1, n_y = 1, n_z = 1;
    if (theta > 0.01)
    {
        n_x = x / sin(theta * 0.5);
        n_y = y / sin(theta * 0.5);
        n_z = z / sin(theta * 0.5);
    }
    Eigen::Vector4d AxisAngle;
    AxisAngle << theta, n_x, n_y, n_z;
    return AxisAngle;
}
