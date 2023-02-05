#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include "interpolation.h"
#include "quaternion.h"

// M matrix of CatmullRom interpolation
Eigen::Matrix4d getCatmullRomMat()
{
    Eigen::Matrix4d M_CR;
    M_CR << -0.5, 1.5, -1.5, 0.5, 
        1.0, -2.5, 2.0, -0.5, 
        -0.5, 0, 0.5, 0, 
        0, 1.0, 0, 0;
    return M_CR;
}

// M matrix of B interpolation
Eigen::Matrix4d getBMat()
{
    Eigen::Matrix4d M_B;
    M_B << -1.0 / 6.0, 0.5, -0.5, 1.0 / 6.0, 
        0.5, -1.0, 0.5, 0, 
        -0.5, 0, 0.5, 0, 
        1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0, 0;
    return M_B;
}

// rotation matrix around axis X
Eigen::Matrix3d rotateAroundX(double angle)
{
    Eigen::Matrix3d M_X;
    M_X << 1.0, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle);
    return M_X;
}

// rotation matrix around axis Y
Eigen::Matrix3d rotateAroundY(double angle)
{
    Eigen::Matrix3d M_Y;
    M_Y << cos(angle), 0, sin(angle), 0, 1.0, 0, -sin(angle), 0, cos(angle);
    return M_Y;
}

// rotation matrix around axis Z
Eigen::Matrix3d rotateAroundZ(double angle)
{
    Eigen::Matrix3d M_Z;
    M_Z << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1.0;
    return M_Z;
}

// rotation matrix from fixed angles
Eigen::Matrix3d rotateMatrix(double alpha, double beta, double gamma, std::string& rtype)
{
    Eigen::Matrix3d M1, M2, M3;
    if (rtype == "XYX")
    {
        M1 = rotateAroundX(alpha);
        M2 = rotateAroundY(beta);
        M3 = rotateAroundX(gamma);
    }
    else if (rtype == "XZY")
    {
        M1 = rotateAroundX(alpha);
        M2 = rotateAroundZ(beta);
        M3 = rotateAroundY(gamma);
    }
    else if (rtype == "XZX")
    {
        M1 = rotateAroundX(alpha);
        M2 = rotateAroundZ(beta);
        M3 = rotateAroundX(gamma);
    }
    else if (rtype == "YZX")
    {
        M1 = rotateAroundY(alpha);
        M2 = rotateAroundZ(beta);
        M3 = rotateAroundX(gamma);
    }
    else if (rtype == "YZY")
    {
        M1 = rotateAroundY(alpha);
        M2 = rotateAroundZ(beta);
        M3 = rotateAroundY(gamma);
    }
    else if (rtype == "YXZ")
    {
        M1 = rotateAroundY(alpha);
        M2 = rotateAroundX(beta);
        M3 = rotateAroundZ(gamma);
    }
    else if (rtype == "YXY")
    {
        M1 = rotateAroundY(alpha);
        M2 = rotateAroundX(beta);
        M3 = rotateAroundY(gamma);
    }
    else if (rtype == "ZXY")
    {
        M1 = rotateAroundZ(alpha);
        M2 = rotateAroundX(beta);
        M3 = rotateAroundY(gamma);
    }
    else if (rtype == "ZXZ")
    {
        M1 = rotateAroundZ(alpha);
        M2 = rotateAroundX(beta);
        M3 = rotateAroundZ(gamma);
    }
    else if (rtype == "ZYX")
    {
        M1 = rotateAroundZ(alpha);
        M2 = rotateAroundY(beta);
        M3 = rotateAroundX(gamma);
    }
    else if (rtype == "ZYZ")
    {
        M1 = rotateAroundZ(alpha);
        M2 = rotateAroundY(beta);
        M3 = rotateAroundZ(gamma);
    }
    else
    {
        M1 = rotateAroundX(alpha);
        M2 = rotateAroundY(beta);
        M3 = rotateAroundZ(gamma);
    }
    Eigen::Matrix3d rotateMat = M1 * M2 * M3;
    return rotateMat;
}

// interpolate positions
std::vector<Eigen::Vector3d> getPosition(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, std::vector<double> t, int intertype)
{
    Eigen::MatrixXd P(4, 3);
    P << p0.transpose(),
        p1.transpose(),
        p2.transpose(),
        p3.transpose();
    Eigen::Matrix4d M;
    if (intertype == 0)
    {
        M = getCatmullRomMat();
    }
    else
    {
        M = getBMat();
    }
    std::vector<Eigen::Vector3d> positions;
    for (int i = 0; i < t.size(); i++)
    {
        Eigen::Vector4d T;
        T << t[i] * t[i] * t[i], t[i] * t[i], t[i], 1;
        positions.push_back(T.transpose() * M * P);
    }
    return positions;
}

// interpolate positions
std::vector<Eigen::Vector3d> getPosition(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double start, double end, double stride, int intertype)
{
    Eigen::MatrixXd P(4, 3);
    P << p0.transpose(),
        p1.transpose(),
        p2.transpose(),
        p3.transpose();
    Eigen::Matrix4d M;
    if (intertype == 0)
    {
        M = getCatmullRomMat();
    }
    else
    {
        M = getBMat();
    }
    std::vector<Eigen::Vector3d> positions;
    for (double t = start; t <= end; t += stride)
    {
        Eigen::Vector4d T;
        T << t * t * t, t * t, t, 1;
        positions.push_back(T.transpose() * M * P);
    }
    return positions;
}

// B Spline interpolation
void BSplineInterpotation(std::vector<Eigen::Vector3d> ctrl_pnts, double stride)
{

}

// Normalized Linear Interpolation
std::vector<QuaternionS> NLerp(QuaternionS q1, QuaternionS q2, double dt)
{
    std::vector<QuaternionS> Qs;
    for (double t = 0; t <= 1; t += dt)
    {
        QuaternionS q = q1 * (1 - t) + q2 * t;
        //q.normalize();
        Qs.push_back(q);
    }
    return Qs;
}

double dot(QuaternionS q1, QuaternionS q2)
{
    return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z + q2.z;
}

// Spherical linear interpolation
std::vector<QuaternionS> SLerp(QuaternionS q1, QuaternionS q2, double dt)
{
    double theta = acos(dot(q1, q2));
    if (theta < 0.1)
    {
        return NLerp(q1, q2, dt);
    }
    if (theta < 0)
    {
        q2.conjugate();
    }
    std::vector<QuaternionS> Qs;
    for (double t = 0; t <= 1; t += dt)
    {
        QuaternionS q = q1 * (sin((1 - t) * theta) / sin(theta)) + q2 * ((sin(t * theta) / sin(theta)));
        //q.normalize();
        Qs.push_back(q);
    }
    return Qs;
}

/* for test
int main()
{
    Vector3d p0, p1, p2, p3;
    p0 << 1, 2, 3;
    p1 << 2, 3, 4;
    p2 << 3, 4, 5;
    p3 << 4, 5, 6;
    std::cout << getPosition(1.0, p0, p1, p2, p3, "CatmulRom");
    return 0;
}
*/