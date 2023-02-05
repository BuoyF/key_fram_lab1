#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include "quaternion.h"


Eigen::Matrix4d getCatmullRomMat();

Eigen::Matrix4d getBMat();

Eigen::Matrix3d rotateAroundX(double angle);

Eigen::Matrix3d rotateAroundY(double angle);

Eigen::Matrix3d rotateAroundZ(double angle);

Eigen::Matrix3d rotateMatrix(double alpha, double beta, double gamma, std::string& rtype);

std::vector<Eigen::Vector3d> getPosition(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, std::vector<double> t, int intertype);
std::vector<Eigen::Vector3d> getPosition(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double start, double end, double stride, int intertype);

double dot(QuaternionS q1, QuaternionS q2);
std::vector<QuaternionS> NLerp(QuaternionS q1, QuaternionS q2, double dt);
std::vector<QuaternionS> SLerp(QuaternionS q1, QuaternionS q2, double dt);
