#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  ROS_WARN("POLYQPGeneration BEGIN");
  // FIXME
  //  enforce initial and final velocity and accleration, for higher order
  //  derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff(m, 3 * p_num1d); // coefficent
  VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);
  VectorXd T1 = Time;
  VectorXd T2 = T1.cwiseProduct(T1);
  VectorXd T3 = T2.cwiseProduct(T1);
  VectorXd T4 = T2.cwiseProduct(T2);
  VectorXd T5 = T4.cwiseProduct(T1);
  VectorXd T6 = T4.cwiseProduct(T2);
  VectorXd T7 = T4.cwiseProduct(T3);
  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  // d_order = 4  ---minimum-snap;
  // d_order -- s
  // p_order -- N=2s-1
  // p_num1d -- 2s

  // enforce initial and final position,velocity and accleration, for higher order derivatives, just assume them be 0
  VectorXd startPos = Path.row(0), startVel = Vel.row(0), startAcc = Acc.row(0);
  VectorXd terminalPos = Path.row(Path.rows() - 1), terminalVel = Vel.row(1), terminalAcc = Acc.row(1);

  MatrixXd coeff = MatrixXd::Zero(m * p_num1d, 3);
  MatrixXd M = MatrixXd::Zero(m * p_num1d, m * p_num1d);
  MatrixXd b = MatrixXd::Zero(m * p_num1d, 3);
  //minco
  M(0, 0) = 1.0;
  M(1, 1) = 1.0;
  M(2, 2) = 2.0;
  M(3, 3) = 6.0;
  b.row(0) = Path.row(0);
  b.row(1) = Vel.row(0);
  b.row(2) = Acc.row(0);
  
  for (int i = 0; i < m - 1; i++)
  {
    M(8 * i + 4, 8 * i + 4) = 24.0;
    M(8 * i + 4, 8 * i + 5) = 120.0 * T1(i);
    M(8 * i + 4, 8 * i + 6) = 360.0 * T2(i);
    M(8 * i + 4, 8 * i + 7) = 840.0 * T3(i);
    M(8 * i + 4, 8 * i + 12) = -24.0;
    M(8 * i + 5, 8 * i + 5) = 120.0;
    M(8 * i + 5, 8 * i + 6) = 720.0 * T1(i);
    M(8 * i + 5, 8 * i + 7) = 2520.0 * T2(i);
    M(8 * i + 5, 8 * i + 13) = -120.0;
    M(8 * i + 6, 8 * i + 6) = 720.0;
    M(8 * i + 6, 8 * i + 7) = 5040.0 * T1(i);
    M(8 * i + 6, 8 * i + 14) = -720.0;
    M(8 * i + 7, 8 * i) = 1.0;
    M(8 * i + 7, 8 * i + 1) = T1(i);
    M(8 * i + 7, 8 * i + 2) = T2(i);
    M(8 * i + 7, 8 * i + 3) = T3(i);
    M(8 * i + 7, 8 * i + 4) = T4(i);
    M(8 * i + 7, 8 * i + 5) = T5(i);
    M(8 * i + 7, 8 * i + 6) = T6(i);
    M(8 * i + 7, 8 * i + 7) = T7(i);
    M(8 * i + 8, 8 * i) = 1.0;
    M(8 * i + 8, 8 * i + 1) = T1(i);
    M(8 * i + 8, 8 * i + 2) = T2(i);
    M(8 * i + 8, 8 * i + 3) = T3(i);
    M(8 * i + 8, 8 * i + 4) = T4(i);
    M(8 * i + 8, 8 * i + 5) = T5(i);
    M(8 * i + 8, 8 * i + 6) = T6(i);
    M(8 * i + 8, 8 * i + 7) = T7(i);
    M(8 * i + 8, 8 * i + 8) = -1.0;
    M(8 * i + 9, 8 * i + 1) = 1.0;
    M(8 * i + 9, 8 * i + 2) = 2.0 * T1(i);
    M(8 * i + 9, 8 * i + 3) = 3.0 * T2(i);
    M(8 * i + 9, 8 * i + 4) = 4.0 * T3(i);
    M(8 * i + 9, 8 * i + 5) = 5.0 * T4(i);
    M(8 * i + 9, 8 * i + 6) = 6.0 * T5(i);
    M(8 * i + 9, 8 * i + 7) = 7.0 * T6(i);
    M(8 * i + 9, 8 * i + 9) = -1.0;
    M(8 * i + 10, 8 * i + 2) = 2.0;
    M(8 * i + 10, 8 * i + 3) = 6.0 * T1(i);
    M(8 * i + 10, 8 * i + 4) = 12.0 * T2(i);
    M(8 * i + 10, 8 * i + 5) = 20.0 * T3(i);
    M(8 * i + 10, 8 * i + 6) = 30.0 * T4(i);
    M(8 * i + 10, 8 * i + 7) = 42.0 * T5(i);
    M(8 * i + 10, 8 * i + 10) = -2.0;
    M(8 * i + 11, 8 * i + 3) = 6.0;
    M(8 * i + 11, 8 * i + 4) = 24.0 * T1(i);
    M(8 * i + 11, 8 * i + 5) = 60.0 * T2(i);
    M(8 * i + 11, 8 * i + 6) = 120.0 * T3(i);
    M(8 * i + 11, 8 * i + 7) = 210.0 * T4(i);
    M(8 * i + 11, 8 * i + 11) = -6.0;

    b.row(8 * i + 7) = Path.row(i + 1);
  }
  M(8 * m - 4, 8 * m - 8) = 1.0;
  M(8 * m - 4, 8 * m - 7) = T1(m - 1);
  M(8 * m - 4, 8 * m - 6) = T2(m - 1);
  M(8 * m - 4, 8 * m - 5) = T3(m - 1);
  M(8 * m - 4, 8 * m - 4) = T4(m - 1);
  M(8 * m - 4, 8 * m - 3) = T5(m - 1);
  M(8 * m - 4, 8 * m - 2) = T6(m - 1);
  M(8 * m - 4, 8 * m - 1) = T7(m - 1);
  M(8 * m - 3, 8 * m - 7) = 1.0;
  M(8 * m - 3, 8 * m - 6) = 2.0 * T1(m - 1);
  M(8 * m - 3, 8 * m - 5) = 3.0 * T2(m - 1);
  M(8 * m - 3, 8 * m - 4) = 4.0 * T3(m - 1);
  M(8 * m - 3, 8 * m - 3) = 5.0 * T4(m - 1);
  M(8 * m - 3, 8 * m - 2) = 6.0 * T5(m - 1);
  M(8 * m - 3, 8 * m - 1) = 7.0 * T6(m - 1);
  M(8 * m - 2, 8 * m - 6) = 2.0;
  M(8 * m - 2, 8 * m - 5) = 6.0 * T1(m - 1);
  M(8 * m - 2, 8 * m - 4) = 12.0 * T2(m - 1);
  M(8 * m - 2, 8 * m - 3) = 20.0 * T3(m - 1);
  M(8 * m - 2, 8 * m - 2) = 30.0 * T4(m - 1);
  M(8 * m - 2, 8 * m - 1) = 42.0 * T5(m - 1);
  M(8 * m - 1, 8 * m - 5) = 6.0;
  M(8 * m - 1, 8 * m - 4) = 24.0 * T1(m - 1);
  M(8 * m - 1, 8 * m - 3) = 60.0 * T2(m - 1);
  M(8 * m - 1, 8 * m - 2) = 120.0 * T3(m - 1);
  M(8 * m - 1, 8 * m - 1) = 210.0 * T4(m - 1);

  b.row(8 * m - 4) = terminalPos;
  b.row(8 * m - 3) = terminalVel;
  b.row(8 * m - 2) = terminalAcc;
  // b.row(8 * m - 1) = tailPVAJ.col(3).transpose();
  // boundary value Matrix
  // MatrixXd F0 = MatrixXd::Zero(d_order, p_num1d),
  //          EM = MatrixXd::Zero(d_order, p_num1d);
  // F0 << 1, 0, 0, 0, 0, 0, 0, 0,
  //     0, 1, 0, 0, 0, 0, 0, 0,
  //     0, 0, 2, 0, 0, 0, 0, 0,
  //     0, 0, 0, 6, 0, 0, 0, 0;
  // double tM = Time(m - 1);
  // EM << 1, tM, pow(tM, 2), pow(tM, 3), pow(tM, 4), pow(tM, 5), pow(tM, 6), pow(tM, 7),
  //     0, 1, 2 * tM, 3 * pow(tM, 2), 4 * pow(tM, 3), 5 * pow(tM, 4), 6 * pow(tM, 5), 7 * pow(tM, 6),
  //     0, 0, 2, 6 * tM, 12 * tM * tM, 20 * pow(tM, 3), 30 * pow(tM, 4), 42 * pow(tM, 5),
  //     0, 0, 0, 6, 24 * tM, 60 * tM * tM, 120 * pow(tM, 3), 210 * pow(tM, 4);

  // M.block(0, 0, d_order, p_num1d) = F0;
  // b.block(0, 0, d_order, 3) << startPos(0), startPos(1), startPos(2),
  //     startVel(0), startVel(1), startVel(2),
  //     startAcc(0), startAcc(1), startAcc(2),
  //     0, 0, 0;
  // M.block(m * p_num1d - d_order, m * p_num1d - p_num1d, d_order, p_num1d) = EM;
  // b.block(m * p_num1d - d_order, 0, d_order, 3) << terminalPos(0), terminalPos(1), terminalPos(2),
  //     terminalVel(0), terminalVel(1), terminalVel(2),
  //     terminalAcc(0), terminalAcc(1), terminalAcc(2),
  //     0, 0, 0;

  // for (int i = 0; i < m - 1; i++)
  // {
  //   MatrixXd Ei = MatrixXd::Zero(p_num1d, p_num1d), Fi = MatrixXd::Zero(p_num1d, p_num1d);
  //   double t = Time(i);
  //   Ei << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6), pow(t, 7),
  //       1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6), pow(t, 7),
  //       0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4), 6 * pow(t, 5), 7 * pow(t, 6),
  //       0, 0, 2, 6 * t, 12 * t * t, 20 * pow(t, 3), 30 * pow(t, 4), 42 * pow(t, 5),
  //       0, 0, 0, 6, 24 * t, 60 * t * t, 120 * pow(t, 3), 210 * pow(t, 4),
  //       0, 0, 0, 0, 24, 120 * t, 360 * pow(t, 2), 840 * pow(t, 3),
  //       0, 0, 0, 0, 0, 120, 720 * t, 2520 * pow(t, 2),
  //       0, 0, 0, 0, 0, 0, 720, 5040 * t;
  //   M.block(p_num1d * i + d_order, p_num1d * i, p_num1d, p_num1d) = Ei;
  //   Fi << 0, 0, 0, 0, 0, 0, 0, 0,
  //       -1, 0, 0, 0, 0, 0, 0, 0,
  //       0, -1, 0, 0, 0, 0, 0, 0,
  //       0, 0, -2, 0, 0, 0, 0, 0,
  //       0, 0, 0, -6, 0, 0, 0, 0,
  //       0, 0, 0, 0, -24, 0, 0, 0,
  //       0, 0, 0, 0, 0, -120, 0, 0,
  //       0, 0, 0, 0, 0, 0, -720, 0;
  //   M.block(p_num1d * i + d_order, p_num1d * i + p_num1d, p_num1d, p_num1d) = Fi;
  //   b.block(p_num1d * i + 3, 0, 1, 3) = Path.row(i + 1);
  // }
  coeff = M.lu().solve(b);
  // for (int i=0;i<3;i++){
  //         coeff.col(i)=M.lu().solve(b.col(i));
  //  }
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      VectorXd mVec = coeff.block(i * p_num1d, j, p_num1d, 1);
      PolyCoeff.block(i, j * p_num1d, 1, p_num1d) = mVec.transpose();
    }
  }
  ROS_WARN("POLYQPGeneration END");
  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective()
{
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t)
{
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++)
  {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t)
{
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++)
  {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t)
{
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++)
  {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}