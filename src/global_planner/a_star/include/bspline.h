#ifndef BSPLINE_H
#define BSPLINE_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;
using namespace Eigen;

double baseFunction(int i, int k, double u, vector<double>node_vector);

vector<double> u_quasi_uniform(int n,int k);//准均匀b样条


vector<double> u_piecewise_B_Spline(int n,int k);

std::vector<Eigen::Vector2d> getBspline(std::vector<Eigen::Vector2d> roarse_path);



#endif //CHHROBOTICS_CPP_BSPLINE_H