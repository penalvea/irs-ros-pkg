#ifndef TaskVelocity_HPP
#define TaskVelocity_HPP

#include <vector>
#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>


class CartesianTaskVelocity{
public:
  CartesianTaskVelocity();
  ~CartesianTaskVelocity();
  Eigen::MatrixXd calculateCartesianVelocity(Eigen::MatrixXd current, Eigen::MatrixXd goal);

};
typedef boost::shared_ptr<CartesianTaskVelocity> CartesianTaskVelocityPtr;


#endif // TaskVelocity_HPP
