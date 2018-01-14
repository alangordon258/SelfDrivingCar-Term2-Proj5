#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
using CppAD::AD;
typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

using namespace std;
enum CostFunctionParts { CTE, EPSI, REF_VEL, STEERING_ACT,THROTTLE_ACT, SPEED_STEER, DELTA_STEER, DELTA_THROTTLE };
class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
