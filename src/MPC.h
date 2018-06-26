#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"


#define REF_CTE 0  // Reference cross track error 0 (Aim for zero track error)
#define REF_EPSI 0 // Reference PSI error as 0
#define REF_V 75   // Reference Velocity 65 (Freeway speed allowed in CA). Assuming it is a freeway even though it is not in the simulator :)

// Set weights parameters for the cost function
#define W_CTE 5000   // Weight for cross track error
#define W_EPSI 5000  // Weight for PSI error
#define W_DV 500    // Increase to remove sharp turns at high speeds
#define W_DELTA 5
#define W_A 5
#define W_DDELTA 200 // increase to remove sharp turns
#define W_DA 200  //increase to remove sudden acceleration or de-acceleration

// Set lower and upper limits for variables.
#define DED25RAD 0.436332 // 25 deg in rad, used as delta bound
#define MAXTHR 1.0 // Maximum a value
#define BOUND 1.0e19 // Bound value for other variables


using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  vector<double> mpc_x;
  vector<double> mpc_y;
  
};

#endif /* MPC_H */
