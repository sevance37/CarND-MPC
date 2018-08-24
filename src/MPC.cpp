#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.15; 

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Set the target velocity (in mph)
double tgt_v = 80*(1609.34/3600.0);

// Set where the state variables and actuator variables are located in a singular vector.
size_t x_0     = 0;
size_t y_0     = x_0 + N;
size_t psi_0   = y_0 + N;
size_t v_0     = psi_0 + N;
size_t cte_0   = v_0 + N;
size_t epsi_0  = cte_0 + N;
size_t delta_0 = epsi_0 + N;
size_t a_0     = delta_0 + N - 1;

size_t i_x_0     = 0;
size_t i_y_0     = i_x_0 + N;
size_t i_psi_0   = i_y_0 + N;
size_t i_v_0     = i_psi_0 + N;
size_t i_cte_0   = i_v_0 + N;
size_t i_epsi_0  = i_cte_0 + N;
size_t i_delta_0 = i_epsi_0 + N;
size_t i_a_0     = i_delta_0 + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.

    double wt_cte = 1.0;
    double wt_epis = 10;
    double wt_v = 0.0125;
    double wt_delta = 1000;
    double wt_a = 0.01;
    double wt_chg_delta = 1000;
    double wt_chg_a = 0.01;
    
    fg[0] = 0;
    
    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += wt_cte*CppAD::pow(vars[i_cte_0 + t], 2);
      fg[0] += wt_epis*CppAD::pow(vars[i_epsi_0 + t], 2);
      fg[0] += wt_v*CppAD::pow(vars[i_v_0 + t] - tgt_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += wt_delta*CppAD::pow(vars[i_delta_0 + t], 2);
      fg[0] += wt_a*CppAD::pow(vars[i_a_0 + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += wt_chg_delta*CppAD::pow(vars[i_delta_0 + t + 1] - vars[i_delta_0 + t], 2);
      fg[0] += wt_chg_a*CppAD::pow(vars[i_a_0 + t + 1] - vars[i_a_0 + t], 2);
    }
    
    // Initialize and Setup Constraints

    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + i_x_0] = vars[i_x_0];
    fg[1 + i_y_0] = vars[i_y_0];
    fg[1 + i_psi_0] = vars[i_psi_0];
    fg[1 + i_v_0] = vars[i_v_0];
    fg[1 + i_cte_0] = vars[i_cte_0];
    fg[1 + i_epsi_0] = vars[i_epsi_0];

    // The rest of the constraints
    for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x_tp1 = vars[i_x_0 + t];
      AD<double> y_tp1 = vars[i_y_0 + t];
      AD<double> psi_tp1 = vars[i_psi_0 + t];
      AD<double> v_tp1 = vars[i_v_0 + t];
      AD<double> cte_tp1 = vars[i_cte_0 + t];
      AD<double> epsi_tp1 = vars[i_epsi_0 + t];

      // The state at time t.
      AD<double> x_t = vars[i_x_0 + t - 1];
      AD<double> y_t = vars[i_y_0 + t - 1];
      AD<double> psi_t = vars[i_psi_0 + t - 1];
      AD<double> v_t = vars[i_v_0 + t - 1];
      AD<double> cte_t = vars[i_cte_0 + t - 1];
      AD<double> epsi_t = vars[i_epsi_0 + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta_t = vars[i_delta_0 + t - 1];
      AD<double> a_t = vars[i_a_0 + t - 1];
      
      // please change this to the 3 coefficients.
      AD<double> f_t = coeffs[0] + coeffs[1]*x_t + coeffs[2]*CppAD::pow(x_t,2) + coeffs[3]*CppAD::pow(x_t,3);
      AD<double> psides_t = CppAD::atan(coeffs[1]+2*coeffs[2]*x_t + 3*coeffs[3]*CppAD::pow(x_t,2));

      // From the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + i_x_0 + t] = x_tp1 - (x_t + v_t * CppAD::cos(psi_t) * dt);
      fg[1 + i_y_0 + t] = y_tp1 - (y_t + v_t * CppAD::sin(psi_t) * dt);
      fg[1 + i_psi_0 + t] = psi_tp1 - (psi_t + v_t * delta_t / Lf * dt);
      fg[1 + i_v_0 + t]   = v_tp1 - (v_t + a_t * dt);
      fg[1 + i_cte_0 + t] = cte_tp1 - ((f_t - y_t) + (v_t * CppAD::sin(epsi_t) * dt));
      fg[1 + i_epsi_0 + t] = epsi_tp1 - ((psi_t - psides_t) + v_t * delta_t / Lf * dt);
    }  
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N*6 + (N-1)*2;
  // Set the number of constraints
  size_t n_constraints = N*6;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];  
  double epsi = state[5]; 
  
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  vars[i_x_0] = x;
  vars[i_y_0] = y;
  vars[i_psi_0] = psi;
  vars[i_v_0] = v;
  vars[i_cte_0] = cte;
  vars[i_epsi_0] = epsi;
  
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.
  
  // Set all non-actuators upper and lower limits to the max negative and positive values.
  for (size_t i = 0; i < i_delta_0; i++) {
    vars_lowerbound[i] = -1.0e10;
    vars_upperbound[i] = 1.0e10;
  }

  // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
  for (size_t i = i_delta_0; i < i_a_0; i++) {
    vars_lowerbound[i] = -0.4363323;
    vars_upperbound[i] = +0.4363323;
  }

  // Acceleration/decceleration upper and lower limits.
  double max_accel = 1;
  for (size_t i = i_a_0; i < n_vars; i++) {
    vars_lowerbound[i] = -max_accel;
    vars_upperbound[i] = max_accel;
  }  
  
  // Lower and upper limits for the constraints. Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  constraints_lowerbound[i_x_0] = x;
  constraints_lowerbound[i_y_0] = y;
  constraints_lowerbound[i_psi_0] = psi;
  constraints_lowerbound[i_v_0] = v;
  constraints_lowerbound[i_cte_0] = cte;
  constraints_lowerbound[i_epsi_0] = epsi;

  constraints_upperbound[i_x_0] = x;
  constraints_upperbound[i_y_0] = y;
  constraints_upperbound[i_psi_0] = psi;
  constraints_upperbound[i_v_0] = v;
  constraints_upperbound[i_cte_0] = cte;
  constraints_upperbound[i_epsi_0] = epsi;
  
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Note: {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> output(2*N);
  output[0] = solution.x[i_delta_0];
  output[1] = solution.x[i_a_0];
  for (size_t t = 1; t < N; t++) { 
      output[2*t] = solution.x[i_x_0+t];
      output[2*t+1] = solution.x[i_y_0+t];
  }

  return output;
}
