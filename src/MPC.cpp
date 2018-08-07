
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"

using CppAD::AD;

class FG_eval {
public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    
    FG_eval(Eigen::VectorXd coeffs, MPC *mpcInstance) {
	_coeffs = coeffs;
	_Lf = 2.67;
	
	_mpcI = mpcInstance;
    }

    void operator()(ADvector& fg, const ADvector& vars) {
	// Store cost function in fg[0]
	fg[0] = 0;
	for (unsigned int t=0; t<_mpcI->_N; ++t) {
// 	for (unsigned int t=0; t<5; ++t) {
	    fg[0] += CppAD::pow(vars[_mpcI->_cte_start + t], 2); // lower cross track error
	    fg[0] += 50*CppAD::pow(vars[_mpcI->_epsi_start + t], 2); // lower track angle error
	    fg[0] += CppAD::pow(vars[_mpcI->_v_start + t] - _mpcI->_desiredV, 2); // reach desired speed
	    
// 	    fg[0] -= 1*vars[_mpcI->_x_start+t];
	}
// 	fg[0] += 1000000*CppAD::pow(vars[_mpcI->_cte_start+1], 2);
	for (int t = 0; t < _mpcI->_N - 1; t++) {
	    fg[0] += CppAD::pow(vars[_mpcI->_delta_start + t], 2); // minimize control signals
	    fg[0] += CppAD::pow(vars[_mpcI->_a_start + t], 2); // minimize control signals
	}
	for (int t = 0; t < _mpcI->_N - 2; t++) {
	    fg[0] += 1000*CppAD::pow(vars[_mpcI->_delta_start + t + 1] - vars[_mpcI->_delta_start + t], 2); // make control signals smooth
	    fg[0] += CppAD::pow(vars[_mpcI->_a_start + t + 1] - vars[_mpcI->_a_start + t], 2); // make control signals smooth
	}
	
	// Setup Constraints
	// We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
	
	//Initial constraints
	fg[1 + _mpcI->_x_start] = vars[_mpcI->_x_start];
	fg[1 + _mpcI->_y_start] = vars[_mpcI->_y_start];
	fg[1 + _mpcI->_psi_start] = vars[_mpcI->_psi_start];
	fg[1 + _mpcI->_v_start] = vars[_mpcI->_v_start];
	fg[1 + _mpcI->_cte_start] = vars[_mpcI->_cte_start];
	fg[1 + _mpcI->_epsi_start] = vars[_mpcI->_epsi_start];

	// The rest of the constraints
	for (int t = 1; t < _mpcI->_N; t++) {
	  AD<double> x1 = vars[_mpcI->_x_start + t];
	  AD<double> y1 = vars[_mpcI->_y_start + t];
	  AD<double> psi1 = vars[_mpcI->_psi_start + t];
	  AD<double> v1 = vars[_mpcI->_v_start + t];
	  AD<double> cte1 = vars[_mpcI->_cte_start + t];
	  AD<double> epsi1 = vars[_mpcI->_epsi_start + t];
	  
	  AD<double> x0 = vars[_mpcI->_x_start + t - 1];
	  AD<double> y0 = vars[_mpcI->_y_start + t - 1];
	  AD<double> psi0 = vars[_mpcI->_psi_start + t - 1];
	  AD<double> v0 = vars[_mpcI->_v_start + t - 1];
	  AD<double> cte0 = vars[_mpcI->_cte_start + t - 1];
	  AD<double> epsi0 = vars[_mpcI->_epsi_start + t - 1];
	  
	  AD<double> delta0 = vars[_mpcI->_delta_start + t - 1];
	  AD<double> a0 = vars[_mpcI->_a_start + t - 1];
	  
	  AD<double> f0 = 0;
	  AD<double> xt = 1;
	  for (unsigned int i=0; i<_coeffs.size(); ++i) {
	      f0 += _coeffs[i]*xt;
	      xt *= x0;
	  }
// 	  AD<double> f0 = _coeffs[0] + _coeffs[1] * x0;
	  AD<double> psi_des0 = CppAD::atan(_coeffs[1]);
	
	  fg[1 + _mpcI->_x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * _mpcI->_dt);
	  fg[1 + _mpcI->_y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * _mpcI->_dt);
	  fg[1 + _mpcI->_psi_start + t] = psi1 - (psi0 + v0 / _Lf * delta0 * _mpcI->_dt);
	  fg[1 + _mpcI->_v_start + t] = v1 - (v0 + a0 * _mpcI->_dt);
	  fg[1 + _mpcI->_cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * _mpcI->_dt);
	  fg[1 + _mpcI->_epsi_start + t] = epsi1 - (psi0 -psi_des0 + v0 / _Lf * delta0 * _mpcI->_dt);
	}
    }

protected:
    
    Eigen::VectorXd				_coeffs;				//!< Fitted polynomial coefficients
    double						_Lf;
    MPC						*_mpcI;
};

MPC::MPC(size_t N, double dt) {
    _N = N;
    _dt = dt;
    
    _x_start = 0;
    _y_start = _x_start + _N;
    _psi_start = _y_start + _N;
    _v_start = _psi_start + _N;
    _cte_start = _v_start + _N;
    _epsi_start = _cte_start + _N;
    _delta_start = _epsi_start + _N;
    _a_start = _delta_start + _N - 1;
    
    _controlDelay = 0;
    _controlDelayCurrent = 0;
}

MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // Number of model variables (includes both states and inputs):
    size_t n_vars = _N * 6 + (_N - 1) * 2;
    // Number of constraints
    size_t n_constraints = _N*6;

    // Initial value of the independent variables. SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
	vars[i] = 0;
    vars[_x_start] = state[0];
    vars[_y_start] = state[1];
    vars[_psi_start] = state[2];
    vars[_v_start] = state[3];
    vars[_cte_start] = state[4];
    vars[_epsi_start] = state[5];
    
    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // All non-actuators upper and lowerlimits are set to the max negative and positive values.
    for (int i = 0; i < _delta_start; i++) {
	vars_lowerbound[i] = std::numeric_limits<double>::lowest();
	vars_upperbound[i] = std::numeric_limits<double>::max();
    }
    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    for (int i = _delta_start; i < _a_start; i++) {
	vars_lowerbound[i] = -0.436332;
	vars_upperbound[i] = 0.436332;
    }
    // Acceleration/decceleration upper and lower limits.
    for (int i = _a_start; i < n_vars; i++) {
	vars_lowerbound[i] = -1.0;
	vars_upperbound[i] = 1.0;
    }
    
    // Set lower and upper limits for the constraints (should be 0 besides initial state).
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
	constraints_lowerbound[i] = 0;
	constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = state[0];
    constraints_lowerbound[_y_start] = state[1];
    constraints_lowerbound[_psi_start] = state[2];
    constraints_lowerbound[_v_start] = state[3];
    constraints_lowerbound[_cte_start] = state[4];
    constraints_lowerbound[_epsi_start] = state[5];
    constraints_upperbound[_x_start] = state[0];
    constraints_upperbound[_y_start] = state[1];
    constraints_upperbound[_psi_start] = state[2];
    constraints_upperbound[_v_start] = state[3];
    constraints_upperbound[_cte_start] = state[4];
    constraints_upperbound[_epsi_start] = state[5];
      
    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs, this);

    // Options for IPOPT solver
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

    // Place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
								    constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;
    
    static CppAD::ipopt::solve_result<Dvector> _goodSolution = solution;
    static auto prevCost = cost;
    
    // if new cost is too high, we drop current solution and use previouse one
    if ( (cost < 10 * prevCost) || (_controlDelayCurrent >= _N) ) {
	_goodSolution = solution;
	_controlDelayCurrent = _controlDelay;
	prevCost = cost;
    }
    else
	++_controlDelayCurrent;
    
    // update mpc traectory points
    _mpc_x.erase(_mpc_x.begin(), _mpc_x.end());
    _mpc_y.erase(_mpc_y.begin(), _mpc_y.end());
    for (unsigned int i=0; i<_N; ++i) {
	_mpc_x.push_back(solution.x[i+_x_start]);
	_mpc_y.push_back(solution.x[i+_y_start]);
    }
    
    return {_goodSolution.x[_delta_start+_controlDelayCurrent], _goodSolution.x[_a_start+_controlDelayCurrent]};
}

void MPC::SetDesiredV(double desiredV)
{
    _desiredV = desiredV;
    return;
}

void MPC::SetControlDelay(unsigned int cdelay)
{
    _controlDelay = cdelay;
    return;
}

std::vector<double> & MPC::MPC_X()
{
    return _mpc_x;
}

std::vector<double> & MPC::MPC_Y()
{
    return _mpc_y;
}
