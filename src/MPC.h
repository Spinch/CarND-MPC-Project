
#ifndef _MPC_h_
#define _MPC_h_

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {

friend class FG_eval;

public:

    /** @brief Constructor
     *  @param[in] N number of predicted points
     *  @param[in] dt time step of preditcion
     */
    MPC(size_t N, double dt);
    
    /** @breif Destructor
     */
    virtual ~MPC();

    /** @brief Solve the model given an initial state and polynomial coefficients.
     *  @param[in] state initial system state
     *  @param[in] coeffs polynomial coefficients of target line
     *  @return actuations for time step set by "SetControlDelay" function
     */
    std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    
    /** @brief Set desired speed
     *  @param[in] desiredV desired speed
     */
    void SetDesiredV(double desiredV);
    
//     /** @breif Set time step of control delay; control delay will be cdelay*dt
//      *  @param[in] cdelay time step of control delay
//      */
//     void SetControlDelay(unsigned int cdelay);
    
    /** @breif Predict car state after dt
     *  @param[in] state curent system state
     *  @param[in] dt prediction time
     *  @param[in] delta steering angle
     *  @param[in] a acceleration
     *  @return car state after dt
     */
    Eigen::VectorXd PredictState(Eigen::VectorXd state, double dt, double delta, double a);
    
    /** @get predicted car x points for visualisation
     *  @return vector of predicted x coordinates
     */
    std::vector<double> &MPC_X();
    
    /** @get predicted car y points for visualisation
     *  @return vector of predicted y coordinates
     */
    std::vector<double> &MPC_Y();
    
protected:
    
    double						_desiredV;					//!< desired speed
    size_t						_N;							//!< number of predicted steps
    double						_dt;							//!< delta t between prediction steps
    double						_Lf;							//!< distance between front axis and gravity center

    size_t						_x_start;						//!< start point of x parameters in all parameters vector
    size_t						_y_start;						//!< start point of y parameters in all parameters vector
    size_t						_psi_start;					//!< start point of psi parameters in all parameters vector
    size_t						_v_start;						//!< start point of v parameters in all parameters vector
    size_t						_cte_start;					//!< start point of cte parameters in all parameters vector
    size_t						_epsi_start;					//!< start point of epsi parameters in all parameters vector
    size_t						_delta_start;					//!< start point of delta parameters in all parameters vector
    size_t						_a_start;						//!< start point of a parameters in all parameters vector
    
    std::vector<double>			_mpc_x;						//!< vector of predicted x coordinates
    std::vector<double>			_mpc_y;						//!< vector of predicted y coordinates
    
    unsigned int					_controlDelay;				//!< current step of control delay
    unsigned int					_controlDelayCurrent;			//!< set step of control delay
};

#endif /* _MPC_h_ */
