#pragma once

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>

#include "system_models.h"

namespace ct_test {

/**
 * @brief Simple LQR controller for spring-mass-damper system
 */
class SpringMassLQR {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief Compute LQR gain for spring-mass-damper system
     * 
     * @param mass Mass of the system [kg]
     * @param stiffness Spring stiffness [N/m]
     * @param damping Damping coefficient [Ns/m]
     * @return Feedback gain matrix K
     */
    static ct::core::FeedbackMatrix<2, 1> computeGain(
        double mass, double stiffness, double damping)
    {
        // Create the system matrices directly
        Eigen::Matrix<double, 2, 2> A;
        Eigen::Matrix<double, 2, 1> B;
        
        // State matrix (continuous time)
        A.setZero();
        A(0, 1) = 1.0;
        A(1, 0) = -stiffness / mass;
        A(1, 1) = -damping / mass;
        
        // Input matrix
        B.setZero();
        B(1, 0) = 1.0 / mass;
        
        // Define cost matrices
        Eigen::Matrix<double, 2, 2> Q;
        Eigen::Matrix<double, 1, 1> R;
        
        Q.setIdentity();
        Q(0, 0) = 10.0;  // Higher penalty on position
        Q(1, 1) = 1.0;   // Lower penalty on velocity
        
        R(0, 0) = 0.1;   // Control cost
        
        // Create LQR solver
        ct::optcon::LQR<2, 1> lqrSolver;
        ct::core::FeedbackMatrix<2, 1> K;
        
        // Compute the LQR gain
        lqrSolver.compute(Q, R, A, B, K);
        
        return K;
    }
};

/**
 * @brief Simple PD controller
 */
class PDController : public ct::core::Controller<2, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const size_t state_dim = 2;
    static const size_t control_dim = 1;
    
    /**
     * @brief Constructor
     * 
     * @param kp Proportional gain
     * @param kd Derivative gain
     */
    PDController(double kp, double kd)
        : kp_(kp), kd_(kd)
    {
    }
    
    /**
     * @brief Copy constructor
     */
    PDController(const PDController& other) 
        : kp_(other.kp_), kd_(other.kd_) 
    {
    }
    
    /**
     * @brief Clone method for deep copying
     */
    PDController* clone() const override
    {
        return new PDController(*this);
    }
    
    /**
     * @brief Compute control action
     */
    void computeControl(const ct::core::StateVector<state_dim>& state,
                        const double& t,
                        ct::core::ControlVector<control_dim>& controlAction) override
    {
        controlAction(0) = -kp_ * state(0) - kd_ * state(1);
    }
    
private:
    double kp_;  // proportional gain
    double kd_;  // derivative gain
};

} // namespace ct_test