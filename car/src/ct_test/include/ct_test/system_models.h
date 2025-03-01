#pragma once

#include <ct/core/core.h>

namespace ct_test {

/**
 * @brief Linear system for a simple spring-mass-damper system
 */
class SpringMassDamper : public ct::core::LinearSystem<2, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;
    
    /**
     * @brief Constructor
     * @param mass Mass [kg]
     * @param stiffness Spring stiffness [N/m]
     * @param damping Damping coefficient [Ns/m]
     */
    SpringMassDamper(double mass, double stiffness, double damping)
        : mass_(mass), stiffness_(stiffness), damping_(damping)
    {
        // Initialize system matrices
        A_.setZero();
        B_.setZero();
        
        // State matrix (continuous time)
        A_(0, 1) = 1.0;
        A_(1, 0) = -stiffness / mass;
        A_(1, 1) = -damping / mass;
        
        // Input matrix
        B_(1, 0) = 1.0 / mass;
    }
    
    /**
     * @brief Copy constructor
     */
    SpringMassDamper(const SpringMassDamper& other)
        : mass_(other.mass_), stiffness_(other.stiffness_), damping_(other.damping_),
          A_(other.A_), B_(other.B_) {}
    
    /**
     * @brief Clone method for deep copying
     */
    SpringMassDamper* clone() const override
    {
        return new SpringMassDamper(*this);
    }
    
    /**
     * @brief Get state matrix A
     */
    const ct::core::StateMatrix<2>& getDerivativeState(const ct::core::StateVector<2>& x, 
                                                      const ct::core::ControlVector<1>& u,
                                                      const double t = 0.0) override
    {
        return A_;
    }
    
    /**
     * @brief Get input matrix B
     */
    const ct::core::StateControlMatrix<2, 1>& getDerivativeControl(const ct::core::StateVector<2>& x,
                                                                  const ct::core::ControlVector<1>& u,
                                                                  const double t = 0.0) override
    {
        return B_;
    }
    
private:
    double mass_;
    double stiffness_;
    double damping_;
    ct::core::StateMatrix<2> A_;
    ct::core::StateControlMatrix<2, 1> B_;
};

} // namespace ct_test