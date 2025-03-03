#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ct/core/core.h"
#include "ct_test/system_models.h"
#include "ct_test/optimal_control_problem.h"

using namespace ct_test;

class CTCoreNode : public rclcpp::Node
{
public:
    CTCoreNode() : Node("ct_core_test")
    {
        // Initialize a logger
        RCLCPP_INFO(this->get_logger(), "Starting CT Core test node");
        
        // Create a timer to run our simulation
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&CTCoreNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Running CT Core state space simulation");
        
        // Run the spring-mass-damper simulation
        run_spring_mass_damper_simulation();
        
        // Cancel the timer to prevent repeated executions
        timer_->cancel();
    }
    
    void run_spring_mass_damper_simulation()
    {
        RCLCPP_INFO(this->get_logger(), "Spring-Mass-Damper Simulation:");
        
        // Create a spring-mass-damper system
        double mass = 1.0;       // kg
        double stiffness = 10.0; // N/m
        double damping = 0.1;    // Ns/m
        
        std::shared_ptr<SpringMassDamper> system(new SpringMassDamper(mass, stiffness, damping));
        
        // Create a PD controller
        double kp = 5.0;  // proportional gain
        double kd = 1.0;  // derivative gain
        
        std::shared_ptr<PDController> controller(new PDController(kp, kd));
        
        // Set the controller on the system
        system->setController(controller);
        
        // Create a state vector
        ct::core::StateVector<2> x;
        x << 1.0, 0.0;  // Initial state: position=1, velocity=0
        
        RCLCPP_INFO(this->get_logger(), "Initial state: position=%.2f, velocity=%.2f", x(0), x(1));
        RCLCPP_INFO(this->get_logger(), "Using PD controller with kp=%.1f, kd=%.1f", kp, kd);
        
        // Create an integrator
        ct::core::Integrator<2> integrator(system);
        
        // Simulation parameters
        double dt = 0.01;          // seconds
        ct::core::Time t = 0.0;    // seconds
        size_t numSteps = 100;     // 1 second of simulation
        
        // Print initial state
        RCLCPP_INFO(this->get_logger(), "t=%.2fs: position=%.4f, velocity=%.4f", 
                   t, x(0), x(1));
        
        // Integrate and print at specific intervals
        for (size_t i = 0; i < 4; i++)  // Print every 0.25 seconds
        {
            // Integrate 25 steps
            integrator.integrate_n_steps(x, t, 25, dt);
            t += 25 * dt;
            
            RCLCPP_INFO(this->get_logger(), "t=%.2fs: position=%.4f, velocity=%.4f", 
                       t, x(0), x(1));
        }
        
        RCLCPP_INFO(this->get_logger(), "Simulation complete after %.2f seconds", t);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CTCoreNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}