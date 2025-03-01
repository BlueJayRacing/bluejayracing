#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ct/core/core.h"
#include "ct/optcon/optcon.h"
#include "ct_test/system_models.h"
#include "ct_test/optimal_control_problem.h"

using namespace ct_test;

class CTOptconNode : public rclcpp::Node
{
public:
    CTOptconNode() : Node("ct_optcon_test")
    {
        // Initialize a logger
        RCLCPP_INFO(this->get_logger(), "Starting CT Optcon test node");
        
        // Create a timer to run our examples
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&CTOptconNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Running LQR example");
        
        // Run the simple LQR example
        run_lqr_example();
        
        // Cancel the timer to prevent repeated executions
        timer_->cancel();
    }
    
    void run_lqr_example()
    {
        RCLCPP_INFO(this->get_logger(), "Spring-Mass-Damper LQR Example:");
        
        try {
            // Define system parameters
            double mass = 1.0;       // kg
            double stiffness = 10.0; // N/m
            double damping = 0.1;    // Ns/m
            
            RCLCPP_INFO(this->get_logger(), "Creating LQR controller for system with:");
            RCLCPP_INFO(this->get_logger(), "  Mass: %.2f kg", mass);
            RCLCPP_INFO(this->get_logger(), "  Spring stiffness: %.2f N/m", stiffness);
            RCLCPP_INFO(this->get_logger(), "  Damping coefficient: %.2f Ns/m", damping);
            
            // Get LQR gain matrix using our helper function
            auto K = SpringMassLQR::computeGain(mass, stiffness, damping);
            
            // Print the result
            RCLCPP_INFO(this->get_logger(), "LQR gain matrix:");
            RCLCPP_INFO(this->get_logger(), "K = [%.4f, %.4f]", K(0, 0), K(0, 1));
            
            // Test the controller with an initial state
            ct::core::StateVector<2> x;
            x << 1.0, 0.0;  // Initial state: position=1, velocity=0
            
            ct::core::ControlVector<1> u = -K * x;
            
            RCLCPP_INFO(this->get_logger(), "For initial state: position=%.2f, velocity=%.2f", x(0), x(1));
            RCLCPP_INFO(this->get_logger(), "LQR control input: u=%.4f", u(0));
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in LQR example: %s", e.what());
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CTOptconNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}