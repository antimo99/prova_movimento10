#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "lettura_singola/quintic.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>
#include <string>
#include <rclcpp/wait_for_message.hpp>

class MyNode : public rclcpp::Node
{
public:
    MyNode(const rclcpp::NodeOptions opt = rclcpp::NodeOptions()) : Node("quintico", opt)
    {
        command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_velocity_controller/command", 10);

        q0.data.resize(joint_names_.size());
        q0.data = {0, 0, 0, 0, 0, 0, 0};
        qf.data.resize(joint_names_.size());
        valori_iniziali.position.resize(joint_names_.size());
        using namespace std::chrono_literals;

        // Attendi il singolo messaggio e salvalo nella variabile globale
        if (!rclcpp::wait_for_message<sensor_msgs::msg::JointState>(valori_iniziali, shared_from_this(), "/joint_states", 10s))
        {
            for (size_t i = 0; i < valori_iniziali.position.size(); ++i)
            {
                q0.data[i]=valori_iniziali.position[i];
                RCLCPP_INFO(this->get_logger(), "Valore iniziale salvato");
            }
        }

        // Configurazione finale dei joint
        qf.data = q0.data;
        qf.data[6]=qf.data[6]+M_PI/4;


        rclcpp::WallRate loop_rate(1000); // 1 kHz
        double traj_duration = 8.0;
        rclcpp::Time t0 = this->now();
        rclcpp::Duration t(0, 0);

        // Ciclo di "generazione traiettoria"
        while (rclcpp::ok() && t.seconds() <= traj_duration)
        {
            t = this->now() - t0;

            // riempio il goal
            std_msgs::msg::Float64MultiArray goal;
            goal.data.resize(joint_names_.size());

            // riempio il vettore usando la funzione quintic
            for (size_t i = 0; i < joint_names_.size(); ++i)
                goal.data[i] = quintic(t.seconds(), q0.data[i], qf.data[i], traj_duration);

            command_pub_->publish(goal);
            loop_rate.sleep();
        }
    }

private:
    std::vector<std::string> joint_names_ = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    std_msgs::msg::Float64MultiArray q0, qf;
    sensor_msgs::msg::JointState valori_iniziali;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}