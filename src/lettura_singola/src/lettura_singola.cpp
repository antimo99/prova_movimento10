#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class JointStateReader : public rclcpp::Node
{
public:
    JointStateReader()
    : Node("reader")
    {
        // Subscriber per il topic /joint_states
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JointStateReader::jointStateCallback, this, std::placeholders::_1));

        // Flag per capire se è stata ricevuta almeno una lettura
        message_received_ = false;
    }

private:
    // Callback che riceve i messaggi dal topic /joint_states
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!message_received_) {
            // Una volta che riceviamo il primo messaggio, possiamo elaborarlo
            RCLCPP_INFO(this->get_logger(), "Lettura iniziale dal topic /joint_states:");

            // Mostra le informazioni sui giunti
            for (size_t i = 0; i < msg->name.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Nome: %s, Posizione: %f", msg->name[i].c_str(), msg->position[i]);
            }

            // Salva la configurazione iniziale
            q0_ = *msg;
            message_received_ = true;

            // Mostra la configurazione iniziale q0
            RCLCPP_INFO(this->get_logger(), "Posizione in q0:");
            for (size_t i = 0; i < q0_.name.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Posizione in q0: %f", q0_.position[i]);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    bool message_received_;
    sensor_msgs::msg::JointState q0_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateReader>());
    rclcpp::shutdown();
    return 0;
}