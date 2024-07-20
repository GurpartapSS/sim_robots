#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <libserial/SerialPort.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
namespace nemo_firmware {

using lcCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class nemoInterface : public hardware_interface::SystemInterface {
    public:
    nemoInterface() {

    }
    ~nemoInterface() {
        if(arduino_.IsOpen()) {
            try {
                arduino_.Close();
            } catch(...) {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("nemoInterface"), "something went wrong while closing port " << port_);
            }
        }
    }

    virtual lcCallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state) override {
        RCLCPP_INFO(rclcpp::get_logger("nemoInterface"), "LifeCycle Node activated!");
        velocity_cmds_ = {0.0,0.0};
        position_states_ = {0.0,0.0};
        position_states_ = {0.0,0.0};

        try {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        } catch(...){
            RCLCPP_FATAL(rclcpp::get_logger("nemoInterface"),"Something went wrong when opening port");
            return lcCallbackReturn::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("nemoInterface"),"HW interface is ready!");
        return lcCallbackReturn::SUCCESS;
    }

    virtual lcCallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state) override {
        RCLCPP_INFO(rclcpp::get_logger("nemoInterface"),"Stopping HW interface!!!");
        if(arduino_.IsOpen()) {
            try {
                arduino_.Close();
            } catch(...) {
                RCLCPP_FATAL(rclcpp::get_logger("nemoInterface"),"Something went wrong when closing port");
                lcCallbackReturn::FAILURE;
            }
        }
        return lcCallbackReturn::SUCCESS;  
    }

    virtual lcCallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override {
        lcCallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != lcCallbackReturn::SUCCESS) {
            return result;
        }
        try {
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range e) {
            RCLCPP_FATAL(rclcpp::get_logger("nemoInterface"), "No serial port!");
            return lcCallbackReturn::FAILURE;
        }

        velocity_cmds_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        return lcCallbackReturn::SUCCESS;
    }
    
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for( size_t i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }
        return state_interfaces;
    }
    
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0 ; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &velocity_cmds_[i]));
        }
        return command_interfaces;
    }

    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration &period) override {
        if(arduino_.IsDataAvailable()) {
            std::string msg;
            arduino_.ReadLine(msg);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("nemoInterface"),"reading " << msg);
            std::stringstream ss(msg);
            std::string res;
            int multiplier = 1;
            while(std::getline(ss, res, ',')) {
                multiplier = res[1] == 'p' ? 1 : -1;
                if(res[0] == 'r') {
                    velocity_states_[0] = multiplier* std::stod(res.substr(2, res.size()));
                } else if(res[0] == 'l'){
                    velocity_states_[1] = multiplier* std::stod(res.substr(2, res.size()));
                }
            }
        }
        return hardware_interface::return_type::OK;
    }
    
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration &period) override {
        std::stringstream message_stream;
        char right_wheel_sign = velocity_cmds_[0] > 0 ? 'p' : 'n';
        char left_wheel_sign = velocity_cmds_[1] > 0 ? 'p' : 'n';
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";
        if(std::abs(velocity_cmds_[0]) < 10.0) {
            compensate_zeros_right = "0";
        } else {
            compensate_zeros_right = "";
        }
        if(std::abs(velocity_cmds_[1]) < 10.0) {
            compensate_zeros_left = "0";
        } else {
            compensate_zeros_left = "";
        }

        message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << abs(velocity_cmds_[0])
        << "," << "l" << left_wheel_sign << compensate_zeros_left << abs(velocity_cmds_[1]) << ",";
        try {
            arduino_.Write(message_stream.str());
            RCLCPP_INFO_STREAM(rclcpp::get_logger("nemoInterface"),"sending " << message_stream.str().c_str());
        } catch(...) {
                RCLCPP_FATAL(rclcpp::get_logger("nemoInterface"),"Something went wrong when writing to port");
                return hardware_interface::return_type::ERROR;
        }
                return hardware_interface::return_type::OK;
    }

private:
    LibSerial::SerialPort arduino_;
    std::string port_;
    std::vector<double> velocity_cmds_;
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;  
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nemo_firmware::nemoInterface, hardware_interface::SystemInterface);