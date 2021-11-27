#include "subsystems/userinput.h"

namespace robot
{

    UserInput::UserInput(){}

    void UserInput::registerSticks(std::vector<int> stickIds){
        for(int id : stickIds){
            sticks.push_back(frc::Joystick(id));
        }
    }

    void UserInput::createRosBindings(rclcpp::Node *node){
        for(auto stick = sticks.begin(); stick != sticks.end(); ++stick){
            stickPubs.push_back(
                node->create_publisher<sensor_msgs::msg::Joy>("/sticks/stick" + stick->GetPort(), rclcpp::SensorDataQoS())
            );
        }
    }

    void UserInput::reset(){}

    void UserInput::onStart(){}

    void UserInput::onLoop(){}

    void UserInput::publishData(){
        for(int i = 0; i < sticks.size(); i++){
            sensor_msgs::msg::Joy stickData;

            int numAxes = sticks.at(i).GetAxisCount();
            std::vector<float> axesValues; 
            for(int axis = 1; axis < numAxes + 1; axis++){
                axesValues.push_back(sticks.at(i).GetRawAxis(axis));
            }
            stickData.axes = axesValues;

            int numButtons = sticks.at(i).GetButtonCount();
            std::vector<int> buttonValues;
            for(int button = 1; button < numButtons + 1; button++){
                buttonValues.push_back(sticks.at(i).GetRawButton(button)? 0: 1);
            }
            stickData.buttons = buttonValues;

            stickPubs.at(i)->publish(stickData);
        }
    }
    
} // namespace robot
