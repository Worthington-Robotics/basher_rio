#include "subsystems/userinput.h"
#include <cmath>
#include <string>

namespace robot
{

    UserInput::UserInput() {}

    void UserInput::registerSticks(std::vector<int> stickIds)
    {
        for (int id : stickIds)
        {
            sticks.push_back(frc::Joystick(id));
        }
    }

    void UserInput::createRosBindings(rclcpp::Node *node)
    {
        for (auto stick = sticks.begin(); stick != sticks.end(); ++stick)
        {
            stickPubs.push_back(
                node->create_publisher<sensor_msgs::msg::Joy>("/sticks/stick" + std::to_string(stick->GetPort()), rclcpp::SensorDataQoS())
            );
        }
    }

    void UserInput::reset() {}

    void UserInput::onStart() {}

    void UserInput::onLoop() {}

    void UserInput::publishData()
    {
        for (int i = 0; i < sticks.size(); i++)
        {
            sensor_msgs::msg::Joy stickData;

            int numAxes = sticks.at(i).GetAxisCount();
            std::vector<float> axesValues;
            for (int axis = 1; axis < numAxes + 1; axis++)
            {
                axesValues.push_back(sticks.at(i).GetRawAxis(axis));
            }
            stickData.axes = axesValues;

            int numButtons = sticks.at(i).GetButtonCount();
            std::vector<int> buttonValues;
            for(int button = 1; button < numButtons + 1; button++){
                buttonValues.push_back(sticks.at(i).GetRawButton(button)? 1: 0);
            }
            stickData.buttons = buttonValues;

            stickPubs.at(i)->publish(stickData);
        }
    }

    std::vector<double> UserInput::evalDeadband(const sensor_msgs::msg::Joy &joyMsg,
                                                const double deadBand, const int power)
    {
        auto output = std::vector<double>();
        for (double axis : joyMsg.axes)
        {
            if (abs(axis) < deadBand)
            {
                output.push_back(0.0);
            }
            else
            {
                if (axis < 0)
                {
                    output.push_back(abs(pow(axis, power)));
                }
                else
                {
                    output.push_back(abs(pow(axis, power)));
                }
            }
        }
        return output;
    }

    std::vector<double> UserInput::scalarCut(const sensor_msgs::msg::Joy &joyMsg,
                                             const double deadBand, const int power, const std::vector<double> scalars)
    {
        auto output = evalDeadband(joyMsg, deadBand, power);
        for (int i = 0; i < scalars.size(); i++)
        {
            output.at(i) = output.at(i) * scalars.at(i);
        }
        return output;
    }

    double UserInput::mapValue(double input, double minOutput, double maxOutput)
    {
        return (input + 1) * (maxOutput - minOutput) / (2) + minOutput;
    }

} // namespace robot
