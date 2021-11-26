#include "subsystems/drivetrain.h"
#include "Constants.h"

namespace robot {

    Drivetrain::Drivetrain(){
        leftMaster = std::make_shared<TalonFX>(DRIVE_LEFT_MASTER);
        rightMaster = std::make_shared<TalonFX>(DRIVE_RIGHT_MASTER);

        leftFollower = std::make_shared<TalonFX>(DRIVE_LEFT_FOLLOWER);
        rightFollower = std::make_shared<TalonFX>(DRIVE_RIGHT_FOLLOWER);

        // enable follower mode
        rightFollower->Follow(*rightMaster);
        leftFollower->Follow(*leftMaster);

        shifter = std::make_shared<frc::DoubleSolenoid>(DRIVE_SHIFT_LOW, DRIVE_SHIFT_HIGH);

        // build the ramsete controller
        controller = frc::RamseteController();
    }

    void Drivetrain::createRosBindings(rclcpp::Node * node){

    }

    void Drivetrain::reset(){

    }

    void Drivetrain::onStart(){

    }

    void Drivetrain::onLoop(){
        switch(driveState){

        }

        switch(shiftState){

        }

    }

    void Drivetrain::publishData(){

    }

    void Drivetrain::trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){

    }

    void Drivetrain::twistCallback(const geometry_msgs::msg::Twist msg){

    }

    /**
     * callback for gearbox settings to be changed / streamed in
     **/ 
    void Drivetrain::gearCallback(const std_msgs::msg::Int16 msg){

    }

    void Drivetrain::controlCallback(const std_msgs::msg::Int16 msg){

    }

} // namespace robot
