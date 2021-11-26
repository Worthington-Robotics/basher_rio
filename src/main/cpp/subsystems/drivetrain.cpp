#include "subsystems/drivetrain.h"

namespace robot {

    Drivetrain::Drivetrain(){
        leftMaster = std::make_shared<TalonFX>(1);
        rightMaster = std::make_shared<TalonFX>(2);

        leftFollower = std::make_shared<TalonFX>(3);
        rightFollower = std::make_shared<TalonFX>(4);

        // enable follower mode
        rightFollower->Follow(*rightMaster);
        leftFollower->Follow(*leftMaster);

        shifter = std::make_shared<frc::DoubleSolenoid>(1,2);

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

    }

    void Drivetrain::publishData(){

    }

    void Drivetrain::leftCallback(){

    }

    void Drivetrain::rightCallback(){

    }

    /**
     * callback for gearbox settings to be chnaged / streamed in
     **/ 
    void Drivetrain::gearCallback(){

    }
    
} // namespace robot
