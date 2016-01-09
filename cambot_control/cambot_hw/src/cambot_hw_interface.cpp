#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot:
	public hardware_interface::RobotHW
{
	public:
		MyRobot();	//Setup robot

		//Talk to HW
		void read();
		void write();
};

main() {
	MyRobot cambot;
	//controller_manager::ControllerManager cm(&cambot);

	while (true) {}
}

