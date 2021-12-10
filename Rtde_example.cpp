#include <iostream>
#include <thread>
#include <chrono>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <vector>
#include "openvr.h"
#include <math.h>


using namespace ur_rtde;


static void example_try()
//int main(int argc, char* argv[])
{

	// The constructor simply takes the IP address of the Robot
	RTDEControlInterface rtde_control("134.28.124.104");
	RTDEReceiveInterface rtde_receive("134.28.124.104");
	std::vector<double> joint_q = { -0.343, -1.449, 2.122, -3.79, -1.536, 0 };
	std::vector<double> joint_q_two = { -0.327, -0.624, 1.062, -3.079, -1.534, 0.3 };
	std::vector<double> tcp_pose = { -1.169, 0.208, 0.346, 2.7, 1.652, -1.517 };

	while (rtde_receive.isConnected() && rtde_control.isConnected())
	{
		

		//rtde_control.isJointsWithinSafetyLimits(joint_q)

		// Move to initial joint position with a regular moveJ

		rtde_control.moveJ(joint_q, 0.2, 0.4, false);
		std::vector<double> a = rtde_control.getForwardKinematics(joint_q);
		std::vector<double> b = rtde_control.getInverseKinematics(tcp_pose);
		std::cout << "Forward: " << a[0] << " " << a[1] << " " << a[2] << " " << a[3] << " " << a[4] << " " << a[5] << std::endl;


		if (rtde_control.isProgramRunning())

			std::cout << "Actual Q is: " << std::endl;
		for (const auto& d : rtde_receive.getActualQ())
			std::cout << d << " ";
		//std::this_thread::sleep_for(std::chrono::milliseconds(500));
		std::cout << std::endl;

		std::cout << "TCP POSE is: " << std::endl;
		for (const auto& e : rtde_receive.getActualTCPPose())
			std::cout << e << " ";
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		std::cout << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		// Move to secondary joint position with a regular moveJ
		rtde_control.moveL(tcp_pose, 0.1, 0.2, false);
		//rtde_control.moveL(tcp_pose,0.1,0.2,false);
		std::cout << "Inverse: " << b[0] << " " << b[1] << " " << b[2] << " " << b[3] << " " << b[4] << " " << b[5] << std::endl;

		std::cout << "Actual Q is: " << std::endl;
		for (const auto& d : rtde_receive.getActualQ())
			std::cout << d << " ";
		//std::this_thread::sleep_for(std::chrono::milliseconds(500));
		std::cout << std::endl;

		std::cout << "TCP POSE is: " << std::endl;
		for (const auto& e : rtde_receive.getActualTCPPose())
			std::cout << e << " ";
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		std::cout << std::endl;
		rtde_control.stopJ(2);





	}
	do
	{
		std::cout << '\n' << "Lost connection to robot, waiting for user input before retrying...";
	} while (std::cin.get() != '\n');

	rtde_receive.reconnect();
	if (rtde_receive.isConnected())
	{
		std::cout << "Successfully Reconnected to Robot, Actual q is: " << std::endl;
		//for (const auto& d : rtde_receive.getActualQ())
		for (const auto& d : rtde_receive.getActualTCPPose())
			std::cout << d << " ";
	}

	// Parameters






	//return 0;
}