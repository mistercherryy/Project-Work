
#include <iostream>
#include <thread>
#include <chrono>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <vector>
#include "openvr.h"
#include <math.h>

#include "Library.h"	// Access library details
#include "DeviceList.h" // Access devices and communication state
#include "SenseGlove.h" // SenseGlove interface through which we access data.
#include "Tracking.h"

using namespace ur_rtde;

//Vive Tracker Position getters & constructors
namespace vtp
{
    // Returns indices of valid tracked devices of class @lookForClass
    void FindTrackedDevicesOfClass(vr::IVRSystem* system, const vr::TrackedDeviceClass& lookForClass, std::vector<vr::TrackedDeviceIndex_t>& validIndices)
    {
        for (vr::TrackedDeviceIndex_t idx = 0; idx < vr::k_unMaxTrackedDeviceCount; idx++)
        {
            auto trackedDeviceClass = system->GetTrackedDeviceClass(idx);
            if (trackedDeviceClass == lookForClass)
            {
                validIndices.push_back(idx);
                std::cout << "Found valid device at index " << idx << std::endl;
            }
        }
    }
    // Returns a std::string of a trackedDevice property
    std::string GetTrackedPropString(vr::IVRSystem* system, vr::TrackedDeviceIndex_t unDeviceIndex, vr::TrackedDeviceProperty prop)
    {
        if (!system)
            return std::string();

        const uint32_t bufSize = vr::k_unMaxPropertyStringSize;
        //        char* pchValue = new char[bufSize];
        std::unique_ptr<char*> pchValue = std::make_unique<char*>(new char[bufSize]);
        std::string propString = "";
        vr::TrackedPropertyError pError = vr::TrackedPropertyError::TrackedProp_NotYetAvailable;

        system->GetStringTrackedDeviceProperty(unDeviceIndex, prop, *pchValue.get(), bufSize, &pError);

        if (pError != vr::TrackedPropertyError::TrackedProp_Success)
        {
            propString = "Error";
        }
        else
        {
            propString = std::string(*pchValue.get());
        }
        return propString;
    }

    // +y is up, +x is right, -z is forward, units are meters
    // According to https://github.com/ValveSoftware/openvr/issues/689 ,
    // [X1 Y1 Z1 P1]
    // [X2 Y2 Z2 P2]
    // [X3 Y3 Z3 P3]
    // Where P is position vector
    std::string HmdMatrix34ToString(vr::HmdMatrix34_t& m)
    {
        // Returns m.m[col][row] followed by a comma
        auto mn = [&](auto row, auto col) { return std::to_string(m.m[row][col]) + ","; };


        return
            "[" + mn(0, 0) + mn(0, 1) + mn(0, 2) + mn(0, 3) + "\n" +
            mn(1, 0) + mn(1, 1) + mn(1, 2) + mn(1, 3) + "\n" +
            mn(2, 0) + mn(2, 1) + mn(2, 2) + (mn(2, 3)) + "\n" +
            "0.0,      0.0,       0.0,       1.0]";

    }

    // Returns TrackedDevicePose_t struct of device# trackerIdx
    vr::TrackedDevicePose_t GetTrackedDevicePose(vr::IVRSystem* system_, vr::TrackedDeviceIndex_t& trackerIdx)
    {
        vr::VRControllerState_t state;
        vr::TrackedDevicePose_t pose;
        bool gotPose = system_->GetControllerStateWithPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, trackerIdx, &state, 1, &pose);

        if (!gotPose)
        {
            std::cerr << "Failed to get pose";
        }
        return pose;
    }

    // Prints information about TrackedDevice pose
    void PrintTrackedDevicePose(vr::TrackedDevicePose_t& pose)
    {
       

        bool& poseValid = pose.bPoseIsValid;
        std::cout << "Pose: ";
        if (poseValid)
        {
            // Print matrix

            
            /*----from triad.py
            double yaw = 180 / pi * atan2(mat.m[1][0], mat.m[0][0]);
            double pitch = 180 / pi*atan2(mat.m[2][0], mat.m[0][0]);
            double roll = 180 / pi*atan2(mat.m[2][1], mat.m[2][2]);
            double x = mat.m[0][3];
            double y = mat.m[1][3];
            double z = mat.m[2][3]+1.6; */

           
            //std::vector<double> poses = { x, y, z, yaw, roll, pitch };
            
           
            std::cout << std::endl;
            std::cout << "Transformation Matrix: \n " << std::endl;

            std::cout << "\n" + vtp::HmdMatrix34ToString(pose.mDeviceToAbsoluteTracking) << std::endl;
            std::cout << std::endl;
        }
        else
        {
            std::cout << "Invalid" << std::endl;
        }
    }

}




//static void Source()
int main(int argc, char* argv[])
{
    using namespace vr;
    //ROBOT CONSTRUCTORS
// The constructor simply takes the IP address of the Robot
    RTDEControlInterface rtde_control("134.28.124.104");//move the robot
    RTDEReceiveInterface rtde_receive("134.28.124.104");//receive data from robot
    RTDEIOInterface rtde_io104("134.28.124.104");   //to open the gripper from analog output
    RTDEIOInterface rtde_io86("134.28.124.86");     //to close the gripper from analog output

    std::vector<double> joint_q = { -0.3295, -1.5385, 2.307, 2.359, -1.4872, 0.785 };
    std::vector<double> joint_q_two = { -0.327, -0.624, 1.062, -3.079, -1.534, 0.3 };
    std::vector<double> tcp_pose = { 0.5748, 0.0019, 0.5, 2.907, 2.197, -2.316 };
    std::vector<double> room_testj = { -1.035, -1.3172, 2.0733, -3.897, -1.5708, 0.785 };
    std::vector<double> room_tcp = { -0.5, 0.5, 0.5, 3.660, -0.458, -2.261 };

    double pi = 3.141592;
    //senseglove testing line with version appears on screen
    std::cout << "Testing " << SGCore::Library::Version() << std::endl;
    std::cout << "=======================================" << std::endl;

    //INIT -- Taken from OpenVR sample
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::IVRSystem* system_ = vr::VR_Init(&eError, vr::VRApplication_Other);
    if (eError != vr::VRInitError_None)
    {
        system_ = NULL;
        char buf[1024];
        sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
        //return false;
    }

    // Find generic trackers only
    vr::TrackedDeviceClass lookForClass = TrackedDeviceClass::TrackedDeviceClass_GenericTracker;

    // Store found trackers in vector
    std::vector<vr::TrackedDeviceIndex_t> genericTrackerIndices;
    vtp::FindTrackedDevicesOfClass(system_, lookForClass, genericTrackerIndices);
    if (genericTrackerIndices.empty())
    {
        std::cerr << "No tracked devices found!";
        //return -1;
    }

    if (SGCore::DeviceList::SenseCommRunning()) //check if the Sense Comm is running. If not, warn the end user.
    {
        SGCore::SG::SenseGlove testGlove;

        if (SGCore::SG::SenseGlove::GetSenseGlove(testGlove)) //retrieves the first Sense Glove it can find. Returns true if one can be found
        {
            std::cout << "Activating " << testGlove.ToString() << " on key press." << std::endl;
            //system("pause");
            float maxGlove = 196;
            float maxGpr = 67;
            float speed = 13.4;
            float minGlove = 0;
            float minGrp = 0;
            float grp_previous, grp_current, time;
            grp_previous = 0;
            float tol = 5;
            // Loop through trackers and get more info
            for (auto trackerIdxIt = genericTrackerIndices.begin(); trackerIdxIt != genericTrackerIndices.end(); trackerIdxIt++)
            {
                vr::TrackedDeviceIndex_t& trackerIdx = *trackerIdxIt;
                bool isConnected = system_->IsTrackedDeviceConnected(trackerIdx);
                std::cout << "Device Index: " << trackerIdx << std::endl;
                if (isConnected)
                {
                    // Print property strings
                    auto getPropString = [&](auto desc, auto prop) {
                        return desc + vtp::GetTrackedPropString(system_, trackerIdx, prop) + "\n";
                    };
                    std::cout << "Connected!" << std::endl;
                    std::cout << getPropString("TrackingSystemName: ", Prop_TrackingSystemName_String);
                    std::cout << getPropString("ModelNumber: ", Prop_ModelNumber_String);
                    std::cout << getPropString("SerialNumber: ", Prop_SerialNumber_String);

                    //move to starting point defined by robot joint angles
                    rtde_control.moveJ(room_testj, 0.2, 0.4, false);

                    while (rtde_receive.isConnected() && rtde_control.isConnected())
                    {
                        auto time_getpose_start = std::chrono::high_resolution_clock::now();


                        auto pose = vtp::GetTrackedDevicePose(system_, trackerIdx); //POSE VARIABLE CREATED AS AN OBJECT
                       
                        //std::cout << "Getting new Pose" << std::endl;
                        //Retrieving Glove Pose: The position / rotation of the glove, as well as its sensor angles placed in the right direction.
                        SGCore::SG::SG_GlovePose glovePose;

                        //GET NECESSARY POSE implementations in here and send them
                        vr::HmdMatrix34_t mat = pose.mDeviceToAbsoluteTracking;
                        //from https://github.com/ValveSoftware/openvr/issues/888
                        double sy = std::sqrt(std::pow(mat.m[2][1], 2) + std::pow(mat.m[2][2], 2));
                        double roll = atan2(mat.m[2][1], mat.m[2][2]);
                        double pitch = atan2(mat.m[2][0], sy);
                         //double pitch = atan2(mat.m[2][0], mat.m[0][0]);
                        double yaw = atan2(mat.m[1][0], mat.m[0][0]);
                        double x = -mat.m[0][3] - 0.7;
                        double y = mat.m[1][3];
                        double z = mat.m[2][3] - 0.25;

                        std::vector<double> poses = { x, y, z, -yaw, pitch, -roll };//according to openvr

                        //tracking data will be added to the values of tcp pose of the starting position
                        //std::vector<double> sent_poses = { room_tcp[0] - x , room_tcp[1] + z , room_tcp[2] + y , room_tcp[3]+yaw, room_tcp[4]-pitch , room_tcp[5]+roll };
                        std::vector<double> sent_poses = { room_tcp[0] - z , room_tcp[1] + x , room_tcp[2] + y , room_tcp[3] , room_tcp[4]-pitch , room_tcp[5]+roll};
                       //0.2 given because of tcp
                       
                        
                        //PREVIOUS MOVE COMMAND
                        //rtde_control.moveL(sent_poses, 0.25, 0.9, false); //MOVE THE ROBOT ACCORDING TO NEW PARAMETERS
                        
                        //This condition is just written in case there are problems with YAW(RX) rotation.
                       // if(yaw> 1.3089){ //if yaw is bigger than 75 degs
                            std::vector<double> joint_angles = rtde_control.getInverseKinematics(sent_poses);
                            std::vector<double> new_jointangles = { joint_angles [0],joint_angles [1],joint_angles [2],joint_angles[3] ,joint_angles[4], 0.7+yaw };
                          
                            
                            auto time_getpose_end = std::chrono::high_resolution_clock::now();
                            

                            auto time_moverobot_start = std::chrono::high_resolution_clock::now();


                           // std::this_thread::sleep_for(std::chrono::milliseconds(10));
                            rtde_control.moveJ(new_jointangles);
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                          
                            
                            auto time_moverobot_end = std::chrono::high_resolution_clock::now();
                       // }
                        //else
                       // {//if yaw is less than 75 degs
                           // rtde_control.moveL(sent_poses, 0.25, 0.9, false); //MOVE THE ROBOT ACCORDING TO NEW PARAMETERS  
                        //}


                        std::cout << "TCP POSE is: " << std::endl;
                        for (const auto& e : rtde_receive.getActualTCPPose())
                            std::cout << e << " ";
                        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        std::cout << std::endl;

                        std::cout << " Actual Q is: " << std::endl;
                        for (const auto& d : rtde_receive.getActualQ())
                            std::cout << d << " ";
                        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        std::cout << std::endl;

                        // vtp::PrintTrackedDevicePose(pose);
                       
                        auto time_getdist_start = std::chrono::high_resolution_clock::now();


                        if (testGlove.GetGlovePose(glovePose))
                        {
                            //HERE WE GET THE DISTANCE BETWEEN TWO FINGERS FROM SENSEGLOVE
                            


                            //If we wish to calculate hand variables, we need a "hand profile" to tell the Sense Glove our hand lengths.
                            SGCore::SG::SG_HandProfile handProfile = SGCore::SG::SG_HandProfile::Default(testGlove.IsRight()); //create a default profile, either left or right.
                            SGCore::Kinematics::BasicHandModel handModel = SGCore::Kinematics::BasicHandModel::Default(testGlove.IsRight()); //create a default profile, either left or right.
                            std::vector<SGCore::Kinematics::Vect3D> tipPositions = glovePose.CalculateFingerTips(handProfile); //calculates fingertip position


                            std::cout << std::endl;

                            float dThumbMiddle = tipPositions[0].DistTo(tipPositions[2]) - 20; //calculates the distance between thumb (0) and middle finger (1), in mm.
                            std::cout << "The distance between THUMB and MIDDLE finger is " << std::to_string(dThumbMiddle) << "mm." << std::endl;
                            //20 is given as an offset due to thickness of fingers. Normally distTo() calculates distance between sensors of glove fingertips.
                            std::this_thread::sleep_for(std::chrono::milliseconds(8));


                            auto time_getdist_end = std::chrono::high_resolution_clock::now();


                            // grp_current = myMap(dThumbMiddle);
                            grp_current = (dThumbMiddle - minGlove) * (maxGpr - minGrp) / (maxGlove - minGlove) + minGrp;
                            time = 1000 * abs(grp_current - grp_previous) / speed;
                            
                            auto time_opengrip_start = std::chrono::high_resolution_clock::now();

                            if ((grp_current - grp_previous) > tol) {
                                rtde_io104.setAnalogOutputVoltage(0, 0.3);
                                std::cout << "OPEN THE GRIPPER" << std::endl;
                                std::this_thread::sleep_for(std::chrono::milliseconds((int)time));
                                rtde_io104.setAnalogOutputVoltage(0, 0);
                            }

                            if ((grp_current - grp_previous) < -tol) {
                                rtde_io86.setAnalogOutputVoltage(0, 0.3);
                                std::cout << "CLOSE THE GRIPPER" << std::endl;
                                std::this_thread::sleep_for(std::chrono::milliseconds((int)time));
                                rtde_io86.setAnalogOutputVoltage(0, 0);
                            }
                           
                            
                            auto time_opengrip_end = std::chrono::high_resolution_clock::now();

                            //delay time parameters
                            std::chrono::duration<double, std::milli> tracker_getpose  = time_getpose_end - time_getpose_start;
                            std::cout << "TRACKER GET POSE time: " << tracker_getpose.count() / 1000 << "SECONDS" << std::endl;

                            std::chrono::duration<double, std::milli> moverobot = time_moverobot_end - time_moverobot_start ;
                            std::cout << "MOVEROBOT TIME: " << moverobot.count() / 1000 << "SECONDS" << std::endl;

                            std::chrono::duration<double, std::milli> getdist_time = time_getdist_end - time_getdist_start;
                            std::cout << "GET SENSEGLOVE DIST TIME: " << getdist_time.count() / 1000 << "SECONDS" << std::endl; 

                            std::chrono::duration<double, std::milli> opengrip_time = time_opengrip_end - time_opengrip_start;
                            std::cout << "GRIPPER OPEN TIME: " << opengrip_time.count() / 1000 << "SECONDS" << std::endl;

                            std::chrono::duration<double, std::milli> system_time = time_opengrip_end - time_getpose_start;
                            std::cout << "SYSTEM TIME: " << system_time.count() / 1000 << "SECONDS" << std::endl;

                            grp_previous = grp_current;

                            std::cout << "END OF THE CYCLE,END OF THE CYCLE,END OF THE CYCLE,END OF THE CYCLE,END OF THE CYCLE" << std::endl;
                            std::cout << std::endl;
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(8)); //wait for 8ms.
                    }
                }
                else
                {
                    std::cout << "Error:Not connected!" << std::endl;
                }
                std::cout << "--------------" << std::endl;
            }

            // SHUTDOWN
            vr::VR_Shutdown();
            system_ = NULL;
            //return 0;

        }
    }
    else
        std::cout << "SenseComm is not running. Please start it and try again." << std::endl;

    std::cout << "=======================================" << std::endl;
    std::cout << "Press any key to exit." << std::endl;
    system("pause");
}


