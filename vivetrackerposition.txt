#include <vector>
#include <iostream>
#include "openvr.h"
#include <chrono>
#include <thread>
#include <math.h>

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

        // double mn00 = std::stod(mn(0, 0));


        //double a, b, c, d, e, f;

         //const std::vector<double> movel_pose = { x, y, z, yaw, pitch, roll };
         //// First argument is the pose 6d vector followed by speed and acceleration
     //rtde_control.moveL({ -0.143, -0.435, 0.20, -0.001, 3.12, 0.04 }, 0.2, 0.2);

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
        double pi = 3.141592;

        bool& poseValid = pose.bPoseIsValid;
        std::cout << "Pose: ";
        if (poseValid)
        {
            // Print matrix

            vr::HmdMatrix34_t mat = pose.mDeviceToAbsoluteTracking;
            /*----from triad.py
            double yaw = 180 / pi * atan2(mat.m[1][0], mat.m[0][0]);
            double pitch = 180 / pi*atan2(mat.m[2][0], mat.m[0][0]);
            double roll = 180 / pi*atan2(mat.m[2][1], mat.m[2][2]);
            double x = mat.m[0][3];
            double y = mat.m[1][3];
            double z = mat.m[2][3]+1.6; */

            //from https://github.com/ValveSoftware/openvr/issues/888
            double sy = std::sqrt(std::pow(mat.m[0][0], 2) + std::pow(mat.m[0][1], 2));
            double roll = 180 / pi * atan2(mat.m[1][2], mat.m[2][2]);
            double pitch = 180 / pi * atan2(mat.m[0][2], sy);
            double yaw = 180 / pi * atan2(mat.m[0][1], mat.m[0][0]);
            double x = mat.m[0][3];
            double y = mat.m[1][3]-0.8;
            double z = mat.m[2][3] + 1.38;
            //std::vector<double> poses = { x, y, z, yaw, roll, pitch };
            std::vector<double> poses = { x, y, z, roll, pitch, yaw };
            // 
                                            //x,y,z,yaw,pitch,roll
                                            //x,y,z,rz,ry,rx
                                            //x,y,z,rx,ry,rz needed for movel command
                                            // rotation around front-back axis is called (roll)
                                            // rotation around vertical axis is called (yaw)
                                            // rotation around side-to-side is called (pitch)
                                            //
                                            // according to text above;
                                            // after actual experiment: roll --> yaw , pitch--> roll, yaw --> pitch
                                            // gives more accurate result by taking rpy representation into consideration.
                                            // 
                                            // for robot: x is forward-backward, y is left-right, z is up-down directions
                                            // for vivetrackers x is forward-backward, y is up-down, z is left-right
                                            // switching y and z in the vector will solve our problem for 
            //{x,z,y}
            std::cout << "X , Y , Z , Rx , Ry , Rz Representation: \n" << std::endl;

            for (int i = 0; i < poses.size(); i++) {
                std::cout << poses[i] << ", ";
                //std::cout << x  << "," <<y <<"," << z<<"," <<yaw << ","<<pitch <<"," <<yaw <<std::endl;
            }
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
/**/
//static void ViveTrackerPosition()
int main(int argc, char* argv)
{
    using namespace vr;

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

            // Print device pose info
            while (true) {

                auto pose = vtp::GetTrackedDevicePose(system_, trackerIdx);
                //get necessray pose implementations in here and send them



                vtp::PrintTrackedDevicePose(pose);

                std::this_thread::sleep_for(std::chrono::milliseconds(200));
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