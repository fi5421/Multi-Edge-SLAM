/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <signal.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include"../../../include/System.h"

// Edge-SLAM
#include <string>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

void signal_callback_handler(int signum)
{
    cout << "Caught signal " << signum << endl;
    // exit(0);
}


int main(int argc, char **argv)
{
    // Edge-SLAM: check arguments
    // Check run type and convert to lowercase
    std::string DoRect(argv[3]);
    std::transform(DoRect.begin(), DoRect.end(), DoRect.begin(), ::tolower);
    std::string RunType(argv[4]);
    std::transform(RunType.begin(), RunType.end(), RunType.begin(), ::tolower);
    ros::init(argc, argv, RunType);
    ros::start();
    if((argc != 5) || ((DoRect.compare("true") != 0) && (DoRect.compare("false") != 0)) || ((RunType.compare("client") != 0) && (RunType.compare("server") != 0)))
    {
        cerr << endl << "Usage: rosrun Edge_SLAM Stereo VOC_PATH SETTINGS_PATH DO_RECTIFY(true|false) RUN_TYPE(client|server)" << endl;
        ros::shutdown();
        return 1;
    }

    // Edge-SLAM
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],RunType,ORB_SLAM2::System::STEREO,true);

    // Edge-SLAM: check client or server
    if (RunType.compare("client") == 0)
    {
        ImageGrabber igb(&SLAM);

        stringstream ss(argv[3]);
        ss >> boolalpha >> igb.do_rectify;

        if(igb.do_rectify)
        {
            cerr<<"in if statement\n";
            // Load settings related to stereo calibration
            cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
            if(!fsSettings.isOpened())
            {
                cerr << "ERROR: Wrong path to settings" << endl;
                return -1;
            }
            cerr<<"here 1\n";
            // cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
            // cerr<<"here 1.1\n";
            // fsSettings["LEFT.K"] >> K_l;
            // cerr<<"here 1.1.2\n";
            
            // fsSettings["RIGHT.K"] >> K_r;
            // cerr<<"here 1.2\n";
            // fsSettings["LEFT.P"] >> P_l;
            // fsSettings["RIGHT.P"] >> P_r;
            // cerr<<"here 1.3\n";

            // fsSettings["LEFT.R"] >> R_l;
            // fsSettings["RIGHT.R"] >> R_r;
            // cerr<<"here 1.4\n";

            // fsSettings["LEFT.D"] >> D_l;
            // fsSettings["RIGHT.D"] >> D_r;

            cv::Mat K_l = (cv::Mat_<double>(3,3)<<458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0);
            cv::Mat K_r = (cv::Mat_<double>(3,3)<<457.587, 0.0, 379.999, 0.0, 456.134, 255.238, 0.0, 0.0, 1);
            cv::Mat P_l = (cv::Mat_<double>(3,4)<<435.2046959714599, 0, 367.4517211914062, 0,  0, 435.2046959714599, 252.2008514404297, 0,  0, 0, 1, 0);
            cv::Mat P_r = (cv::Mat_<double>(3,4)<<435.2046959714599, 0, 367.4517211914062, -47.90639384423901, 0, 435.2046959714599, 252.2008514404297, 0, 0, 0, 1, 0);
            cv::Mat R_l = (cv::Mat_<double>(3,3)<<0.999966347530033, -0.001422739138722922, 0.008079580483432283, 0.001365741834644127, 0.9999741760894847, 0.007055629199258132, -0.008089410156878961, -0.007044357138835809, 0.9999424675829176);
            cv::Mat R_r = (cv::Mat_<double>(3,3)<<0.9999633526194376, -0.003625811871560086, 0.007755443660172947, 0.003680398547259526, 0.9999684752771629, -0.007035845251224894, -0.007729688520722713, 0.007064130529506649, 0.999945173484644);
            cv::Mat D_l = (cv::Mat_<double>(1,5)<<-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0);
            cv::Mat D_r = (cv::Mat_<double>(1,5)<<-0.28368365, 0.07451284, -0.00010473, -3.555907e-05, 0.0);
            
            cerr<<"here 2\n";
            int rows_l = fsSettings["LEFT.height"];
            int cols_l = fsSettings["LEFT.width"];
            int rows_r = fsSettings["RIGHT.height"];
            int cols_r = fsSettings["RIGHT.width"];
            cerr<<"here 3\n";

            if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                    rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
            {
                cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
                return -1;
            }
            cerr<<"here 4\n";
            cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
            cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
             cerr<<"out if statement\n";
        }

        ros::NodeHandle nh;

        message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
        message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
        sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

        ros::spin();

        // Edge-SLAM: split shutdown between client and server
        // Stop all threads
        SLAM.ClientShutdown();
    }
    else
    {
        ros::spin();

        // Edge-SLAM: split shutdown between client and server
        // Stop all threads
        SLAM.ServerShutdown();

        // Save camera trajectory
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
        // SLAM.SaveTrajectoryKITTI("KeyFrame_Traj_KITTI.txt");
    }

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}

