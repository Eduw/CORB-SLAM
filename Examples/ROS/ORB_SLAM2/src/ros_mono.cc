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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    //W
    void GrabImage2(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;

    double total1 = 0;
    double total2 = 0;

    int index1 = 0;
    int index2 = 0;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    string strSettingsFile2; //W

    if(argc == 3) //W
    {
        strSettingsFile2 = argv[2];
    }
    else if(argc == 4) //W
    {
        strSettingsFile2 = argv[3];
    }
    else
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2], strSettingsFile2,ORB_SLAM2::System::MONOCULAR,true); //W added 2nd settings file


    //W
    cout << endl << "System initialized" << endl;

    ImageGrabber igb(&SLAM);

    //W
    cout << endl << "Imagegraber initialized" << endl;


    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    //W
    ros::Subscriber sub2 = nodeHandler.subscribe("/camera/image_raw2", 1, &ImageGrabber::GrabImage2,&igb);
    //W
    cout << endl << "Grabimage executed" << endl;

    //ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    //spinner.spin();

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveSeperateKeyFrameTrajectoryTUM("KeyFrameTrajectory1.txt","KeyFrameTrajectory2.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //cout << "Grabimage1" << endl;
//#ifdef COMPILEDWITHC11
//    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//#else
//    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
//#endif
    //cout.precision(15);
    //cout << cv_ptr->header.stamp.toSec() << endl;
      mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());


//#ifdef COMPILEDWITHC11
//    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//#else
//    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
//#endif
//    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//    total1 = total1+ttrack;
//    index1 = index1 + 1;
//    if(index1==500) {
//        cout << "Mean total 1: " << total1 / 500 << endl;
//        index1 = 0;
//        total1=0;
//    }
    //cout << "Grabimage1 end" << endl ;
    //cout << ttrack << endl;
}

//W
void ImageGrabber::GrabImage2(const sensor_msgs::ImageConstPtr& msg)
{

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
//   // cout << "Grabimage2" << endl;
//#ifdef COMPILEDWITHC11
//    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//#else
//    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
//#endif
//    cout.precision(15);
//    cout << cv_ptr->header.stamp.toSec() << endl;

    mpSLAM->TrackMonocular2(cv_ptr->image,cv_ptr->header.stamp.toSec());
//#ifdef COMPILEDWITHC11
//    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//#else
//    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
//#endif
//    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//
//    //cout << "Grabimage2 end" << endl ;
//    //cout << ttrack << endl;
//
//    total2 = total2+ttrack;
//    index2 = index2 + 1;
//    if(index2==500) {
//        cout << "Mean total 2: " << total2 / 500 << endl;
//        index2 = 0;
//        total2=0;
//    }
}


