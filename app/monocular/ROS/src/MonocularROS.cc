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

#include "MonocularROS.h"

MonocularROS::MonocularROS(){
}

bool MonocularROS::init(const std::string _configPath){

    const string &strSettingPath = _configPath;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int ReuseMap = fSettings["is_ReuseMap"];
    const string strMapPath = fSettings["ReuseMap"];

    bool bReuseMap = false;
    if (1 == ReuseMap)
        bReuseMap = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM_ = new ORB_SLAM2::System(strORBvoc,strCamSet,ORB_SLAM2::System::MONOCULAR,true, bReuseMap,strMapPath);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;


    const string imageTopic = fSettings["image_topic"];
    imgSub_ = nh_.subscribe<sensor_msgs::Image>(imageTopic, 1, [&](const sensor_msgs::Image::ConstPtr& _msg){
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(_msg);
        }
            catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        // Pass the image to the SLAM system
        SLAM_->TrackMonocular(cv_ptr->image,0);

        if(SLAM_->isShutdown())
            return;
    });

    return true;
}

void MonocularROS::shutdown(){
    // Stop all threads
    SLAM_->Shutdown();
    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("/home/marrcogrova/Documents/KeyFrameTrajectory.txt");
}