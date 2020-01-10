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


    mapPub_    = nh_.advertise<sensor_msgs::PointCloud2>("/inspector/map", 1);
    imgPub_    = nh_.advertise<sensor_msgs::Image> ("inspector/debug_image", 1);
    posePub_   = nh_.advertise<geometry_msgs::PoseStamped> ("inspector/pose", 1);
    pathVOPub_ = nh_.advertise<nav_msgs::Path> ("inspector/path", 1);

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

        publishROS();

        if(SLAM_->isShutdown())
            return;
    });

    return true;
}

template <typename topicType>
void MonocularROS::publishMessage(ros::Publisher _pub , topicType _msg){
  _msg.header.frame_id = "map"; 
  _msg.header.stamp=ros::Time::now();
  _pub.publish(_msg);
  return;
}

void MonocularROS::publishROS(){

    ORB_SLAM2::Tracking::eTrackingState state = SLAM_->GetSLAMState();
    if (state == ORB_SLAM2::Tracking::eTrackingState::OK){

        cv::Mat position = SLAM_->GetCurrentPosition();
        if (!position.empty()) {

            tf::Transform grasp_tf = TransformFromMat (position);
            tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, ros::Time::now(), "map");
            geometry_msgs::PoseStamped pose_msg;
            tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
            posePub_.publish(pose_msg);

            VOpath_.poses.push_back(pose_msg);
            publishMessage<nav_msgs::Path>(pathVOPub_,VOpath_);
        }
    }

    cv::Mat image = SLAM_->DrawCurrentFrame();
    const sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_msg->header.frame_id = "camera_link";
    image_msg->header.stamp=ros::Time::now();
    imgPub_.publish(image_msg);

    sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (SLAM_->GetAllMapPoints());
    publishMessage<sensor_msgs::PointCloud2>(mapPub_,cloud);
}


sensor_msgs::PointCloud2 MonocularROS::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> _mapPoints) {
  if (_mapPoints.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int numChannels = 3; // x y z

  cloud.height = 1;
  cloud.width = _mapPoints.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = numChannels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(numChannels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<numChannels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[numChannels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (_mapPoints.at(i)->nObs >= minObservationsPerPoint_) {
      data_array[0] = _mapPoints.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* _mapPoints.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* _mapPoints.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, numChannels*sizeof(float));
    }
  }

  return cloud;
}

tf::Transform MonocularROS::TransformFromMat(cv::Mat _transformation) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = _transformation.rowRange(0,3).colRange(0,3);
  translation = _transformation.rowRange(0,3).col(3);

  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}

void MonocularROS::shutdown(){
    // Stop all threads
    SLAM_->Shutdown();
    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("/home/marrcogrova/Documents/KeyFrameTrajectory.txt");
}