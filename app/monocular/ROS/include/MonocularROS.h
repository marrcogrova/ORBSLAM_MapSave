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
#include <opencv2/core/core.hpp>
#include "System.h"

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>

class MonocularROS{

    public:
        MonocularROS();

        bool init(const std::string _configPath);

        void shutdown();

        void publishROS();
        void publishImageROS(cv::Mat _image);
        void PublishPositionAsTransform(cv::Mat _transformation);
        void PublishPositionAsPoseStamped(cv::Mat _transformation);

        tf::Transform TransformFromMat (cv::Mat _transformation);
        sensor_msgs::PointCloud2 MapPointsToPointCloud(std::vector<ORB_SLAM2::MapPoint*> _mapPoints);

    private:
        ORB_SLAM2::System *SLAM_;
        int minObservationsPerPoint_ = 2;

        ros::NodeHandle nh_;

        ros::Subscriber imgSub_;

        ros::Publisher imgPub_;
        ros::Publisher posePub_;
        ros::Publisher mapPub_;
};