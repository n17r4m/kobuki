/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <leg_tracker/laser_processor.h>
#include <leg_tracker/calc_leg_features.h>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <math.h>       /* atan2 */


using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;


const int MIN_POINTS_PER_CLUSTER = 3;
const double CONNECTED_THRESH = 0.13; // 0.06;


#define PI 3.14159265


class ExtractPositiveTrainingClusters
{
public:
  TransformListener tfl_;
  string laser_frame_;
  string scan_topic_;

  ScanMask mask_;

  int mask_count_;

  int feat_count_;

  ros::NodeHandle nh_;

  char save_bag_file_[100];
  char load_bag_file_[200];

  int min_angle_,
      max_angle_,
      max_dist_;


  ExtractPositiveTrainingClusters(int argc, char **argv) 
  {
    save_bag_file_[0] = 0;
    load_bag_file_[0] = 0;


    if(argc == 8)
    {
      strncpy(load_bag_file_,argv[1],200);
      strncpy(save_bag_file_,argv[2],100);
      scan_topic_ = argv[3];
      laser_frame_ = argv[4]; 
      min_angle_ = atoi(argv[5]);
      max_angle_ = atoi(argv[6]);
      max_dist_ =  atoi(argv[7]);
    }
    else 
    {
      ROS_ERROR("incorrect number of arguments passed in when extract_positive_training_clusters_from_rosbag node was launched");
    }
  }


  ~ExtractPositiveTrainingClusters()
  {
  }


  void extract()
  {
    rosbag::Bag save_bag;
    save_bag.open(save_bag_file_, rosbag::bagmode::Write);


    rosbag::Bag load_bag;
    load_bag.open(load_bag_file_, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string(scan_topic_)); // TODO topic put as command line argument
    rosbag::View view(load_bag, rosbag::TopicQuery(topics)); 
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      sensor_msgs::LaserScan::ConstPtr scan_msg = m.instantiate<sensor_msgs::LaserScan>();
      if (scan_msg != NULL)
      {
        // Could not get scan_msg to be passed direction to laser_processor functions, so we create a new LaserScan message that can be
        // TODO see SRS_leg_detector for cleaner way of doing this
        sensor_msgs::LaserScan scan;
        scan.header = scan_msg->header;
        scan.angle_min = scan_msg->angle_min;
        scan.angle_max = scan_msg->angle_max;
        scan.angle_increment = scan_msg->angle_increment;
        scan.time_increment = scan_msg->time_increment;
        scan.scan_time = scan_msg->scan_time;
        scan.range_min = scan_msg->range_min;
        scan.range_max = scan_msg->range_max;
        scan.ranges = scan_msg->ranges;
        scan.intensities = scan_msg->intensities;

        ScanProcessor processor(scan,mask_);
        processor.splitConnected(CONNECTED_THRESH);
        processor.removeLessThan(MIN_POINTS_PER_CLUSTER);

        geometry_msgs::PoseArray leg_cluster_positions;
        leg_cluster_positions.header.frame_id = "laser_frame";

        for (list<SampleSet*>::iterator i = processor.getClusters().begin();
                                        i != processor.getClusters().end();
                                        i++)
        {
          tf::Point cluster_position = (*i)->center();

          double x_pos = cluster_position[0],
                 y_pos = cluster_position[1],
                 angle = atan2(y_pos,x_pos) * 180 / PI,
                 dist_abs = sqrt(x_pos*x_pos + y_pos*y_pos);


          if (angle > min_angle_ and angle < max_angle_ and dist_abs < max_dist_) 
          {
            geometry_msgs::Pose new_leg_cluster_position;
            new_leg_cluster_position.position.x = cluster_position[0];
            new_leg_cluster_position.position.y = cluster_position[1];
            leg_cluster_positions.poses.push_back(new_leg_cluster_position);
          }
        }
        if (leg_cluster_positions.poses.size())  // at least one person has been found in current scan
        {
          // save position of person to be used later for training 
          save_bag.write("/leg_cluster_positions", ros::Time::now(), leg_cluster_positions); 


           // Save an identical scan to one we recieved but with a different topic and reference frame.
          sensor_msgs::LaserScan new_scan; 
          new_scan.header.frame_id = "laser_frame";  // TODO should this just be /world?
          new_scan.angle_min = scan_msg->angle_min;
          new_scan.angle_max = scan_msg->angle_max;
          new_scan.angle_increment = scan_msg->angle_increment;
          //new_scan.time_increment = scan_msg->time_increment;  // these ones give me segfaults
          //new_scan.scan_time = scan_msg->scan_time;
          new_scan.range_min = scan_msg->range_min;
          new_scan.range_max = scan_msg->range_max;
          new_scan.ranges = scan_msg->ranges;
          save_bag.write("/training_scan", ros::Time::now(), new_scan);



          // save a marker of the position of the cluster we extracted. Just used so we can playback the rosbag file and visually verify the correct clusters have been extracted
          visualization_msgs::MarkerArray ma;
          int MAX_NUM_MARKERS = 10;
          for (int i = 0;
              i < leg_cluster_positions.poses.size();
              i++)
          {
            // display cluster to view in rviz
            visualization_msgs::Marker m;
            //m.header.stamp = positive_clusters.header.stamp;
            m.header.frame_id = "laser_frame";
            m.ns = "LEGS";
            m.id = i;
            m.type = m.SPHERE;
            m.pose.position.x = leg_cluster_positions.poses[i].position.x;
            m.pose.position.y = leg_cluster_positions.poses[i].position.y;
            m.pose.position.z = 0.1;
            m.scale.x = .2;
            m.scale.y = .2;
            m.scale.z = .2;
            m.color.a = 1;
//            m.lifetime = ros::Duration(0.2);
            m.color.b = 0.0;
            m.color.r = 1.0;
            ma.markers.push_back(m);
          }
          for (int i = leg_cluster_positions.poses.size();  // clearing any lingering markers from Rviz
              i <= MAX_NUM_MARKERS;
              i++)
          {
            visualization_msgs::Marker m;
            m.header.frame_id = "laser_frame";
            m.ns = "LEGS";
            m.id = i;
            m.action = visualization_msgs::Marker::DELETE;
            ma.markers.push_back(m);
          }
          save_bag.write("/visualization_marker_array", ros::Time::now(), ma);
        }
      }
    }
    load_bag.close();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv,"extract_positive_leg_clusters");

  ExtractPositiveTrainingClusters etc(argc, argv);
  etc.extract();
   
  return 0;
}

