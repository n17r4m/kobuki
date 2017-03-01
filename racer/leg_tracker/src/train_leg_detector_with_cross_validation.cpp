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


#include <leg_tracker/laser_processor.h>
#include <leg_tracker/calc_leg_features.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/PoseArray.h>

using namespace std;
using namespace laser_processor;
using namespace ros;



// TODO retrain using test data!!!!


const int MIN_POINTS_PER_CLUSTER = 3;
const double CONNECTED_THRESH = 0.13; // 0.06; 
const int UNDERSAMPLE_FACTOR = 25;//1;//10; //50;



class TrainLegDetector
{
public:
  ScanMask mask_;
  int mask_count_;

  char* scan_topic;

  CvRTrees forest_;

  CvSVM svm_;

  int feat_count_;
  
  float training_error_;

  // ----------------------------------------------------------------------------------------------
  TrainLegDetector() : mask_count_(0), feat_count_(0)
  {
  }


  // ----------------------------------------------------------------------------------------------
  void loadPosData(const char* rosbag_file, 
                   const char* scan_topic, 
                   vector< vector<float> > &load_into_data)
  {
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string(scan_topic)); 
    topics.push_back(std::string("/leg_cluster_positions")); // TODO topic put as command line argument
    rosbag::View view(bag, rosbag::TopicQuery(topics)); 

    geometry_msgs::PoseArray positive_clusters;

    int message_num = 0;
    int initial_pos_data_size = (int)load_into_data.size();
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      geometry_msgs::PoseArray::ConstPtr pose_array_msg = m.instantiate<geometry_msgs::PoseArray>();
      if (pose_array_msg != NULL)
      {
        positive_clusters = *pose_array_msg;
      }


      sensor_msgs::LaserScan::ConstPtr scan_msg = m.instantiate<sensor_msgs::LaserScan>();
      if (scan_msg != NULL and positive_clusters.poses.size())
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


        if (mask_count_++ < 20)
          mask_.addScan(scan);
        else
        {
          ScanProcessor processor(scan,mask_);
          processor.splitConnected(CONNECTED_THRESH);
          processor.removeLessThan(MIN_POINTS_PER_CLUSTER);
 
          for (list<SampleSet*>::iterator i = processor.getClusters().begin();
             i != processor.getClusters().end();
             i++)
           {
             tf::Point cluster_position = (*i)->center();

             for (int j = 0; 
                      j < positive_clusters.poses.size();
                      j++)
             {
               double dist_x = positive_clusters.poses[j].position.x - cluster_position[0],
                      dist_y = positive_clusters.poses[j].position.y - cluster_position[1],             
                      dist_abs = sqrt(dist_x*dist_x + dist_y*dist_y);

               if (dist_abs < 0.0001)
               {
                 load_into_data.push_back(calcLegFeatures(*i, scan));
                 break;                                           
               }
             }
           }
         }
         message_num++;
       } 
     }
     bag.close();

     printf("\t Got %i scan messages, %i samples, %i mask_count_ from %s  \n",message_num, (int)load_into_data.size() - initial_pos_data_size, mask_count_,  rosbag_file);
  } 


  // ----------------------------------------------------------------------------------------------
  void loadNegData(const char* rosbag_file, 
                   const char* scan_topic, 
                   vector< vector<float> > &load_into_data)
  {
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string(scan_topic)); // TODO topic put as command line argument
    rosbag::View view(bag, rosbag::TopicQuery(topics)); 

    int message_num = 0;
    int initial_neg_data_size = (int)load_into_data.size();
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


        if (mask_count_++ < 20)
          mask_.addScan(scan);
        else
        {
          ScanProcessor processor(scan,mask_);
          processor.splitConnected(CONNECTED_THRESH);
          processor.removeLessThan(MIN_POINTS_PER_CLUSTER);
 
          for (list<SampleSet*>::iterator i = processor.getClusters().begin();
                                          i != processor.getClusters().end();
                                          i++)
          {
            if (rand() % UNDERSAMPLE_FACTOR == 0) // TODO one way of undersampling the negative class
              load_into_data.push_back(calcLegFeatures(*i, scan));                 
          }
        }
        message_num++;
      } 
    }
    bag.close();

    printf("\t Got %i scan messages, %i samples, %i mask_count_ from %s  \n",message_num, (int)load_into_data.size() - initial_neg_data_size, mask_count_,  rosbag_file);
  } 


  // ----------------------------------------------------------------------------------------------
  void train(const vector< vector<float> > &train_pos_data, 
             const vector< vector<float> > &train_neg_data)
  {
    int sample_size = train_pos_data.size() + train_neg_data.size();
    feat_count_ = train_pos_data[0].size();

    CvMat* cv_data = cvCreateMat( sample_size, feat_count_, CV_32FC1);
    CvMat* cv_resp = cvCreateMat( sample_size, 1, CV_32S);

    // Put positive data in opencv format.
    int j = 0;
    for (vector< vector<float> >::const_iterator i = train_pos_data.begin();
         i != train_pos_data.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];
      
      cv_resp->data.i[j] = 1;
      j++;
    }

    // Put negative data in opencv format.
    for (vector< vector<float> >::const_iterator i = train_neg_data.begin();
         i != train_neg_data.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];
      
      cv_resp->data.i[j] = -1;
      j++;
    }

    CvMat* var_type = cvCreateMat( 1, feat_count_ + 1, CV_8U );
    cvSet( var_type, cvScalarAll(CV_VAR_ORDERED));
    cvSetReal1D( var_type, feat_count_, CV_VAR_CATEGORICAL );
    
    // Random forest training parameters
    // One parameter not set here is the factor to undersample the negative examples. See UNDERSAMPLE_FACTOR at top of this file
    float priors[] = {1.0, 1.0};

    CvRTParams fparam(10000,              // max depth of tree
                      2,                  // min sample count to split tree
                      0,                  // regression accuracy (?)
                      false,              // use surrogates (?)
                      1000,               // max categories
                      priors,             // priors
                      true,               // calculate variable importance 
                      2,                  // number of active vars for each tree node
                      100,                // max trees in forest
                      0.001f,             // forest accuracy (sufficient OOB error)
                      CV_TERMCRIT_ITER);  // termination criteria. CV_TERMCRIT_ITER = once we reach max number of forests
    forest_.train( cv_data,                // train data 
                  CV_ROW_SAMPLE,          // tflag
                  cv_resp,                // responses (i.e. labels)
                  0,                      // varldx (?)
                  0,                      // sampleldx (?)
                  var_type,               // variable type 
                  0,                      // missing data mask
                  fparam);                // parameters

    training_error_ = 100.0*forest_.get_train_error();

//    cv::Mat var_importance = forest.getVarImportance();
//    printf("Feature importance: \n"); 
//    for (int i=0; i<var_importance.size().width; i++)
//      printf("  %i: %f%%\n", i+1, 100.0*var_importance.at<float>(0,i));
//    printf("\n");


    cvReleaseMat(&cv_data);
    cvReleaseMat(&cv_resp);
    cvReleaseMat(&var_type);
  }

  // ----------------------------------------------------------------------------------------------
  void test(const vector< vector<float> > &test_pos_data, 
            const vector< vector<float> > &test_neg_data,
            int &correct_pos,
            int &correct_neg)
  {
    CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);
 
    // test on positive test examples
    int test_pos_right = 0;
    int test_pos_total = 0;
    int dist_hist_pos_right[20];
    int dist_hist_pos_total[20];
    std::fill_n(dist_hist_pos_right, 20, 0);
    std::fill_n(dist_hist_pos_total, 20, 0);
    for (vector< vector<float> >::const_iterator i = test_pos_data.begin();
         i != test_pos_data.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest_.predict( tmp_mat ) > 0)
        test_pos_right++;
      test_pos_total++;

      // construct distance histogram
      float dist = (float)((*i)[15]);
      for (int k = 0; k < 20; k++)
      {
        if (dist < (k*0.5) or k == 19)
        {
          if (forest_.predict( tmp_mat ) > 0)
            dist_hist_pos_right[k]++;
          dist_hist_pos_total[k]++;
          break;
        }
      }    
    }

    // test on negative test examples
    int test_neg_right = 0;
    int test_neg_total = 0;
    int dist_hist_neg_right[20];
    int dist_hist_neg_total[20];
    std::fill_n(dist_hist_neg_right, 20, 0);
    std::fill_n(dist_hist_neg_total, 20, 0);
    for (vector< vector<float> >::const_iterator i = test_neg_data.begin();
         i != test_neg_data.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest_.predict( tmp_mat ) < 0)
        test_neg_right++;
      test_neg_total++;
      
      
      // construct distance histogram
      float dist = (float)((*i)[15]);
      for (int k = 0; k < 20; k++)
      {
        if (dist < (k*0.5) or k == 19)
        {
          if (forest_.predict( tmp_mat ) < 0)
            dist_hist_neg_right[k]++;
          dist_hist_neg_total[k]++;
          break;
        }
      }   
    }

    correct_pos = test_pos_right;
    correct_neg = test_neg_right;

//    printf(" Positive train set: %d/%d \t\t Error: %g%%\n",pos_right, pos_total, 100.0 - 100.0*(float)(pos_right)/pos_total);
//    printf(" Negative train set: %d/%d \t\t Error: %g%%\n",neg_right, neg_total, 100.0 - 100.0*(float)(neg_right)/neg_total);
//    printf(" Combined train set: %d/%d \t Error: %g%%\n\n", pos_right + neg_right, pos_total + neg_total, 100.0 - 100.0*(float)(pos_right + neg_right)/(pos_total + neg_total));

    printf("     Positive test set:  %d/%d \t\t Error: %g%%\n",test_pos_right, test_pos_total, 100.0 - 100.0*(float)(test_pos_right)/test_pos_total);
    printf("     Negative test set:  %d/%d \t\t Error: %g%%\n\n",test_neg_right, test_neg_total, 100.0 - 100.0*(float)(test_neg_right)/test_neg_total);
//    printf(" Combined test set: %d/%d \t\t Error: %g%%\n\n", test_pos_right + test_neg_right, test_pos_total + test_neg_total, 100.0 - 100.0*(float)(test_pos_right + test_neg_right)/(test_pos_total + test_neg_total));
//    
//    printf(" \n dist_hist positive: \n");
//    for (int i = 1; i < 20; i++)
//      printf(" < %f: %i / %i \t Error: %g%%\n", i*0.5, dist_hist_pos_right[i], dist_hist_pos_total[i], 100.0 - 100.0*(float)(dist_hist_pos_right[i])/dist_hist_pos_total[i]);
//      
//    printf(" \n dist_hist negative: \n");
//    for (int i = 1; i < 20; i++)
//      printf(" < %f: %i / %i \t Error: %g%%\n", i*0.5, dist_hist_neg_right[i], dist_hist_neg_total[i], 100.0 - 100.0*(float)(dist_hist_neg_right[i])/dist_hist_neg_total[i]);


    cvReleaseMat(&tmp_mat);
  }
};

// ----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  TrainLegDetector tld;

  char save_file[100];
  save_file[0] = 0;


  vector< vector< vector<float> > > train_pos_data;
  vector< vector< vector<float> > > train_neg_data;
  vector< vector<float> > test_neg_data;
  vector< vector<float> > test_pos_data;


  // Parse command line arguements
  printf("\nLoading data...\n");
  for (int i = 1; i < argc-3; i++) // argc-3 because the last couple arguments are send in from ROS and aren't relevant
  {
    if (!strcmp(argv[i],"--pos"))
    {
      char* rosbag_file = argv[++i]; 
      char* scan_topic = argv[++i];
      tld.mask_.clear();
      tld.mask_count_ = 1000; // effectively disable masking  TODO Not sure what this does!!!!
      vector< vector<float> > new_pos_data;
      tld.loadPosData(rosbag_file, scan_topic, new_pos_data);
      train_pos_data.push_back(new_pos_data);
    }
    else if (!strcmp(argv[i],"--neg"))
    {
      char* rosbag_file = argv[++i]; 
      char* scan_topic = argv[++i];
      tld.mask_.clear();
      tld.mask_count_ = 1000; // effectively disable masking  TODO Not sure what this does!!!!
      vector< vector<float> > new_neg_data;
      tld.loadNegData(rosbag_file, scan_topic, new_neg_data);
      train_neg_data.push_back(new_neg_data);
    }
    else if (!strcmp(argv[i],"--test_pos"))
    {
      char* rosbag_file = argv[++i]; 
      char* scan_topic = argv[++i];
      tld.mask_.clear();
      tld.mask_count_ = 1000; // effectively disable masking  TODO Not sure what this does!!!!
      tld.loadPosData(rosbag_file, scan_topic, test_pos_data);
    }
    else if (!strcmp(argv[i],"--test_neg"))
    {
      char* rosbag_file = argv[++i]; 
      char* scan_topic = argv[++i];
      tld.mask_.clear();
      tld.mask_count_ = 1000; // effectively disable masking  TODO Not sure what this does!!!!
      tld.loadNegData(rosbag_file, scan_topic, test_neg_data);
    }
    else if (!strcmp(argv[i],"--save"))
    {
      if (++i < argc)
        strncpy(save_file,argv[i],100);
    }
  }

  // Error check the loaded data
  if (train_pos_data.empty() or train_neg_data.empty()) //or tld.test_data_.size() == 0)
    ROS_ERROR("data not loaded from rosbags properly \n");
  else
  {
    printf("\n  Total positive training samples: %i \t Total negative training samples: %i \n", (int)train_pos_data.size(), (int)train_neg_data.size());
    printf("  Total positive test samples: %i \t Total negative test samples: %i \n\n", (int)test_pos_data.size(), (int)test_neg_data.size());
  }

  // Cross validation
  printf("Cross validating \n");
  int cross_val_folds = (int)train_pos_data.size();  // TODO Important: This code assumes the chuncks of positive training data = chunks of negative training data. Will break easily.
  int correct_pos = 0; // for recording cross validation scores
  int correct_neg = 0;
  int total_pos = 0;
  int total_neg = 0;
  for (int fold = 0; fold < cross_val_folds; fold++) 
  {
    printf("  Cross validation fold %i \n", fold);
    
    // Populate current fold's train and test sets
    vector< vector<float> > cross_val_train_pos_data;
    vector< vector<float> > cross_val_train_neg_data;
    vector< vector<float> > cross_val_test_pos_data;
    vector< vector<float> > cross_val_test_neg_data;
    for (int i = 0; i < cross_val_folds; i++) 
    {
      // TODO note: this part's pretty hacky and assumes certain number of positive and negative bags passed in. Will break if different input bags used
      if (i != fold)
      {
        for (int j = 0; j < train_pos_data[i].size(); j++)
          cross_val_train_pos_data.push_back(train_pos_data[i][j]);
          
        for (int j = 0; j < train_neg_data[i*3].size(); j++)
          cross_val_train_neg_data.push_back(train_neg_data[i*3][j]);         
        for (int j = 0; j < train_neg_data[i*3 + 1].size(); j++)
          cross_val_train_neg_data.push_back(train_neg_data[i*3 + 1][j]);      
        for (int j = 0; j < train_neg_data[i*3 + 2].size(); j++)
          cross_val_train_neg_data.push_back(train_neg_data[i*3 +2][j]);
      }
      else
      {
        for (int j = 0; j < train_pos_data[i].size(); j++)
          cross_val_test_pos_data.push_back(train_pos_data[i][j]);
          
        for (int j = 0; j < train_neg_data[i*3].size(); j++)
          cross_val_test_neg_data.push_back(train_neg_data[i*3][j]);                   
        for (int j = 0; j < train_neg_data[i*3 + 1].size(); j++)
          cross_val_test_neg_data.push_back(train_neg_data[i*3 + 1][j]); 
        for (int j = 0; j < train_neg_data[i*3 + 2].size(); j++)
          cross_val_test_neg_data.push_back(train_neg_data[i*3 + 2][j]); 
      }
    }
    
    // Evaluate current fold
    printf("    Positive training samples on this fold: %i, Negative: %i \n", (int)cross_val_train_pos_data.size(), (int)cross_val_train_neg_data.size());
    tld.train(cross_val_train_pos_data, cross_val_train_neg_data);
    int correct_pos_fold = 0;
    int correct_neg_fold = 0;
    tld.test(cross_val_test_pos_data, cross_val_test_neg_data, correct_pos_fold, correct_neg_fold);
    correct_pos += correct_pos_fold;
    correct_neg += correct_neg_fold;
    total_pos += (int)cross_val_test_pos_data.size();
    total_neg += (int)cross_val_test_neg_data.size();
  }

  // Print cross validation results
  printf ("\nCross validation results: \n");
  printf("  Positive: %i/%i \t\t Error: %g%%\n", correct_pos, total_pos, 100.0 - 100.0*(float)(correct_pos)/total_pos);
  printf("  Negative: %i/%i \t\t Error: %g%%\n", correct_neg, total_neg, 100.0 - 100.0*(float)(correct_neg)/total_neg);
  printf("  Combined: %i/%i \t\t Error: %g%%\n\n", correct_pos+correct_neg, total_pos+total_neg, 100.0 - 100.0*(float)(correct_pos+correct_neg)/(total_pos+total_neg));
}
