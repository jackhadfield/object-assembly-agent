#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <object_assembly_msgs/ConnectionInfoList.h>


class AssemblySubtask
{

public:
    AssemblySubtask(int id,
                    int num_checks,
                    std::vector<int> first_object,
    		        std::vector<int> second_object,
                    std::string frame,
    		        tf::Vector3 relative_position,
                    tf::Quaternion relative_orientation,
                    std::vector<double> margin,
    		        std::string connection_type,
                    std::vector<std::string> object_symmetries,
                    tf::Vector3 up_vector);

    bool evaluate_subtask(
        std::vector<geometry_msgs::Pose> current_object_poses,
        double &score);

    bool evaluate_option(int option_id,
                         tf::Transform first_object_pose,
                         tf::Transform second_object_pose);

    bool compare_tf_vectors(tf::Vector3 v1,
                              tf::Vector3 v2,
                              double x_margin,
                              double y_margin,
                              double z_margin,
                              double &score);

    bool compare_tf_quaternions(tf::Quaternion q1,
                                  tf::Quaternion q2,
                                  double margin,
                                  double &score);

private:
    void sort_vector3(tf::Vector3 &v);

    bool evaluate_no_symmetry(tf::Transform first_object_pose,
                              tf::Transform second_object_pose,
                              double &score);

    bool evaluate_cubic_cubic(tf::Transform first_object_pose,
                              tf::Transform second_object_pose,
                              double &score);

public:
    geometry_msgs::Pose connection_pose;

private:
    const int subtask_id_; //starting at 0
    bool subtask_complete_ = false;
    const int num_checks_;
    int checks_ = 0;
    std::vector<int> first_object_;
    std::vector<int> second_object_;
    
    std::string connection_type_;

    std::string frame_;
    tf::Vector3 relative_position_; //TODO fix initialization
    tf::Quaternion relative_orientation_;
    
    //margin[0,1,2]: position margin, margin[3,4,5,6]: orientation margin
    std::vector<double> margin_;

    std::string first_object_symmetry_;
    std::string second_object_symmetry_;
    tf::Vector3 up_vector_;
    tf::Vector3 rot_axis_;
    double rot_angle_;
};
