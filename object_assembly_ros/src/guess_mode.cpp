//#include <Eigen/Dense>

#include <fstream>
#include <ctime>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include <assembly_task_guess_mode.hpp>

//#include <fl/util/profiling.hpp>

//#include <opi/interactive_marker_initializer.hpp>
//#include <osr/free_floating_rigid_bodies_state.hpp>
//#include <dbot/util/camera_data.hpp>
//#include <dbot/tracker/rms_gaussian_filter_object_tracker.hpp>
//#include <dbot/tracker/builder/rms_gaussian_filter_tracker_builder.hpp>

#include <dbot_ros_msgs/ObjectsState.h>
#include <object_assembly_msgs/FetchSurfaceParams.h>
#include <object_assembly_msgs/ConnectionInfoList.h>
//#include <dbot_ros/utils/ros_interface.hpp>
//#include <dbot_ros/utils/ros_camera_data_provider.hpp>

//#include "opencv2/core/eigen.hpp"
//#include "opencv2/opencv.hpp"
#include <guess_mode_ui.hpp>


class AssemblyAgentNode
{

public:

    AssemblyAgentNode(std::vector<std::string> object_names, std::vector<AssemblyTask> tasks, std::string connection_info_list_topic, std::vector<object_assembly_msgs::ConnectionInfo> connection_list, int argc, char** argv)
        : object_names_(object_names), tasks_(tasks), connection_list_(connection_list), node_handle_("~"), num_tasks_(tasks.size()), argc_(argc), argv_(argv)
    {
        std::cout << "Creating Assembly Agent Node\n";

        connection_list_publisher_ = node_handle_.advertise<object_assembly_msgs::ConnectionInfoList>(connection_info_list_topic, 0);

        std::vector<std::string> filenames;
        for(int i = 0; i< object_names_.size(); i++)
            filenames.push_back("/home/jack/catkin_ws/src/object_assembly_ros/resource/cube.obj");
        ui = new GuessModeUI(filenames);
        for (int i = 0; i < ui->meshes_.size(); i++)
        {
            for (int j = 0; j < ui->meshes_[i].vertices.size(); j++)
                ui->meshes_[i].set_uniform_colour(glm::vec3(0.3,0.3,0.0));
        }
        //This should be ok; a mutex is locked when entering new poses
        std::thread (BaseUI::gui_main_static, argc_, argv_).detach();
    }

    ~AssemblyAgentNode()
    {
        delete ui;
    }

    double probability_from_densities(std::vector<double> &density, int i)
    {
        if (num_tasks_ == 3)
        {
            double x = density[i];
            double y = density[(i+1)%3];
            double z = density[(i+2)%3];
            return 1.0/3.0*x*y*z + 1.0/3.0*(1.0-x)*(1.0-y)*(1.0-z) + 1.0/2.0*x*y*(1.0-z) + 1.0/2.0*x*(1.0-y)*z + x*(1.0-y)*(1.0-z);
        }
        else
        {
            std::cerr << "Number of tasks not supported yet :(\n";
            exit(1);
        }
        return 0.0;
    }

    void assembly_agent_callback(const dbot_ros_msgs::ObjectsState& state)
    {
        //num_msgs_received_++;
        //std::cout << "Received poses: " << num_msgs_received_ << "\n";
	    std::vector<geometry_msgs::Pose> poses;
	    for (int i = 0; i < state.objects_state.size(); i++) {        
	        poses.push_back (state.objects_state[i].pose.pose);
        }
	    current_object_poses_ = poses;
        double total_prob = 0;
        std::vector<double> density;
        for (int i = 0; i < num_tasks_; i++)
        {
            std::cout << "Task " << i+1 << " ";
            density.push_back(tasks_[i].evaluate_task(current_object_poses_, connection_list_));
            //total_prob += probabilities[i];
        }
        for (int i = 0; i < num_tasks_; i++)
        {
           // if (total_prob - 0.0001 < 0)
           //     std::cout << "Task " << i+1 << " probability: " << 1.0/double(num_tasks_) << "\n";
           // else
            std::cout << "Task " << i+1 << " probability: " << probability_from_densities(density, i) << "\n";
        }
// 1/3*x*y*z + 1/3*(1-x)*(1-y)*(1-z) + 1/2*x*y*(1-z) + 1/2*x*(1-y)*z + x*(1-y)*(1-z)
        /*
	    int current_subtask = task_.evaluate_task(current_object_poses_, connection_list_);
        if (current_subtask == task_.num_subtasks_)
        {
            std::cout << "You have completed the assembly task!!" << '\n';
        }
        else
        {
            std::cout << task_.subtask_description(current_subtask) << " (subtask: " << current_subtask+1 << ")\n";
            object_assembly_msgs::ConnectionInfoList connections_msg;
            for (int i = 0; i < current_subtask + 1; i++)
                connections_msg.connections.push_back(
                                        connection_list_[i]);
            connection_list_publisher_.publish(connections_msg);
        }
        */
        std::vector<tf::Transform> new_poses;
        for (int i = 0; i < current_object_poses_.size(); i++)
        {
            tf::Transform single_pose(tf::Quaternion (
                                        current_object_poses_[i].orientation.x,
                                        current_object_poses_[i].orientation.y,
                                        current_object_poses_[i].orientation.z,
                                        current_object_poses_[i].orientation.w),
                                      tf::Vector3 (
                                        current_object_poses_[i].position.x,
                                        current_object_poses_[i].position.y,
                                        current_object_poses_[i].position.z));
            new_poses.push_back(single_pose);
        }
        ui->update_poses(new_poses);
    }

private:
    const int num_tasks_;
    std::vector<AssemblyTask> tasks_;
    //int max_particles_;
    ros::NodeHandle node_handle_;
    std::vector<std::string> object_names_;
    std::vector<geometry_msgs::Pose> current_object_poses_;
    geometry_msgs::Pose current_relative_pose_;
    std::vector<object_assembly_msgs::ConnectionInfo> connection_list_;
    ros::Publisher connection_list_publisher_;
    GuessModeUI *ui;
    int argc_;
    char** argv_;
    int num_msgs_received_ = 0;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "assembly_agent");
    ros::NodeHandle nh("~");

    tf::Vector3 up_vector;
/*
    std::string surface_params_service;
    nh.getParam("surface_params_service", surface_params_service);
    ros::ServiceClient client = nh.serviceClient<object_assembly_msgs::FetchSurfaceParams>(surface_params_service);
    object_assembly_msgs::FetchSurfaceParams srv;
    srv.request.recalculate = false;
    while (!client.call(srv))
    {
        ROS_ERROR("Failed to call background removal service. Trying again...");
        usleep(500);
    }
    up_vector.setX(srv.response.surface_parameters.a1);
    up_vector.setY(-1.0);
    up_vector.setZ(srv.response.surface_parameters.a3);
*/
    //TODO: REMOVE NEXT 3 LINES WHEN FINISHED EXPERIMENTING
    up_vector.setX(0.0);
    up_vector.setY(-1.0);
    up_vector.setZ(0.0);
    up_vector.normalize();

    // object data
    std::vector<std::string> object_names;
    std::vector<std::string> object_symmetries;
    std::vector<std::string> task_names;
    std::string object_pose_topic;
    std::vector<std::string> descriptions;

    //task data
    int num_tasks, num_checks;
    std::vector<AssemblyTask> tasks;

    /* ------------------------------ */
    /* -     Read out data          - */
    /* ------------------------------ */
    // get object names
    nh.getParam("objects", object_names);
    nh.getParam("object_symmetries", object_symmetries);
    nh.getParam("object_pose_topic", object_pose_topic);
    nh.getParam("number_of_checks", num_checks);
    std::vector<double> margin;
    nh.getParam("margin", margin);
    int max_particles;
    nh.getParam("max_particles", max_particles);

    std::vector<object_assembly_msgs::ConnectionInfo> connection_list;
    // get task data
    nh.getParam("number_of_tasks", num_tasks);

    for (int i = 1; i <= num_tasks; i++) {
        std::string task_pre = std::string("task") + std::to_string(i) + "/";
        int num_connections;
        nh.getParam(task_pre + "number_of_connections", num_connections);
        std::string task_name;
        nh.getParam(task_pre + "name", task_name);
        std::vector<AssemblySubtask> subtasks;
        std::vector<std::vector<int>> connection_pairs;
        for (int j = 1; j <= num_connections; j++)
        {
            std::string connection_pre = task_pre + "connection" + std::to_string(j) + "/";
	        std::vector<int> parts;
            nh.getParam(connection_pre + "parts", parts);
            parts[0]--; //convert to zero-based
            parts[1]--;
            connection_pairs.push_back(parts);
            std::vector<double> relative_pose;
            nh.getParam(connection_pre + "relative_pose", relative_pose);
            tf::Vector3 relative_position;
            relative_position.setValue(relative_pose[0],
                                       relative_pose[1],
                                       relative_pose[2]);
            tf::Quaternion relative_orientation = 
                    tf::Quaternion(relative_pose[3],
                                   relative_pose[4],
                                   relative_pose[5],
                                   relative_pose[6]);
            //TODO...
            // subtask shorthand prefix
            AssemblySubtask subtask(i-1,
                                    num_checks,
                                    parts[0],
        				            parts[1],
                                    "no_frame",
        				            relative_position,
                                    relative_orientation,
                                    margin,
        				            "place",
                                    object_symmetries,
                                    up_vector);
	        subtasks.push_back(subtask);

            //build connection messages to save time later
            //TODO move to AssemblyAgentNode, because the relative pose
            //in the message should change when objects are symmetrical
            object_assembly_msgs::ConnectionInfo connection;
            connection.part = parts[0] - 1;
            connection.relative_part = parts[1] - 1;
            connection.relative_pose.position.x = relative_pose[0];
            connection.relative_pose.position.y = relative_pose[1];
            connection.relative_pose.position.z = relative_pose[2];
            connection.relative_pose.orientation.x = relative_pose[3];
            connection.relative_pose.orientation.y = relative_pose[4];
            connection.relative_pose.orientation.z = relative_pose[5];
            connection.relative_pose.orientation.w = relative_pose[6];
            connection.num_particles = 0;
            connection_list.push_back(connection);
        }

        AssemblyTask task(object_names.size(), connection_pairs, num_connections, subtasks, max_particles);
        tasks.push_back(task);
	
    }

    std::string connection_info_list_topic;
    nh.getParam("connection_info_list_topic",
                connection_info_list_topic);

    AssemblyAgentNode assembly_agent_node(object_names, tasks,
                connection_info_list_topic, connection_list, argc, argv);

    ros::Subscriber subscriber = nh.subscribe(
        object_pose_topic, 1, &AssemblyAgentNode::assembly_agent_callback, &assembly_agent_node);

    ros::spin();

    return 0;
}
