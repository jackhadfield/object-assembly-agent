//#include <Eigen/Dense>

#include <fstream>
#include <ctime>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>

#include <assembly_task.hpp>

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


class AssemblyAgentNode
{

public:

    AssemblyAgentNode(AssemblyTask task, std::string connection_info_list_topic, std::vector<object_assembly_msgs::ConnectionInfo> connection_list)
        : task_(task), connection_list_(connection_list), node_handle_("~")
    {
        std::cout << "Creating Assembly Agent Node\n";



        connection_list_publisher_ = node_handle_.advertise<object_assembly_msgs::ConnectionInfoList>(connection_info_list_topic, 0);
        //object_state_publisher_ =
        //    node_handle_.advertise<XXXXXXXX>("XXXXXXXX", 0);
    }

    void assembly_agent_callback(const dbot_ros_msgs::ObjectsState& state)
    {
	    std::vector<geometry_msgs::Pose> poses;
	    for (int i = 0; i < state.objects_state.size(); i++) {        
	        poses.push_back (state.objects_state[i].pose.pose);
        }
	    current_object_poses_ = poses;
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

    }

private:
    AssemblyTask task_;
    //int max_particles_;
    ros::NodeHandle node_handle_;
    std::vector<std::string> object_names_;
    std::vector<geometry_msgs::Pose> current_object_poses_;
    geometry_msgs::Pose current_relative_pose_;
    std::vector<object_assembly_msgs::ConnectionInfo> connection_list_;
    ros::Publisher connection_list_publisher_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "assembly_agent");
    ros::NodeHandle nh("~");

    tf::Vector3 up_vector;

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
    up_vector.normalize();

    // object data
    std::vector<std::string> object_names;
    std::vector<std::string> object_symmetries;
    std::string object_pose_topic;
    std::vector<std::string> descriptions;

    //task data
    int num_subtasks, num_checks;
    std::vector<AssemblySubtask> subtasks;

    /* ------------------------------ */
    /* -     Read out data          - */
    /* ------------------------------ */
    // get object names
    nh.getParam("objects", object_names);
    nh.getParam("object_symmetries", object_symmetries);
    nh.getParam("object_pose_topic", object_pose_topic);
    nh.getParam("number_of_checks", num_checks);

    std::vector<object_assembly_msgs::ConnectionInfo> connection_list;
    // get task data
    nh.getParam("number_of_subtasks", num_subtasks);

    for (int i=1; i<=num_subtasks; i++) {
        std::string connection_type;
	    std::vector<int> first_object;
    	std::vector<int> second_object;

	// subtask shorthand prefix
        std::string subtask_pre = std::string("subtasks/") + "subtask" + std::to_string(i) + "/";

        std::string description;
        nh.getParam(subtask_pre + "description", description);
        descriptions.push_back(description);

	    nh.getParam(subtask_pre + "object", first_object);
	    nh.getParam(subtask_pre + "relative_object", second_object);
	    nh.getParam(subtask_pre + "connection_type", connection_type);

	    std::vector<double> relative_pose;
	    nh.getParam(subtask_pre + "relative_pose", relative_pose);
        tf::Vector3 relative_position;
        relative_position.setValue(relative_pose[0],
                                   relative_pose[1],
                                   relative_pose[2]);
        tf::Quaternion relative_orientation = 
                tf::Quaternion(relative_pose[3],
                               relative_pose[4],
                               relative_pose[5],
                               relative_pose[6]);
        std::vector<double> margin;
	    nh.getParam(subtask_pre + "margin", margin);
        std::string frame;
        nh.getParam(subtask_pre + "frame", frame);
	
        AssemblySubtask subtask(i-1,
                                num_checks,
                                first_object,
    				            second_object,
                                frame,
    				            relative_position,
                                relative_orientation,
                                margin,
    				            connection_type,
                                object_symmetries,
                                up_vector);
	    subtasks.push_back(subtask);

        //build connection messages to save time later
        //TODO move to AssemblyAgentNode, because the relative pose
        //in the message should change when objects are symmetrical
        object_assembly_msgs::ConnectionInfo connection;
        connection.part = first_object[0] - 1;
        connection.relative_part = second_object[0] - 1;
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

    int max_particles;
    nh.getParam("max_particles", max_particles);

    AssemblyTask task(num_subtasks, subtasks, max_particles, descriptions);

    std::string connection_info_list_topic;
    nh.getParam("connection_info_list_topic",
                connection_info_list_topic);

    AssemblyAgentNode assembly_agent_node(task,
                connection_info_list_topic, connection_list);

    ros::Subscriber subscriber = nh.subscribe(
        object_pose_topic, 1, &AssemblyAgentNode::assembly_agent_callback, &assembly_agent_node);

    ros::spin();

    return 0;
}
