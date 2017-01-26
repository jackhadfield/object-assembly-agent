#include <assembly_task_guess_mode.hpp>

AssemblyTask::AssemblyTask(
    int num_parts,
    std::vector<std::vector<int>> connection_pairs,
    int num_connections,
    std::vector<AssemblySubtask> subtasks,
    int max_particles,
    int connection_list_offset,
    std::string connection_vector_topic
    )
    : node_handle_("~"),
      num_parts_(num_parts),
      connection_pairs_(connection_pairs),
      num_connections_(num_connections),
      subtasks_(subtasks),
      max_particles_(max_particles),
      connection_list_offset_(connection_list_offset)
{
    tf::Quaternion unit_quaternion(0.0, 0.0, 0.0, 1.0);
    for (int i = 0; i < num_parts_; i++)
    {
        rotations_.push_back(unit_quaternion);
        part_subgraph_index_.push_back(-1);
    }
    for (int i = 0; i < num_connections_; i++)
    {
        connection_status_vector_.push_back(false);
    }
    scores_ = std::vector<double>(num_connections,0);
    connection_vector_publisher_ = node_handle_.advertise<object_assembly_msgs::ConnectionVector>(connection_vector_topic, 0);
}

double AssemblyTask::evaluate_task(std::vector<tf::Transform> current_object_poses, std::vector<object_assembly_msgs::ConnectionInfo> &connection_list)
{
    //for (int i = 0; i < num_parts_; i++)
    //    std::cout <<"Rotation " << i << ": " << rotations_[i].x() << " "<< rotations_[i].y() << " "<< rotations_[i].z() << " "<< rotations_[i].w() << "\n";

    for (int i = 0; i < subtasks_.size(); i++)
    {
//std::cout << "checking subtask " << i << "\n";
        if (!connection_status_vector_[i])
        {
            if (part_subgraph_index_[connection_pairs_[i][0]] == -1 &&
                part_subgraph_index_[connection_pairs_[i][1]] == -1)
            {
                //neither part is connected to a third part
                connection_status_vector_[i] = subtasks_[i].evaluate_subtask(current_object_poses, scores_[i]);
                //std::cout << "(-1,-1): Just evaled subtask\n";
                if (connection_status_vector_[i])
                {
                    //AssemblySubgraph new_subgraph;
                    //new_subgraph.nodes.push_back(connection_pairs_[i][0]);
                    //new_subgraph.nodes.push_back(connection_pairs_[i][1]);
                    //new_subgraph.rotations.push_back(subtasks_[i].first_rotation);
                    //new_subgraph.rotations.push_back(subtasks_[i].first_rotation * subtasks_[i].second_rotation);
                    rotations_[connection_pairs_[i][0]] = subtasks_[i].first_rotation;
                    rotations_[connection_pairs_[i][1]] = subtasks_[i].first_rotation * subtasks_[i].second_rotation;
                    rotations_[connection_pairs_[i][1]].normalize();
                    //std::cout << "1st angle: " << subtasks_[i].first_rotation.getAngle() << "\n";
                    //std::cout << "2nd angle: " << subtasks_[i].second_rotation.getAngle() << "\n";
                    //std::cout << connection_pairs_[i][0] << " angle: " << rotations_[connection_pairs_[i][0]].getAngle() << "\n";
                    //std::cout << connection_pairs_[i][1] << " angle: " << rotations_[connection_pairs_[i][1]].getAngle() << "\n";
                    //std::cout << "1st: " << subtasks_[i].first_rotation.x() << " " << subtasks_[i].first_rotation.y() << " " << subtasks_[i].first_rotation.z() << " " << subtasks_[i].first_rotation.w() << "\n";
                    //std::cout << "2nd: " << subtasks_[i].second_rotation.x() << " " << subtasks_[i].second_rotation.y() << " " << subtasks_[i].second_rotation.z() << " " << subtasks_[i].second_rotation.w() << "\n";
                    //std::cin >> scores_[i];
                    //subgraph_list_.push_back(new_subgraph);
                    part_subgraph_index_[connection_pairs_[i][0]] = ++subgraph_max_index_;
                    part_subgraph_index_[connection_pairs_[i][1]] = subgraph_max_index_;
                }
            }
            else if (part_subgraph_index_[connection_pairs_[i][1]] == -1)
            {
                //first part is already connected to a third part
                int subgraph_ind = part_subgraph_index_[connection_pairs_[i][0]];
                subtasks_[i].set_first_object_symmetry("none");
                tf::Quaternion prerotation = rotations_[connection_pairs_[i][0]].inverse();
                subtasks_[i].set_prerotation(prerotation);
                //std::cout << " angle = " << prerotation.getAngle() << "\n";
                //std::cout << "axis = " << prerotation.getAxis().getX() << " " << prerotation.getAxis().getY() << " " << prerotation.getAxis().getZ() << "\n";
                connection_status_vector_[i] = subtasks_[i].evaluate_subtask(current_object_poses, scores_[i]);
                if (connection_status_vector_[i])
                {
                    rotations_[connection_pairs_[i][1]] = rotations_[connection_pairs_[i][0]] * subtasks_[i].second_rotation;
                    //std::cout << "2nd angle: " << subtasks_[i].second_rotation.getAngle() << "\n";
                    //std::cout << connection_pairs_[i][0] << " angle: " << rotations_[connection_pairs_[i][0]].getAngle() << "\n";
                    //std::cout << connection_pairs_[i][1] << " angle: " << rotations_[connection_pairs_[i][1]].getAngle() << "\n";
                    //std::cout << "2nd: " << subtasks_[i].second_rotation.getAxis().getX() << " " << subtasks_[i].second_rotation.getAxis().getY() << " " << subtasks_[i].second_rotation.getAxis().getZ() << "\n";
                    //std::cout << "prerot: " << rotations_[connection_pairs_[i][0]].getAxis().getX() << " " << rotations_[connection_pairs_[i][0]].getAxis().getY() << " " << rotations_[connection_pairs_[i][0]].getAxis().getZ() << "\n";
                    //std::cin >> scores_[i];
                    //subgraph_list_[subgraph_ind].nodes.push_back(connection_pairs_[i][1]);
                    part_subgraph_index_[connection_pairs_[i][1]] = subgraph_ind;
                }
            }
            else if(part_subgraph_index_[connection_pairs_[i][0]] == -1)
            {
                //second part is already connected to a third part
                int subgraph_ind = part_subgraph_index_[connection_pairs_[i][1]];
                subtasks_[i].set_second_object_symmetry("none");
                tf::Quaternion prerotation = rotations_[connection_pairs_[i][1]].inverse();
                subtasks_[i].set_prerotation(prerotation);
                connection_status_vector_[i] = subtasks_[i].evaluate_subtask(current_object_poses, scores_[i]);
                if (connection_status_vector_[i])
                {
                    rotations_[connection_pairs_[i][0]] = rotations_[connection_pairs_[i][1]] * subtasks_[i].first_rotation;
                    //subgraph_list_[subgraph_ind].nodes.push_back(connection_pairs_[i][0]);
                    part_subgraph_index_[connection_pairs_[i][0]] = subgraph_ind;
                }
            }
            else
            {
                //both parts are connected to other components
                //Some clutter will be created, but the possible subgraphs are limited in number, so the effect is negligible
                subtasks_[i].set_first_object_symmetry("none");
                subtasks_[i].set_second_object_symmetry("none");
                tf::Quaternion prerotation1 = rotations_[connection_pairs_[i][0]].inverse();
                tf::Quaternion prerotation2 = rotations_[connection_pairs_[i][1]].inverse();
                subtasks_[i].set_prerotations(prerotation1, prerotation2);
                connection_status_vector_[i] = subtasks_[i].evaluate_subtask(current_object_poses, scores_[i]);
                if (connection_status_vector_[i])
                {
                    int subgraph_ind0 = part_subgraph_index_[connection_pairs_[i][0]];
                    int subgraph_ind1 = part_subgraph_index_[connection_pairs_[i][1]];
                    for (int j = 0; j < num_parts_; j++)
                    {
                        //subgraph_list_[subgraph_ind0].nodes.push_back(subgraph_list_[subgraph_ind1].nodes[j]);
                        if (part_subgraph_index_[j] == subgraph_ind1)
                            part_subgraph_index_[j] = subgraph_ind0;
                        //part_subgraph_index_[subgraph_list_[subgraph_ind1].nodes[j]] = subgraph_ind0;
                    }
                }
            }
        }
        if (connection_status_vector_[i])
        {
            connection_list[connection_list_offset_ + i].relative_pose = subtasks_[i].connection_pose;
            connection_list[connection_list_offset_ + i].num_particles = max_particles_;
        }
        else if (scores_[i] >= 1.0/max_particles_)
        {
            connection_list[connection_list_offset_ + i].relative_pose = subtasks_[i].connection_pose;
            connection_list[connection_list_offset_ + i].num_particles = int(scores_[i] * max_particles_);
        }
    }
    int count_ones = 0;
    std::cout << "Status Vector: [";
    for (int i = 0; i < num_connections_; i++)
    {
        std::cout << connection_status_vector_[i] << ", ";
        if (connection_status_vector_[i]) count_ones++;
    }
    std::cout << "\b\b  \b\b]\n";
    publish_connection_vector();

    return double(count_ones)/double(num_connections_);
}

void AssemblyTask::publish_connection_vector()
{
    object_assembly_msgs::ConnectionVector connection_vector_msg;
    for (int i = 0; i < connection_status_vector_.size(); i++)
        connection_vector_msg.v.push_back((char)connection_status_vector_[i]);
    connection_vector_publisher_.publish(connection_vector_msg);
}
