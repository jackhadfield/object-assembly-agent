#include <assembly_task_learning_mode.hpp>
#include <algorithm>
#include <pngwriter.h>

AssemblyTask::AssemblyTask(
    int num_parts,
    std::vector<std::vector<int>> connection_pairs,
    int num_connections,
    std::vector<AssemblySubtask> subtasks,
    int max_particles,
    std::vector<std::string> words,
    std::string english,
    std::string image_output_dir
    )
    : num_parts_(num_parts),
      connection_pairs_(connection_pairs),
      num_connections_(num_connections),
      subtasks_(subtasks),
      max_particles_(max_particles),
      image_output_dir_(image_output_dir),
      words_shuffled_(words),
      english_(english)
{
    create_word_images();

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
}

bool AssemblyTask::evaluate_task(std::vector<geometry_msgs::Pose> current_object_poses, std::vector<object_assembly_msgs::ConnectionInfo> &connection_list)
{
/*
    if (connections_complete_==num_connections_)
    {
        //Assembly finished! Return true
        return true;
    }
*/

    for (int i = 0; i < subtasks_.size(); i++)
    {
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
                    AssemblySubgraph new_subgraph;
                    new_subgraph.nodes.push_back(connection_pairs_[i][0]);
                    new_subgraph.nodes.push_back(connection_pairs_[i][1]);
                    //new_subgraph.rotations.push_back(subtasks_[i].first_rotation);
                    //new_subgraph.rotations.push_back(subtasks_[i].first_rotation * subtasks_[i].second_rotation);
                    rotations_[connection_pairs_[i][0]] = subtasks_[i].first_rotation;
                    rotations_[connection_pairs_[i][1]] = subtasks_[i].first_rotation * subtasks_[i].second_rotation;
                    rotations_[connection_pairs_[i][1]].normalize();
                    //std::cout << "1st angle: " << subtasks_[i].first_rotation.getAngle() << "\n";
                    //std::cout << "2nd angle: " << subtasks_[i].second_rotation.getAngle() << "\n";
                    //std::cout << connection_pairs_[i][0] << " angle: " << rotations_[connection_pairs_[i][0]].getAngle() << "\n";
                    //std::cout << connection_pairs_[i][1] << " angle: " << rotations_[connection_pairs_[i][1]].getAngle() << "\n";
                    //std::cout << "1st: " << subtasks_[i].first_rotation.getAxis().getX() << " " << subtasks_[i].first_rotation.getAxis().getY() << " " << subtasks_[i].first_rotation.getAxis().getZ() << "\n";
                    //std::cout << "2nd: " << subtasks_[i].second_rotation.getAxis().getX() << " " << subtasks_[i].second_rotation.getAxis().getY() << " " << subtasks_[i].second_rotation.getAxis().getZ() << "\n";
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
    }
    int count_ones = 0;
    //std::cout << "Status Vector: [";
    for (int i = 0; i < num_connections_; i++)
    {
        //std::cout << connection_status_vector_[i] << ", ";
        if (connection_status_vector_[i]) count_ones++;
    }
    //std::cout << "\b\b  \b\b]\n";

    return (count_ones == num_connections_);
/*
    while (subtasks_[current_subtask_].evaluate_subtask(current_object_poses, score))
    {
        connection_list[current_subtask_].relative_pose = subtasks_[current_subtask_].connection_pose;
        connection_list[current_subtask_].num_particles = 25;// max_particles_;
        current_subtask_++;
        if (current_subtask_==num_subtasks_)
        {
            //Assembly finished! Return number of subtasks
            return current_subtask_;
        }
    }
    connection_list[current_subtask_].relative_pose = subtasks_[current_subtask_].connection_pose;
    connection_list[current_subtask_].num_particles = int(score * max_particles_);
    //std::cout << "num_particles: " << connection_list[current_subtask_].num_particles << " from score: " << score << "\n";
    return current_subtask_;
*/
}

void AssemblyTask::create_word_images()
{
    for (int i = 0; i < words_shuffled_.size(); i++)
    {
        pngwriter png_temp(words_shuffled_[i].length() * 14, 21, 1.0, (image_output_dir_ + words_shuffled_[i] + ".png").c_str()); //width then height. TODO: maybe x2 for utf8?
        char font[] = "/home/jack/catkin_ws/src/object_assembly_ros/resource/FreeSansBold.ttf";
        char * text = new char [words_shuffled_[i].length() + 1];
        std::strcpy(text, words_shuffled_[i].c_str());
        int width = png_temp.get_text_width_utf8(font, 14, text);
        if (width >= 28)
        {
            pngwriter png(width, width, 1.0, (image_output_dir_ + words_shuffled_[i] + ".png").c_str());
            png.plot_text_utf8(font, 14,
                    0, (int)(width/2)-6, 0.0,
				    text,
				    0.0, 0.0, 0.0);
            png.scale_k(128.0/width);
            png.square(1, 1,
			    128, png.getheight(),
			    0.0, 0.0, 0.0);
            png.close();
        }
        else
        {
            pngwriter png(128, 128, 1.0, (image_output_dir_ + words_shuffled_[i] + ".png").c_str());
            png.plot_text_utf8(font, 28,
                    64-(int)(width/2), 64-14, 0.0,
				    text,
				    0.0, 0.0, 0.0);
            png.square(1, 1,
			    128, png.getheight(),
			    0.0, 0.0, 0.0);
            png.close();
        }
        delete[] text;
    }
}

void AssemblyTask::display_english_sentence()
{
    std::cout << english_ << "\n";
}
