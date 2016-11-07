#include <assembly_task.hpp>

AssemblyTask::AssemblyTask(
    int num_subtasks,
    std::vector<AssemblySubtask> subtasks,
    int max_particles,
    std::vector<std::string> descriptions
    )
    : num_subtasks_(num_subtasks),
      subtasks_(subtasks),
      max_particles_(max_particles),
      descriptions_(descriptions)
{
}

int AssemblyTask::evaluate_task(std::vector<geometry_msgs::Pose> current_object_poses, std::vector<object_assembly_msgs::ConnectionInfo> &connection_list)
{
    if (current_subtask_==num_subtasks_)
    {
        //Assembly finished! Return number of subtasks
        return current_subtask_;
    }
    double score;
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
}

std::string AssemblyTask::subtask_description(int subtask)
{
    return descriptions_[subtask];
}
