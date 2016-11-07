#include <assembly_subtask.hpp>

class AssemblyTask
{

public:
    AssemblyTask(int num_subtasks,
                 std::vector<AssemblySubtask> subtasks,
                 int max_particles,
                 std::vector<std::string> descriptions);
    
    //returns current subtask
    int evaluate_task(std::vector<geometry_msgs::Pose> current_object_poses, std::vector<object_assembly_msgs::ConnectionInfo> &connection_list);

    std::string subtask_description(int subtask);

    int num_subtasks_;

private:
    int current_subtask_ = 0;
    std::vector<AssemblySubtask> subtasks_;
    int max_particles_;
    std::vector<std::string> descriptions_;

};
