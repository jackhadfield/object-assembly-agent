#include <assembly_subtask.hpp>
//TODO rotate desired relative pose, not received poses, to save time

AssemblySubtask::AssemblySubtask(
    int id,
    int num_checks,
    std::vector<int> first_object,
    std::vector<int> second_object,
    std::string frame,
    tf::Vector3 relative_position,
    tf::Quaternion relative_orientation,
    std::vector<double> margin,
    std::string connection_type,
    std::vector<std::string> object_symmetries,
    tf::Vector3 up_vector)
    : subtask_id_(id),
      num_checks_(num_checks),
      first_object_(first_object),
      second_object_(second_object),
      frame_(frame),
      relative_position_(relative_position),
      relative_orientation_(relative_orientation),
      margin_(margin),
      connection_type_(connection_type),
      up_vector_(up_vector)
{
    // Change to 0 based
    for (int i = 0; i < first_object_.size(); i++)
    {
        first_object_[i] -= 1;
    }
    for (int i = 0; i < second_object_.size(); i++)
    {
        second_object_[i] -= 1;
    }
    if (first_object_.size() == 1)
        first_object_symmetry_ = object_symmetries[first_object_[0]];
    else {
        //TODO
    }
    if (second_object_.size() == 1)
        second_object_symmetry_ = object_symmetries[second_object_[0]];
    else {
        //TODO
    }
    //align y axis with down vector
    tf::Vector3 down_vector = up_vector_;
    down_vector *= -1;
    rot_axis_ = down_vector.cross(tf::Vector3(0,1,0));
    rot_angle_ = down_vector.angle(tf::Vector3(0,1,0));
    std::cout << "angle: " << rot_angle_ << "\n";
    tf::Quaternion down_vector_to_y(rot_axis_, -rot_angle_);
    tf::Vector3 relative_pos_table_frame = relative_position_.rotate(rot_axis_, -rot_angle_);
    connection_pose.position.x = relative_pos_table_frame.x();
    connection_pose.position.y = relative_pos_table_frame.y();
    connection_pose.position.z = relative_pos_table_frame.z();
    tf::Quaternion relative_rot_table_frame = down_vector_to_y * 
        relative_orientation_;
    connection_pose.orientation.x = relative_rot_table_frame.x();
    connection_pose.orientation.y = relative_rot_table_frame.y();
    connection_pose.orientation.z = relative_rot_table_frame.z();
    connection_pose.orientation.w = relative_rot_table_frame.w();
}

bool AssemblySubtask::evaluate_subtask(
        std::vector<geometry_msgs::Pose> current_object_poses,
        double &score) {
    tf::Transform first_object_pose, second_object_pose;
    std::vector<double> first_object_position, second_object_position;
    tf::Quaternion first_object_orientation(), 
                   second_object_orientation();
    std::string first_object_symmetry, second_object_symmetry;

    if (first_object_.size() == 1) {
        first_object_position.push_back (
                current_object_poses[first_object_[0]].position.x);
        first_object_position.push_back (
                current_object_poses[first_object_[0]].position.y);
        first_object_position.push_back (
                current_object_poses[first_object_[0]].position.z);
        tf::Quaternion q1 (
                current_object_poses[first_object_[0]].orientation.x,
                current_object_poses[first_object_[0]].orientation.y,
                current_object_poses[first_object_[0]].orientation.z,
                current_object_poses[first_object_[0]].orientation.w);
        first_object_pose.setRotation (q1);
        tf::Vector3 v1 (
                current_object_poses[first_object_[0]].position.x,
                current_object_poses[first_object_[0]].position.y,
                current_object_poses[first_object_[0]].position.z);
        first_object_pose.setOrigin (v1);
    }
    else {
        //tf::Quaternion temp1();   
        //first_object_orientation = temp1;
	//TODO
    }

    if (second_object_.size() == 1) {
        second_object_position.push_back (
                current_object_poses[second_object_[0]].position.x);
        second_object_position.push_back (
                current_object_poses[second_object_[0]].position.y);
        second_object_position.push_back (
                current_object_poses[second_object_[0]].position.z);
        tf::Quaternion q2 (
                current_object_poses[second_object_[0]].orientation.x,
                current_object_poses[second_object_[0]].orientation.y,
                current_object_poses[second_object_[0]].orientation.z,
                current_object_poses[second_object_[0]].orientation.w);
        second_object_pose.setRotation (q2);
        tf::Vector3 v2 (
                current_object_poses[second_object_[0]].position.x,
                current_object_poses[second_object_[0]].position.y,
                current_object_poses[second_object_[0]].position.z);
        second_object_pose.setOrigin (v2);
        
    }
    else {
        //tf::Quaternion temp2();
        //second_object_orientation = temp2;
        //TODO
    }

    if (first_object_symmetry_ == "cubic" &&
            second_object_symmetry_ == "cubic") {
        if (evaluate_cubic_cubic(first_object_pose,
                                 second_object_pose,
                                 score)) {
            checks_++;
            if (checks_ >= num_checks_)
                subtask_complete_ = true;
        }
        else
        {
            checks_ = 0;
        }
    }
    return subtask_complete_;
}

bool AssemblySubtask::evaluate_no_symmetry(
                                  tf::Transform first_object_pose,
                                  tf::Transform second_object_pose,
                                  double &score)
{
//TODO build tf here and accept origins and rotations as parameters
    bool position_check, orientation_check;
    double position_score, orientation_score;
    tf::Transform t21(first_object_pose);
    t21 = t21.inverseTimes(second_object_pose);
    if (frame_ == "relative")
    {
        position_check = compare_tf_vectors(relative_position_,
                                            t21.getOrigin(),
                                            margin_[0],
                                            margin_[1],
                                            margin_[2],
                                            position_score);
    }
    else
    {
        tf::Vector3 first_pos = first_object_pose.getOrigin();
        first_pos.rotate(rot_axis_, rot_angle_);
        tf::Vector3 second_pos = second_object_pose.getOrigin();
        second_pos.rotate(rot_axis_, rot_angle_);
        position_check = compare_tf_vectors(relative_position_,
                                            first_pos - second_pos,
                                            margin_[0],
                                            margin_[1],
                                            margin_[2],
                                            position_score);
    }
    orientation_check = compare_tf_quaternions(relative_orientation_,
                                               t21.getRotation(),
                                               margin_[3],
                                               orientation_score);
    score = position_score * orientation_score;
    return position_check * orientation_check;
}

void AssemblySubtask::sort_vector3(tf::Vector3 &v)
{
    double temp, x, y, z;
    x = v.getX();
    y = v.getY();
    z = v.getZ();
    if (x > z)
    {
        temp = x;
        x = z;
        z = temp;
    }
    if (x > y)
    {
        temp = x;
        x = y;
        y = temp;
    }
    if (y > z)
    {
        temp = y;
        y = z;
        z = temp;
    }
    v.setX(x);
    v.setY(y);
    v.setZ(z);
}

bool AssemblySubtask::evaluate_cubic_cubic(
                                  tf::Transform first_object_pose,
                                  tf::Transform second_object_pose,
                                  double &score)
{
    bool position_check, orientation_check;
    double position_score, orientation_score;
    //for (int i = 0; i < 24; i++) {
    //    for (int j = 0; j < 24; j++) {
            //tf::Quaternion q1(,,,i%4);
            //[1,0,0],[-1,0,0],[0,1,0],...

    tf::Transform t21(first_object_pose);
    t21 = t21.inverseTimes(second_object_pose);
    if (frame_ == "relative")
    {
        tf::Vector3 position_diff = (first_object_pose.getOrigin()-second_object_pose.getOrigin()).absolute();
        sort_vector3(position_diff);
        position_check = compare_tf_vectors(relative_position_,
                                            position_diff,
                                            margin_[0],
                                            margin_[1],
                                            margin_[2],
                                            position_score);
    }
    else
    {
        tf::Vector3 first_pos = first_object_pose.getOrigin();
        first_pos.rotate(rot_axis_, rot_angle_);
        tf::Vector3 second_pos = second_object_pose.getOrigin();
        second_pos.rotate(rot_axis_, rot_angle_);
        position_check = compare_tf_vectors(relative_position_,
                                            first_pos - second_pos,
                                            margin_[0],
                                            margin_[1],
                                            margin_[2],
                                            position_score);
    }
    auto abs_dot_product = std::abs(relative_orientation_.dot(t21.getRotation()));
    orientation_check = (abs_dot_product > 1 - margin_[3] ||
        abs_dot_product < margin_[3] ||
        (abs_dot_product > 0.5 - margin_[3] && 
        abs_dot_product < 0.5 + margin_[3]) || 
        (abs_dot_product > 0.70711 - margin_[3] && 
        abs_dot_product < 0.70711 + margin_[3]));
    orientation_score = 1; //TODO fix
    score = position_score * orientation_score;
    return position_check && orientation_check;
}

bool AssemblySubtask::compare_tf_vectors(tf::Vector3 v1,
                                           tf::Vector3 v2,
                                           double x_margin,
                                           double y_margin,
                                           double z_margin,
                                           double &score)
{
    //std::cout << "v1: " << v1.getX() << " " << v1.getY() << " " << v1.getZ() << "\n";
    //std::cout << "v2: " << v2.getX() << " " << v2.getY() << " " << v2.getZ() << "\n";
    //const int N=10;
    //double x_check = std::max(std::min(1.0, std::abs(v1.getX() - v2.getX())/(x_margin*(1-N)) + N/(N-1)), 0.0);
    //double y_check = std::max(std::min(1.0, std::abs(v1.getY() - v2.getY())/(y_margin*(1-N)) + N/(N-1)), 0.0);
    //double z_check = std::max(std::min(1.0, std::abs(v1.getZ() - v2.getZ())/(z_margin*(1-N)) + N/(N-1)), 0.0);
    //score = x_check * y_check * z_check;

    const int N = 8;
    const double ymax = 100;
    const double sqrt3 = 1.732050808;
    //score = std::min(1.0, sqrt3/std::sqrt( pow((v1.getX() - v2.getX())/(x_margin), 2) + pow((v1.getY() - v2.getY())/(y_margin), 2) + pow((v1.getZ() - v2.getZ())/(z_margin), 2) ) );
    score = std::min(1.0, sqrt3/std::sqrt( ((ymax-1)/(N*N-1)) * (pow((v1.getX() - v2.getX())/(x_margin), 2) + pow((v1.getY() - v2.getY())/(y_margin), 2) + pow((v1.getZ() - v2.getZ())/(z_margin), 2) ) + 3*(N*N-ymax)/(N*N-1) ) );
    //score = std::min(1.0, sqrt3/std::sqrt( (ymax/(N*N))*(pow((v1.getX() - v2.getX())/(x_margin), 2) + pow((v1.getY() - v2.getY())/(y_margin), 2) + pow((v1.getZ() - v2.getZ())/(z_margin), 2)) ) );
    //std::cout << "score: " << score << "\n";

    return (std::abs(v1.getX() - v2.getX()) < x_margin &&
            std::abs(v1.getY() - v2.getY()) < y_margin &&
            std::abs(v1.getZ() - v2.getZ()) < z_margin);
}

bool AssemblySubtask::compare_tf_quaternions(tf::Quaternion q1,
                                             tf::Quaternion q2,
                                             double margin,
                                             double &score)
{
    score = 1; //TODO
    return double(std::abs(q1.dot(q2)) > 1 - margin);
}
