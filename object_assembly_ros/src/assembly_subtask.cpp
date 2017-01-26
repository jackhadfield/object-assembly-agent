#include <assembly_subtask.hpp>
//TODO rotate desired relative pose, not received poses, to save time
//TODO fix second_rotation (currently only works if desired rotation is zero)

AssemblySubtask::AssemblySubtask(
    int id,
    int num_checks,
    int first_object,
    int second_object,
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
    //relativeXZ_ = tf::Vector3(relative_position_.getX(), relative_position_.getY(), 0);
    relativeY_ = tf::Vector3(0, 0, relative_position_.getY());
    relativeXZ_sorted_ = relative_position_.absolute();
    sort_vector3(relativeXZ_sorted_);

    first_object_symmetry_ = object_symmetries[first_object_];
    second_object_symmetry_ = object_symmetries[second_object_];

    //align y axis with down vector
    tf::Vector3 down_vector = up_vector_;
    down_vector *= -1;
    rot_angle_ = down_vector.angle(tf::Vector3(0,1,0));
    if (std::abs(rot_angle_) < 0.001)
        rot_axis_ = tf::Vector3(1.0, 0.0, 0.0);
    else if (std::abs(rot_angle_ - M_PI) < 0.001)
    {
        double x = down_vector.getX();
        double y = down_vector.getY();
        double z = down_vector.getZ();
        if (z != 0 && -x != y)
        {
            rot_axis_ = tf::Vector3(z, z, -x-y);
        }
        else
        {
            rot_axis_ = tf::Vector3(-y-z, x, x);
        }
    }
    else
        rot_axis_ = down_vector.cross(tf::Vector3(0,1,0));
    rot_axis_.normalize();

    tf::Quaternion down_vector_to_y(rot_axis_, -rot_angle_);
    relative_pos_table_frame_ = relative_position_.rotate(rot_axis_, -rot_angle_);
    connection_pose.position.x = relative_pos_table_frame_.x();
    connection_pose.position.y = relative_pos_table_frame_.y();
    connection_pose.position.z = relative_pos_table_frame_.z();
    relative_rot_table_frame_ = down_vector_to_y * relative_orientation_;
    connection_pose.orientation.x = relative_rot_table_frame_.x();
    connection_pose.orientation.y = relative_rot_table_frame_.y();
    connection_pose.orientation.z = relative_rot_table_frame_.z();
    connection_pose.orientation.w = relative_rot_table_frame_.w();
}

bool AssemblySubtask::evaluate_subtask(
        std::vector<tf::Transform> &current_object_poses,
        double &score)
{
    tf::Transform first_object_pose, second_object_pose;
    first_object_pose = current_object_poses[first_object_];
    second_object_pose = current_object_poses[second_object_];

    if (evaluate_any(first_object_pose, second_object_pose, score))
    {
        checks_++;
        if (checks_ >= num_checks_)
            subtask_complete_ = true;
    }
    else
    {
        checks_ = 0;
    }

    return subtask_complete_;
}

bool AssemblySubtask::evaluate_subtask(
        std::vector<geometry_msgs::Pose> &current_object_poses,
        double &score)
{
    tf::Transform first_object_pose(tf::Quaternion(
                                        current_object_poses[first_object_].orientation.x,
                                        current_object_poses[first_object_].orientation.y,
                                        current_object_poses[first_object_].orientation.z,
                                        current_object_poses[first_object_].orientation.w),
                                    tf::Vector3(
                                        current_object_poses[first_object_].position.x,
                                        current_object_poses[first_object_].position.y,
                                        current_object_poses[first_object_].position.z));

    tf::Transform second_object_pose(tf::Quaternion(
                                        current_object_poses[second_object_].orientation.x,
                                        current_object_poses[second_object_].orientation.y,
                                        current_object_poses[second_object_].orientation.z,
                                        current_object_poses[second_object_].orientation.w),
                                     tf::Vector3(
                                        current_object_poses[second_object_].position.x,
                                        current_object_poses[second_object_].position.y,
                                        current_object_poses[second_object_].position.z));

    if (evaluate_any(first_object_pose, second_object_pose, score))
    {
        checks_++;
        if (checks_ >= num_checks_)
            subtask_complete_ = true;
    }
    else
    {
        checks_ = 0;
    }

    return subtask_complete_;
}

bool AssemblySubtask::evaluate_any(tf::Transform &first_object_pose,
                                   tf::Transform &second_object_pose,
                                   double &score)
{
//std::cout << "Eval any\n";
    if (first_object_symmetry_ == "cubic" &&
            second_object_symmetry_ == "cubic")
    {
        return evaluate_cubic_cubic(first_object_pose,
                                    second_object_pose,
                                    score);
    }
    else if (first_object_symmetry_ == "none" &&
            second_object_symmetry_ == "cubic")
    {
        return evaluate_none_cubic(first_object_pose,
                                   second_object_pose,
                                   score);
    }
    else if (first_object_symmetry_ == "cubic" &&
            second_object_symmetry_ == "none")
    {
        return evaluate_cubic_none(first_object_pose,
                                   second_object_pose,
                                   score);
    }
    else
    {
        return evaluate_no_symmetry(first_object_pose,
                                    second_object_pose,
                                    score);
    }
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

bool AssemblySubtask::evaluate_no_symmetry(
                                  tf::Transform &first_object_pose,
                                  tf::Transform &second_object_pose,
                                  double &score)
{
    bool position_check1, position_check2, orientation_check = true;
    double position_score1, position_score2, orientation_score = 1.0;

    tf::Transform rotated_first_object_pose = first_object_pose * 
                                              tf::Transform(prerotation1_, tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform t21(rotated_first_object_pose);
    t21 = t21.inverseTimes(second_object_pose);
    tf::Vector3 position_diff1 = t21.getOrigin();
    position_check1 = compare_tf_vectors(relative_position_,
                                         position_diff1,
                                         margin_[0],
                                         margin_[1],
                                         margin_[2],
                                         position_score1);

    tf::Transform rotated_second_object_pose = second_object_pose *
                                               tf::Transform(prerotation2_, tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform t12(rotated_second_object_pose);
    t12 = t12.inverseTimes(first_object_pose);
    tf::Vector3 position_diff2 = t12.getOrigin();
    position_check2 = compare_tf_vectors(-relative_position_,
                                         position_diff2,
                                         margin_[0],
                                         margin_[1],
                                         margin_[2],
                                         position_score2);

    score = position_score1 * orientation_score;
    return position_check1 && position_check2 && orientation_check;
}

bool AssemblySubtask::evaluate_cubic_cubic(
                                  tf::Transform &first_object_pose,
                                  tf::Transform &second_object_pose,
                                  double &score)
{
    bool positionXZ_check, positionY_check, orientation_check = true;
    double positionXZ_score, positionY_score, orientation_score = 1;

    tf::Transform t21(first_object_pose);
    t21 = t21.inverseTimes(second_object_pose);
    tf::Vector3 position_diff = t21.getOrigin();
    tf::Vector3 sorted_position_diff = position_diff.absolute();
    sort_vector3(sorted_position_diff);

    positionXZ_check = compare_tf_vectors(relativeXZ_sorted_,
                                        sorted_position_diff,
                                        margin_[0],
                                        margin_[1],
                                        margin_[2],
                                        positionXZ_score);

    tf::Vector3 first_pos = first_object_pose.getOrigin();
    first_pos.rotate(rot_axis_, rot_angle_);
    tf::Vector3 second_pos = second_object_pose.getOrigin();
    second_pos.rotate(rot_axis_, rot_angle_);
    positionY_check = compare_tf_vectors(relativeY_,
                                        tf::Vector3(0, 0, second_pos.getY() - first_pos.getY()),
                                        margin_[0],
                                        margin_[1],
                                        margin_[2],
                                        positionY_score);

    if (positionXZ_score * positionY_score >= 1.0/max_particles_)
    {
        tf::Vector3 down_vector_in_frame;
        down_vector_in_frame = first_object_pose.inverse() * tf::Vector3(0.0, 1.0, 0.0) - first_object_pose.inverse().getOrigin();
        if (std::abs(relative_position_.angle(position_diff) - M_PI) < 0.001)
        {
            double x = relative_position_.getX();
            double y = relative_position_.getY();
            double z = relative_position_.getZ();
            tf::Vector3 perpendicular;
            //Taken from http://math.stackexchange.com/questions/137362/how-to-find-perpendicular-vector-to-another-vector
            if (z != 0 && -x != y)
            {
                perpendicular = tf::Vector3(z, z, -x-y);
            }
            else
            {
                perpendicular = tf::Vector3(-y-z, x, x);
            }
            perpendicular.normalize();
            first_rotation.setRotation(perpendicular, M_PI);
            down_vector_in_frame = down_vector_in_frame.rotate(perpendicular, M_PI);

        }
        else if (std::abs(relative_position_.angle(position_diff)) < 0.001)
        {

        }
        else
        {
            first_rotation.setRotation(position_diff.cross(relative_position_), position_diff.angle(relative_position_));
            down_vector_in_frame = down_vector_in_frame.rotate(position_diff.cross(relative_position_).normalized(), position_diff.angle(relative_position_));

        }
        if (std::abs(down_vector_in_frame.angle(-up_vector_) - M_PI) < 0.001)
        {
            first_rotation = tf::Quaternion(relative_position_, M_PI) * first_rotation;
        }
        else if (!std::abs(down_vector_in_frame.angle(-up_vector_)) < 0.001)
        {
            first_rotation = tf::Quaternion(down_vector_in_frame.cross(-up_vector_), down_vector_in_frame.angle(-up_vector_)) * first_rotation;
        }        
        first_rotation = round_quaternion_90(first_rotation);
        second_rotation = round_quaternion_90(t21.getRotation());
        set_connection_pose();
    }

    score = positionXZ_score * positionY_score * orientation_score;//std::cout << "calculated score: " << score << "\n";
    return positionXZ_check && positionY_check && orientation_check;
}

bool AssemblySubtask::evaluate_cubic_none(
                                  tf::Transform &first_object_pose,
                                  tf::Transform &second_object_pose,
                                  double &score)
{
    bool position_check, positionY_check = 1, orientation_check;
    double position_score, positionY_score = 1.0, orientation_score;

    tf::Transform rotated_second_object_pose = second_object_pose *
                                               tf::Transform(prerotation_, tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform t21(rotated_second_object_pose);
    t21 = t21.inverseTimes(first_object_pose);

    tf::Vector3 position_diff = t21.getOrigin();

    position_check = compare_tf_vectors(-relative_position_,
                                          position_diff,
                                          margin_[0],
                                          margin_[1],
                                          margin_[2],
                                          position_score);

    if (position_score >= 1.0/max_particles_)
    {
        first_rotation = second_object_pose.inverseTimes(first_object_pose).getRotation();
        set_connection_pose();
    }

    orientation_score = 1;
    orientation_check = true;
    score = position_score * positionY_score * orientation_score;
    return position_check && positionY_check && orientation_check;
}

bool AssemblySubtask::evaluate_none_cubic(
                                  tf::Transform &first_object_pose,
                                  tf::Transform &second_object_pose,
                                  double &score)
{
    bool position_check, positionZ_check = true, orientation_check;
    double position_score, positionZ_score = 1.0, orientation_score;

    tf::Transform rotated_first_object_pose = first_object_pose * 
                                              tf::Transform(prerotation_, tf::Vector3(0.0, 0.0, 0.0));

    tf::Transform t21(rotated_first_object_pose);
    t21 = t21.inverseTimes(second_object_pose);

    tf::Vector3 position_diff = t21.getOrigin();

    position_check = compare_tf_vectors(relative_position_,
                                        position_diff,
                                        margin_[0],
                                        margin_[1],
                                        margin_[2],
                                        position_score);
    if (position_score >= 1.0/max_particles_)
    {
        second_rotation = first_object_pose.inverseTimes(second_object_pose).getRotation();
        set_connection_pose();
    }

    orientation_score = 1;
    orientation_check = true;
    score = position_score * positionZ_score * orientation_score;
    return position_check && positionZ_check && orientation_check;
}

bool AssemblySubtask::compare_tf_vectors(tf::Vector3 v1,
                                         tf::Vector3 v2,
                                         double x_margin,
                                         double y_margin,
                                         double z_margin,
                                         double &score)
{
    const static int N = 8;
    const static double ymax = 100;
    const static double sqrt3 = 1.732050808;
    score = std::min(1.0, sqrt3/std::sqrt( ((ymax-1)/(N*N-1)) * (pow((v1.getX() - v2.getX())/(x_margin), 2) + pow((v1.getY() - v2.getY())/(y_margin), 2) + pow((v1.getZ() - v2.getZ())/(z_margin), 2) ) + 3*(N*N-ymax)/(N*N-1) ) );
//std::cout << "v1: " << v1.getX() << " " << v1.getY() << " " << v1.getZ() << "\n";
//std::cout << "v2: " << v2.getX() << " " << v2.getY() << " " << v2.getZ() << "\n";
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

void AssemblySubtask::set_first_object_symmetry(std::string new_type)
{
    first_object_symmetry_ = new_type;
}

void AssemblySubtask::set_second_object_symmetry(std::string new_type)
{
    second_object_symmetry_ = new_type;
}

void AssemblySubtask::set_prerotation(tf::Quaternion rotation)
{
    //prerotation_ = tf::Quaternion::getIdentity();
    prerotation_ = rotation;
}

void AssemblySubtask::set_prerotations(tf::Quaternion rotation1, tf::Quaternion rotation2)
{
    prerotation1_ = rotation1;
    prerotation2_ = rotation2;
}

tf::Quaternion AssemblySubtask::round_quaternion_90(tf::Quaternion rotation)
{
    double x, y, z, w;
//std::cout << "was: " << rotation.x() << " " << rotation.y() << " " <<rotation.z() << " " <<rotation.w() << "\n";
    if (std::abs(rotation.x()) < 0.25)
        x = 0.0;
    else if (rotation.x() > 0.0)
    {
        if (rotation.x() < 0.6)
            x = 0.5;
        else if (rotation.x() < 0.85)
            x = 0.7071068;
        else
            x = 1.0;
    }
    else
    {
        if (rotation.x() > -0.6)
            x = -0.5;
        else if (rotation.x() > -0.85)
            x = -0.7071068;
        else
            x = -1.0;
    }

    if (std::abs(rotation.y()) < 0.25)
        y = 0.0;
    else if (rotation.y() > 0.0)
    {
        if (rotation.y() < 0.6)
            y = 0.5;
        else if (rotation.y() < 0.85)
            y = 0.7071068;
        else
            y = 1.0;
    }
    else
    {
        if (rotation.y() > -0.6)
            y = -0.5;
        else if (rotation.y() > -0.85)
            y = -0.7071068;
        else
            y = -1.0;
    }

    if (std::abs(rotation.z()) < 0.25)
        z = 0.0;
    else if (rotation.z() > 0.0)
    {
        if (rotation.z() < 0.6)
            z = 0.5;
        else if (rotation.z() < 0.85)
            z = 0.7071068;
        else
            z = 1.0;
    }
    else
    {
        if (rotation.z() > -0.6)
            z = -0.5;
        else if (rotation.z() > -0.85)
            z = -0.7071068;
        else
            z = -1.0;
    }

    if (std::abs(rotation.w()) < 0.25)
        w = 0.0;
    else if (rotation.w() > 0.0)
    {
        if (rotation.w() < 0.6)
            w = 0.5;
        else if (rotation.w() < 0.85)
            w = 0.7071068;
        else
            w = 1.0;
    }
    else
    {
        if (rotation.w() > -0.6)
            w = -0.5;
        else if (rotation.w() > -0.85)
            w = -0.7071068;
        else
            w = -1.0;
    }

    tf::Quaternion rounded_rotation(x, y, z, w);
    rounded_rotation.normalize();
//std::cout << "became: " << rounded_rotation.x() << " " << rounded_rotation.y() << " " <<rounded_rotation.z() << " " <<rounded_rotation.w() << "\n";
    return rounded_rotation;
}

void AssemblySubtask::set_connection_pose()
{
    tf::Vector3 rotated_relative_pos;
    //tf::Quaternion rotated_relative_rot;

    if (first_object_symmetry_ == "cubic" &&
            second_object_symmetry_ == "cubic")
    {
        //std::cout << "relative_pos_table_frame_: " << relative_pos_table_frame_.x() << " " << relative_pos_table_frame_.y() << " " << relative_pos_table_frame_.z() << "\n";
        tf::Quaternion first_inverted = first_rotation.inverse();
        //std::cout << "first_inverted: " << first_inverted.x() << " " << first_inverted.y() << " " << first_inverted.z() << " " << first_inverted.w() << "\n";
        rotated_relative_pos = relative_pos_table_frame_.rotate(first_inverted.getAxis(), first_inverted.getAngle());
        //std::cout << "rotated_relative_pos: " << rotated_relative_pos.x() << " " << rotated_relative_pos.y() << " " << rotated_relative_pos.z() << "\n";
        //rotated_relative_rot = second_rotation;
    }
    else if (first_object_symmetry_ == "none" &&
            second_object_symmetry_ == "cubic")
    {
        rotated_relative_pos = relative_pos_table_frame_.rotate(prerotation_.getAxis(), prerotation_.getAngle());
    }
    else if (first_object_symmetry_ == "cubic" &&
            second_object_symmetry_ == "none")
    {
        tf::Quaternion first_inverted = first_rotation.inverse();
        rotated_relative_pos = relative_pos_table_frame_.rotate(first_inverted.getAxis(), first_inverted.getAngle());
    }
    else
    {
        rotated_relative_pos = relative_pos_table_frame_.rotate(prerotation1_.getAxis(), prerotation1_.getAngle());
    }

    
    


    connection_pose.position.x = rotated_relative_pos.x();
    connection_pose.position.y = rotated_relative_pos.y();
    connection_pose.position.z = rotated_relative_pos.z();

    //connection_pose.orientation.x = rotated_relative_rot.x();
    //connection_pose.orientation.y = rotated_relative_rot.y();
    //connection_pose.orientation.z = rotated_relative_rot.z();
    //connection_pose.orientation.w = rotated_relative_rot.w();

}
