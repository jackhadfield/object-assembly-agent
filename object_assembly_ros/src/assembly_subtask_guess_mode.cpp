#include <assembly_subtask_guess_mode.hpp>
//TODO rotate desired relative pose, not received poses, to save time

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
    rot_axis_ = down_vector.cross(tf::Vector3(0,1,0));
    rot_angle_ = down_vector.angle(tf::Vector3(0,1,0));
    //std::cout << "angle: " << rot_angle_ << "\n";
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

    //first_rotation = tf::Quaternion(0.0, 0.0, 0.0, 1.0);
    //second_rotation = tf::Quaternion(0.0, 0.0, 0.0, 1.0);
}

bool AssemblySubtask::evaluate_subtask(
        std::vector<geometry_msgs::Pose> &current_object_poses,
        double &score) {
    tf::Transform first_object_pose, second_object_pose;
    std::vector<double> first_object_position, second_object_position;
    tf::Quaternion first_object_orientation(), 
                   second_object_orientation();

    //first_object_position.push_back (
    //        current_object_poses[first_object_].position.x);
    //first_object_position.push_back (
    //        current_object_poses[first_object_].position.y);
    //first_object_position.push_back (
    //        current_object_poses[first_object_].position.z);
    tf::Quaternion q1 (
            current_object_poses[first_object_].orientation.x,
            current_object_poses[first_object_].orientation.y,
            current_object_poses[first_object_].orientation.z,
            current_object_poses[first_object_].orientation.w);
    first_object_pose.setRotation (q1);
    tf::Vector3 v1 (
            current_object_poses[first_object_].position.x,
            current_object_poses[first_object_].position.y,
            current_object_poses[first_object_].position.z);
    first_object_pose.setOrigin (v1);

    //second_object_position.push_back (
    //        current_object_poses[second_object_].position.x);
    //second_object_position.push_back (
    //        current_object_poses[second_object_].position.y);
    //second_object_position.push_back (
    //        current_object_poses[second_object_].position.z);
    tf::Quaternion q2 (
            current_object_poses[second_object_].orientation.x,
            current_object_poses[second_object_].orientation.y,
            current_object_poses[second_object_].orientation.z,
            current_object_poses[second_object_].orientation.w);
    second_object_pose.setRotation (q2);
    tf::Vector3 v2 (
            current_object_poses[second_object_].position.x,
            current_object_poses[second_object_].position.y,
            current_object_poses[second_object_].position.z);
    second_object_pose.setOrigin (v2);

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

bool AssemblySubtask::evaluate_any(tf::Transform first_object_pose,
                                   tf::Transform second_object_pose,
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

bool AssemblySubtask::evaluate_no_symmetry(
                                  tf::Transform first_object_pose,
                                  tf::Transform second_object_pose,
                                  double &score)
{
//TODO build tf here and accept origins and rotations as parameters
    bool position_check1, position_check2, orientation_check = true;
    double position_score1, position_score2, orientation_score = 1.0;

    tf::Transform rotated_first_object_pose = first_object_pose * 
                                              tf::Transform(prerotation1_, tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform t21(rotated_first_object_pose);
    t21 = t21.inverseTimes(second_object_pose); //TODO: check..
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
    bool positionXY_check, positionZ_check, orientation_check = true;
    double positionXY_score, positionZ_score, orientation_score = 1;
//std::cout << "Eval cubic cubic\n";
    tf::Transform t21(first_object_pose);
    t21 = t21.inverseTimes(second_object_pose); //TODO: check..
    //relative_posXY = tf::Vector3(t21.getOrigin().getX(), t21.getOrigin().getY(), 0);
    //relative_posZ = tf::Vector3(0, 0, t21.getOrigin().getZ());
//std::cout << "1st: " << first_object_pose.getOrigin().getX() << " " << first_object_pose.getOrigin().getY() << " " << first_object_pose.getOrigin().getZ() << "\n";
//std::cout << "2nd: " << second_object_pose.getOrigin().getX() << " " << second_object_pose.getOrigin().getY() << " " << second_object_pose.getOrigin().getZ() << "\n";
    tf::Vector3 position_diff = t21.getOrigin();
    tf::Vector3 sorted_position_diff = position_diff.absolute();
    //... = t21.getOrigin().getX() > 0;
    //... = t21.getOrigin().getY() > 0;
    //... = t21.getOrigin().getZ() > 0;
    sort_vector3(sorted_position_diff);

    positionXY_check = compare_tf_vectors(relativeXZ_sorted_,
                                        sorted_position_diff,
                                        margin_[0],
                                        margin_[1],
                                        margin_[2],
                                        positionXY_score);
    //std::cout << "compared\n";
    tf::Vector3 first_pos = first_object_pose.getOrigin();
    first_pos.rotate(rot_axis_, rot_angle_);
    tf::Vector3 second_pos = second_object_pose.getOrigin();
    second_pos.rotate(rot_axis_, rot_angle_);
    positionZ_check = compare_tf_vectors(relativeY_,
                                        tf::Vector3(0, 0, second_pos.getY() - first_pos.getY()),
                                        margin_[0],
                                        margin_[1],
                                        margin_[2],
                                        positionZ_score);
    //TODO: decide where to put this
    if (positionXY_check && positionZ_check)
    {
        //Align position_diff with relative_position_, so that y axis points down
        //std::cout << "Checking angle " << relative_position_.angle(position_diff) << "\n";
        tf::Vector3 down_vector_in_frame;// = -up_vector_;
        down_vector_in_frame = first_object_pose * tf::Vector3(0.0, 1.0, 0.0) - first_object_pose.getOrigin();
        //std::cout << "down vector: " << down_vector_in_frame.getX() << " " << down_vector_in_frame.getY() << " " << down_vector_in_frame.getZ() << "\n";
        if (std::abs(relative_position_.angle(position_diff) - M_PI) < 0.001) //TODO: check maybe topsy turvy
        {
            //std::cout << "180 degrees!!!\n";
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
            //std::cout << "first_rotation: " << first_rotation.getAxis().getX() << " " << first_rotation.getAxis().getY() << " " << first_rotation.getAxis().getZ() << " " << first_rotation.getAngle() << "\n";
            //std::cout << "down vector: " << down_vector_in_frame.getX() << " " << down_vector_in_frame.getY() << " " << down_vector_in_frame.getZ() << "\n";;
            //std::cout << "final_down_vector: " << down_vector_in_frame.rotate(down_vector_in_frame.cross(-up_vector_), down_vector_in_frame.angle(-up_vector_)).getX() << " " << down_vector_in_frame.rotate(down_vector_in_frame.cross(-up_vector_), down_vector_in_frame.angle(-up_vector_)).getY() << " " << down_vector_in_frame.rotate(down_vector_in_frame.cross(-up_vector_), down_vector_in_frame.angle(-up_vector_)).getZ() << "\n";
        }
        else if (std::abs(relative_position_.angle(position_diff)) < 0.001)
        {
            //std::cout << "0 degrees!!!\n";
        }
        else
        {
            first_rotation.setRotation(position_diff.cross(relative_position_), position_diff.angle(relative_position_));
            down_vector_in_frame = down_vector_in_frame.rotate(position_diff.cross(relative_position_), -position_diff.angle(relative_position_));
//std::cout << "first_rotation: " << first_rotation.getAxis().getX() << " " << first_rotation.getAxis().getY() << " " << first_rotation.getAxis().getZ() << " " << first_rotation.getAngle() << "\n";
            //first_rotation.normalize();
            //second_rotation.normalize();
            //tf::Quaternion temp_quat(down_vector_in_frame.cross(-up_vector_), -down_vector_in_frame.angle(-up_vector_)); //TODO will this work generally or only for 90 degrees?
            //std::cout << "temp_quat: " << temp_quat.getAxis().getX() << " " << temp_quat.getAxis().getY() << " " << temp_quat.getAxis().getZ() << " " << temp_quat.getAngle() << "\n";
        }
        if (std::abs(down_vector_in_frame.angle(-up_vector_) - M_PI) < 0.001)
            first_rotation = tf::Quaternion(relative_position_, M_PI) * first_rotation;
        else if (!std::abs(down_vector_in_frame.angle(-up_vector_)) < 0.001)
            first_rotation = tf::Quaternion(down_vector_in_frame.cross(-up_vector_), -down_vector_in_frame.angle(-up_vector_)) * first_rotation;
        second_rotation = t21.getRotation();
    }

//std::cout << "made new rotations\n";
//TODO: combine scores
    /*
    auto abs_dot_product = std::abs(relative_orientation_.dot(t21.getRotation()));
    orientation_check = (abs_dot_product > 1 - margin_[3] ||
        abs_dot_product < margin_[3] ||
        (abs_dot_product > 0.5 - margin_[3] && 
        abs_dot_product < 0.5 + margin_[3]) || 
        (abs_dot_product > 0.70711 - margin_[3] && 
        abs_dot_product < 0.70711 + margin_[3]));
    orientation_score = 1;
    */
    //TODO fix
    score = positionXY_score * positionZ_score * orientation_score;//std::cout << "calculated scores\n";
    return positionXY_check && positionZ_check && orientation_check;
}

bool AssemblySubtask::evaluate_cubic_none(
                                  tf::Transform first_object_pose,
                                  tf::Transform second_object_pose,
                                  double &score)
{
    bool positionXY_check, positionZ_check = 1, orientation_check;
    double positionXY_score, positionZ_score = 1.0, orientation_score;

    tf::Transform rotated_second_object_pose = second_object_pose *
                                               tf::Transform(prerotation_, tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform t21(rotated_second_object_pose);
    t21 = t21.inverseTimes(first_object_pose);

    tf::Vector3 position_diff = t21.getOrigin();

    positionXY_check = compare_tf_vectors(-relative_position_,
                                          position_diff,
                                          margin_[0],
                                          margin_[1],
                                          margin_[2],
                                          positionXY_score);

    if (positionXY_check)
        first_rotation = second_object_pose.inverseTimes(first_object_pose).getRotation();

    orientation_score = 1; //TODO fix
    orientation_check = true;
    score = positionXY_score * positionZ_score * orientation_score;
    return positionXY_check && positionZ_check && orientation_check;
}

bool AssemblySubtask::evaluate_none_cubic(
                                  tf::Transform first_object_pose,
                                  tf::Transform second_object_pose,
                                  double &score)
{
    bool positionXY_check, positionZ_check = 1, orientation_check;
    double positionXY_score, positionZ_score = 1.0, orientation_score;

    tf::Transform rotated_first_object_pose = first_object_pose * 
                                              tf::Transform(prerotation_, tf::Vector3(0.0, 0.0, 0.0));

    //std::cout << "transform: " << rotated_first_object_pose.getRotation().getAngle() << "\n";
    //std::cout << "transform: " << rotated_first_object_pose.getRotation().getAxis().getX() << " " << rotated_first_object_pose.getRotation().getAxis().getY() << " " << rotated_first_object_pose.getRotation().getAxis().getZ() << "\n";

    tf::Transform t21(rotated_first_object_pose);
    t21 = t21.inverseTimes(second_object_pose); //TODO: check..

    tf::Vector3 position_diff = t21.getOrigin();

    positionXY_check = compare_tf_vectors(relative_position_,
                                          position_diff,
                                          margin_[0],
                                          margin_[1],
                                          margin_[2],
                                          positionXY_score);
    if (positionXY_check)
        second_rotation = first_object_pose.inverseTimes(second_object_pose).getRotation();
//std::cout << "XY check: " << positionXY_check << " , Z check: " << positionZ_check << "\n";
//std::cout << "position_diff: " << position_diff.getX() << " " << position_diff.getY() << " " << position_diff.getZ() << "\n";
    orientation_score = 1; //TODO fix
    orientation_check = true;
    score = positionXY_score * positionZ_score * orientation_score;
    return positionXY_check && positionZ_check && orientation_check;
}

bool AssemblySubtask::compare_tf_vectors(tf::Vector3 v1,
                                         tf::Vector3 v2,
                                         double x_margin,
                                         double y_margin,
                                         double z_margin,
                                         double &score)
{
    const int N = 8;
    const double ymax = 100;
    const double sqrt3 = 1.732050808;
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
