#include <sstream>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <vector> 
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <object_assembly_msgs/Point2D.h>
#include <object_assembly_msgs/Points2D.h>
#include <object_assembly_msgs/Input.h>


using namespace cv;
using namespace xfeatures2d;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
std::vector<Point> CLICKED_POINTS;
//int NUM_CLICKED_POINTS = 0;

class ThingToFind
{
public:
	ThingToFind(std::vector<int> val,std::string name);
	~ThingToFind();
	int values[6];
	std::string name;
};

std::string intToString(int number);

void drawObject(int x, int y, Mat &frame, std::string objectName);

void drawArrow(int x,int y, Mat &frame);

void checkTask(std::vector<double> x, std::vector<double> y, int &current_subtask, Mat &frame);

void mouse_callback(int  event, int  x, int  y, int  flag, void *param);

class ColourTrackerNode {

public:
    ColourTrackerNode(std::vector<std::string> object_names,
                      std::vector<std::vector<int>> hsv_ranges,
                      int flags[5],
                      int tracker_params[5],
                      std::vector<int> crop_range,
                      int particle_filter_downsampling);

    void colour_tracker_callback(const sensor_msgs::ImageConstPtr& ros_image);

    void publish();

    static void on_trackbar(int, void*);

    void createTrackbars();

    void morphOps(Mat &thresh);

    double smoothEstimate(std::vector<int> &past_values,
                          int current_value);

    bool trackFilteredObject(int &x, int &y, Mat threshold,
                             Mat &cameraFeed, std::string objectName, double &score);

    void doStuff(Mat &cameraFeed);

private:
    //max number of objects to be detected in frame
    int MAX_NUM_OBJECTS_; // = 50;
    //minimum and maximum object area
    int MIN_OBJECT_AREA_; // = 10 * 10;
    int MAX_OBJECT_AREA_; // = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
    //names that will appear at the top of each window
    const std::string windowName = "Original Image";
    const std::string windowName1 = "HSV Image";
    const std::string windowName2 = "Thresholded Image";
    const std::string windowName3 = "After Morphological Operations";
    const std::string trackbarWindowName = "Trackbars";

    int num_objects_;
    std::vector<int> x_, y_;
    std::vector<std::vector<int>> past_x_, past_y_;
	int minmax_[6];
    std::vector<double> x_estimate_, y_estimate_;
	std::string objectName_;
    std::vector<ThingToFind> boxes_;
    int pastValues_ = 5;
    std::vector<double> scores_;

    int current_subtask_=0;
	//some boolean variables for different functionality within this
	//program
    bool show_RGB_;
    bool show_HSV_;
    bool show_threshold_;
	bool trackObjects_ = true;
	bool useMorphOps_ = true;
    bool useTrackbars_ = false;
    bool smooth_estimate_;
    int erode_size_;
    int dilate_size_;
	//matrix storage for HSV image
	Mat HSV_;
	//matrix storage for binary threshold image
	Mat threshold_;

    std::vector<int> crop_range_;
    int particle_filter_downsampling_;

    ros::NodeHandle node_handle_;
    ros::Publisher position_estimates_publisher_;
    ros::Publisher input_publisher_;
};

