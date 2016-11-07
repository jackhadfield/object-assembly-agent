//myColourTracker.cpp

//Original code written by  Kyle Hounslow 2013
//Modified by Jack Hadfield 2016

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.


#include "myColourTracker.hpp"

ThingToFind::ThingToFind(std::vector<int> val, std::string strname)
{
	for (int i=0;i<6;i++)
		values[i] = val[i];
	name = strname;
}

ThingToFind::~ThingToFind()
{
}

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 180;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;


std::string intToString(int number) {
    	std::stringstream ss;
    	ss << number;
    	return ss.str();
}

void drawObject(int x, int y, Mat &frame, std::string objectName) {
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    Scalar colour;
    if (objectName=="Green Box")
        colour = Scalar(0, 255, 0);
    if (objectName=="Orange Box")
        colour = Scalar(0, 128, 255);
    if (objectName=="White Box")
        colour = Scalar(128, 128, 128);
    if (objectName=="Yellow Box")
        colour = Scalar(0, 255, 255);

	circle(frame, Point(x, y), 20, colour, 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), colour, 2);
	else line(frame, Point(x, y), Point(x, 0), colour, 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), colour, 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), colour, 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), colour, 2);
	else line(frame, Point(x, y), Point(0, y), colour, 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), colour, 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), colour, 2);

	putText(frame, objectName + intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, colour, 2);

}


void drawArrow(int x,int y, Mat &frame) {
    y = y-15;
    Scalar colour = Scalar(0, 0, 255);
    if (y-75>0 && x>0 && y<FRAME_HEIGHT && x<FRAME_WIDTH) {
        line(frame, Point(x, y), Point(x, y - 75), colour, 2);
        line(frame, Point(x, y), Point(x>25 ? x-25 : 1, y - 25), colour, 2);
        line(frame, Point(x, y), Point(FRAME_WIDTH-26>x ? x+25 : FRAME_WIDTH, y - 25), colour, 2);
    }
}

void checkTask(std::vector<double> x, std::vector<double> y, int &current_subtask, Mat &frame) {

    if (current_subtask==0) {
        if (x[0]!=0&&x[1]!=0&&y[0]!=0&&y[1]!=0) {
            if (abs(x[1]-x[0]) < 30 && y[0]-y[1]>20 && y[0]-y[1]<75) {
                current_subtask++;
                std::cout << "Subtask: " << current_subtask << " DONE!\n";
            }
            else {
                putText(frame, "Subtask 1: Place the orange box", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                putText(frame, "on the green box", Point(0, 80), 2, 1, Scalar(0, 255, 0), 2);
                drawArrow((int)x[0],(int)y[0],frame);
            }
        }
    }
    else if (current_subtask==1) {
        if (x[1]!=0&&x[2]!=0&&y[1]!=0&&y[2]!=0) {
            if (abs(x[2]-x[1]) < 30 && y[1]-y[2]>20 && y[1]-y[2]<75) {
                current_subtask++;
                std::cout << "Subtask: " << current_subtask << " DONE!\n";
            }
            else {
                putText(frame, "Subtask 2: Place the white box", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                putText(frame, "on the orange box", Point(0, 80), 2, 1, Scalar(0, 255, 0), 2);
                drawArrow((int)x[1],(int)y[1],frame);
            }
        }
    }
    else if (current_subtask==2) {
        if (x[2]!=0&&x[3]!=0&&y[2]!=0&&y[3]!=0) {
            if (abs(x[3]-x[2]) < 30 && y[2]-y[3]>20 && y[2]-y[3]<75) {
                current_subtask++;
                std::cout << "Subtask: " << current_subtask << " DONE!\n";
            }
            else {
                putText(frame, "Subtask 3: Place the yellow box", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                putText(frame, "on the white box", Point(0, 80), 2, 1, Scalar(0, 255, 0), 2);
                drawArrow((int)x[2],(int)y[2],frame);
            }
        }
    }
    else {
        putText(frame, "TASK COMPLETE!", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
    }
}

void mouse_callback(int  event, int  x, int  y, int  flag, void *param) {
    if (event == EVENT_LBUTTONDOWN)
    {
        // Store point coordinates
        Point pt;
        pt.x = x;
        pt.y = y;
        CLICKED_POINTS.push_back(pt);
        //NEWCOORDS = true;
    }
}

ColourTrackerNode::ColourTrackerNode(
            std::vector<std::string> object_names,
            std::vector<std::vector<int>> hsv_ranges,
            int flags[5],
            int tracker_params[5],
            std::vector<int> crop_range,
            int particle_filter_downsampling) : node_handle_("~"),
                                                crop_range_(crop_range),
                                                particle_filter_downsampling_(particle_filter_downsampling) {
    num_objects_ = hsv_ranges.size();
    for (int i = 0; i < hsv_ranges.size(); i++) {
        ThingToFind box(hsv_ranges[i], object_names[i]);
        boxes_.push_back(box);
    }

    std::vector<int> temp; //store some zeros
    for (int j = 0; j < pastValues_; j++)
        temp.push_back(0);
    for (int i = 0; i < num_objects_; i++) {
        x_.push_back(0);
        y_.push_back(0);
        past_x_.push_back(temp);
        past_y_.push_back(temp);
        x_estimate_.push_back(0);
        y_estimate_.push_back(0);
        scores_.push_back(0);
    }
    show_RGB_ = flags[0] == 1;
    show_HSV_ = flags[1] == 1;
    show_threshold_ = flags[2] == 1;
    useTrackbars_ = flags[3] == 1;
    smooth_estimate_ = flags[4] == 1;
    MAX_NUM_OBJECTS_ = tracker_params[0];
    MIN_OBJECT_AREA_ = tracker_params[1];
    MAX_OBJECT_AREA_ = tracker_params[2];
    erode_size_ = tracker_params[3];
    dilate_size_ = tracker_params[4];

    //create slider bars for HSV filtering
    if (useTrackbars_) 
        createTrackbars();

    position_estimates_publisher_ = node_handle_.advertise<object_assembly_msgs::Points2D>("objects_Points2D", 0);
    input_publisher_ = node_handle_.advertise<object_assembly_msgs::Input>("input", 0);
}


void ColourTrackerNode::colour_tracker_callback(const sensor_msgs::ImageConstPtr& ros_image) {
    Mat cameraFeed;
    try {
        (cv_bridge::toCvShare(ros_image,"bgr8")->image).copyTo(cameraFeed);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", ros_image->encoding.c_str());
    }
    Rect r1(crop_range_[0], crop_range_[1], crop_range_[2], crop_range_[3]);
    cameraFeed = cameraFeed(r1);
    doStuff(cameraFeed);
}


void ColourTrackerNode::publish() {
    object_assembly_msgs::Points2D points;
    if (CLICKED_POINTS.size() < num_objects_) {
        for (int i = 0; i < x_.size(); i++) {
            object_assembly_msgs::Point2D point;
            point.x = x_estimate_[i];
            point.y = y_estimate_[i];
            point.name = boxes_[i].name;
            points.points.push_back(point);
        }
    }
    else {
        for (int i = 0; i < x_.size(); i++) {
            object_assembly_msgs::Point2D point;
            point.x = CLICKED_POINTS[i].x;
            point.y = CLICKED_POINTS[i].y;
            point.name = boxes_[i].name;
            points.points.push_back(point);
        }
    }
    position_estimates_publisher_.publish(points);
    
    object_assembly_msgs::Input input;
    for (int i = 0; i < num_objects_; i++) {
//TODO Add resize option (currently for sd colour, hd depth)
        input.input.push_back(3.75*points.points[i].x/particle_filter_downsampling_);
        input.input.push_back(2.547*points.points[i].y/particle_filter_downsampling_);
        input.input.push_back(0.7); //TODO remove this line
    }
    for (int i = 0; i < num_objects_; i++) {
//TODO Add resize option (currently for sd colour, hd depth)
        input.input.push_back(scores_[i]);
    }
    input_publisher_.publish(input);
    
}


void ColourTrackerNode::on_trackbar(int, void*) {
    //This function gets called whenever a
    // trackbar position is changed
}


void ColourTrackerNode::createTrackbars() {
    //create window for trackbars

    namedWindow(trackbarWindowName, 0);
    //create memory to store trackbar name on window

    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH), 
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->      
    createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
    createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
    createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
    createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
    createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
    createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);

}


void ColourTrackerNode::morphOps(Mat &thresh) {
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(erode_size_, erode_size_));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(dilate_size_, dilate_size_));

    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);

    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
}


double ColourTrackerNode::smoothEstimate(std::vector<int> &past_values,
                          int current_value) {
    double estimate = 0.0312*current_value; //extra to add up to 1
    past_values.erase(past_values.begin(), past_values.begin()+1);
    past_values.push_back(current_value);
    for (int k=past_values.size()-1;k>=0;k--) {
        estimate = estimate + past_values[k]/pow(2,k+1);
    }
    return estimate;
}


bool ColourTrackerNode::trackFilteredObject(int &x, int &y, Mat threshold,
                             Mat &cameraFeed, std::string objectName, double &score) {
    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    std::vector< std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    double refArea = 0;
    double totalArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
    	int numObjects = hierarchy.size();
    	//if number of objects greater than MAX_NUM_OBJECTS_ we have a noisy filter
        if (numObjects<MAX_NUM_OBJECTS_) {
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;
                totalArea += area;                

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if (area>MIN_OBJECT_AREA_ && area<MAX_OBJECT_AREA_ && area>refArea) {
                    x = moment.m10 / area;
                    y = moment.m01 / area;
                    objectFound = true;
                    refArea = area;
                }
                else ;//objectFound = false;

            }
    /*
            //let user know you found an object
            if (objectFound == true) {
                putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                //draw object location on screen
                drawObject(x, y, cameraFeed, objectName);
            }
    */

        }
        else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
    }
    if (totalArea != 0)
        score = refArea/totalArea;
    else
        score = 0;
    return objectFound;
}


void ColourTrackerNode::doStuff(Mat &cameraFeed) {
    //convert frame from BGR to HSV colorspace
    cvtColor(cameraFeed, HSV_, COLOR_BGR2HSV);
	if (show_RGB_) {
        namedWindow(windowName, WINDOW_AUTOSIZE);
        setMouseCallback(windowName, mouse_callback);
    }
    //repeat for all objects
    for (int obj = 0; obj < num_objects_; obj++) {
        for (int i = 0; i < 6; i++)
		    minmax_[i] = boxes_[obj].values[i];
		objectName_ = boxes_[obj].name;
        //filter HSV image between values and store filtered 
		//image to threshold matrix
        if (useTrackbars_)
            inRange(HSV_, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold_);
        else
            inRange(HSV_, Scalar(minmax_[0], minmax_[2], minmax_[4]), Scalar(minmax_[1], minmax_[3], minmax_[5]), threshold_);
            
        if (useMorphOps_)
            //TODO: remove this part very soon!!!
            if (obj == 3)
            {
                Mat erodeElement = getStructuringElement(MORPH_RECT, Size(1, 1));
                Mat dilateElement = getStructuringElement(MORPH_RECT, Size(dilate_size_, dilate_size_));

                erode(threshold_, threshold_, erodeElement);
                erode(threshold_, threshold_, erodeElement);

                dilate(threshold_, threshold_, dilateElement);
                dilate(threshold_, threshold_, dilateElement);
            }
            else
                morphOps(threshold_);
        if (trackObjects_) {
            if (trackFilteredObject(x_[obj], y_[obj], threshold_, cameraFeed, objectName_, scores_[obj])) {
            //let user know you found an object
                if (smooth_estimate_) {
                    x_estimate_[obj] = smoothEstimate(past_x_[obj], x_[obj]);
                    y_estimate_[obj] = smoothEstimate(past_y_[obj], y_[obj]);
                }
                else {
                    x_estimate_[obj] = (double)x_[obj];
                    y_estimate_[obj] = (double)y_[obj];
                }
                //draw object location on screen
                drawObject((int)x_estimate_[obj], (int)y_estimate_[obj], cameraFeed, objectName_);
            }
        }
    }

    //checkTask(x_estimate_, y_estimate_, current_subtask_, cameraFeed);

    publish();
        
    if (show_threshold_)
        imshow(windowName2, threshold_);
    if (show_RGB_)
        imshow(windowName, cameraFeed);
    if (show_HSV_)
        imshow(windowName1, HSV_);

    waitKey(30);
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "colour_tracker");
    ros::NodeHandle nh("~");

    //Read params from config file
    int num_objects;
    nh.getParam("number_of_objects", num_objects);
    std::vector<std::string> object_names;
    std::vector<std::vector<int>> hsv_ranges;
    for (int i = 0; i < num_objects; i++) {
        std::string object_pre = std::string("objects/") + "object" + std::to_string(i+1) + "/";
        std::string object_name;
        std::vector<int> hsv_range;
        nh.getParam(object_pre + "name", object_name);
        object_names.push_back(object_name);
        nh.getParam(object_pre + "hsv_range", hsv_range);
        hsv_ranges.push_back(hsv_range);        
    }
    std::string camera_topic;
    nh.getParam("camera_topic", camera_topic);
    int flags[5];
    nh.getParam("show_rgb", flags[0]);
    nh.getParam("show_hsv", flags[1]);
    nh.getParam("show_threshold", flags[2]);
    nh.getParam("use_trackbars", flags[3]);
    nh.getParam("smooth_estimate", flags[4]);

    int tracker_params[5];
    nh.getParam("MAX_NUM_OBJECTS", tracker_params[0]);
    nh.getParam("MIN_OBJECT_AREA", tracker_params[1]);
    nh.getParam("MAX_OBJECT_AREA", tracker_params[2]);
    nh.getParam("erode_size", tracker_params[3]);
    nh.getParam("dilate_size", tracker_params[4]);

    std::vector<int> crop_range;
    nh.getParam("crop_range", crop_range);
    int particle_filter_downsampling;
    nh.getParam("particle_filter_downsampling", particle_filter_downsampling);

    ColourTrackerNode colour_tracker_node(object_names, hsv_ranges, flags, tracker_params, crop_range, particle_filter_downsampling);

    ros::Subscriber subscriber = nh.subscribe(camera_topic, 1, &ColourTrackerNode::colour_tracker_callback, &colour_tracker_node);

    ros::spin();
}
