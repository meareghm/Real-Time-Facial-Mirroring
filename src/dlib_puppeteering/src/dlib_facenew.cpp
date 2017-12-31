#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <math.h>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <unistd.h>
#include <vector>

#include <cstdlib> //random
#include <iostream> //random

#include <dlib_puppeteering/lm_points.h>


using namespace std;
using namespace cv;
using namespace dlib;


double lmPoints[68][2];

double max_valueaR = 0;
double max_valueaL = 0;

bool publish = false;

double bvaR = 0;
double roundOff_bvaR = 0.000;

double bvaL = 0;
double roundOff_bvaL = 0.000;

std::vector<float> dlibX;
std::vector<float> dlibY;
std::vector<int> dlibFaceIndex;//face count

double dlibXAvg, dlibYAvg, sumX, sumY;

dlib::image_window win;
dlib::frontal_face_detector detector;
dlib::shape_predictor pose_model;
ros::Publisher pub;

void dlibCallback(const sensor_msgs::ImageConstPtr& msg) {

    // Declare a variable to hold converted image(from ros to opencv)
    cv_bridge::CvImagePtr cvPtr;
    try {
        // Convert ros image into opencv matrix image
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Declare a variable to hold an image frame and grab a frame
    cv::Mat temp;
    temp = cvPtr->image;

    // Turn OpenCV's Mat image into something dlib can deal with
    dlib::cv_image<dlib::bgr_pixel> cimg(temp);

    // Detect faces 
    std::vector<dlib::rectangle> faces = detector(cimg);

    // Find the pose of each face.
    std::vector<dlib::full_object_detection> shapes;

    for (unsigned long i = 0; i < faces.size(); ++i)
        shapes.push_back(pose_model(cimg, faces[i]));

    // Display it all on the screen
    win.clear_overlay();
    win.set_image(cimg);
    win.add_overlay(render_face_detections(shapes));

    //Declare variables to process and store messages to be published
    dlib_puppeteering::lm_points dlibValues;

    //Enters if any shape detected has dlib drawn overlays
    if(shapes.size()>0){
        dlibX.clear();
        dlibY.clear();
        dlibFaceIndex.clear();

        // Format the values to be published on "/dlib_values"
        // for all/each faces on the frame store dlib values(x and y), dlib count-no. of faces on the frame
        // the vectors dlibX and dlibY contains dlib values for all faces on the frame at once in the order of
        // of the detected face first, second and etc... so ets size=no_of_face * 68-facelandmarks.

        for (unsigned long i = 0; i < shapes.size(); ++i) // Iterate through shapes
        {                    
            const full_object_detection& d = shapes[i]; // Hold the detected face i
            for (unsigned long j = 0; j < 68; ++j) // Hold all the 68 face landmark coordinates(x, y)
            {
                lmPoints[j][0] = (double)d.part(j).x();
                lmPoints[j][1] = (double)d.part(j).y();
                dlibX.push_back(lmPoints[j][0]);
                dlibY.push_back(lmPoints[j][1]);
            }
            //Normalization
            //better to do normalization here than doing inside mapper because
            //if the number of faces are greater than one then separating the first face
            //values in mapper is tedious to do normalization and publish the normalized values.

            dlibXAvg = 0;
            dlibYAvg = 0;

            /*for(unsigned long a = 0; a < 68; ++a){
                dlibXAvg = dlibXAvg+lmPoints[a][0];
                dlibYAvg = dlibYAvg+lmPoints[a][1];
            }
            dlibXAvg = dlibXAvg/68;
            dlibYAvg = dlibYAvg/68;*/

            dlibFaceIndex.push_back(i);
            /*for (unsigned long c = 0; c < 68; ++c)
            {
                dlibX.push_back(lmPoints[c][0] - dlibXAvg);
                dlibY.push_back(lmPoints[c][1] - dlibYAvg);
            }*/
        }

        std::cout<<"\nDlib-INFO: Processing Face Detected!"<<std::endl;

        dlibValues.dlib_X = dlibX;
        dlibValues.dlib_Y = dlibY;
        dlibValues.dlib_face_index = dlibFaceIndex;

        pub.publish(dlibValues);
    } 
    else{
        std::cout<<"\nDlib-INFO: No Face Detected!"<<std::endl;
    }
}

int main(int argc, char **argv) {
    std::string homedir = getenv("HOME");
    if (homedir != "") {
        std::cout<<"Home directory set to \""<<homedir<<"\""<<endl;

        detector = dlib::get_frontal_face_detector();
        dlib::deserialize(homedir+"/shape_predictor_68_face_landmarks.dat") >> pose_model;
        ros::init(argc, argv, "dlib_puppeteering_node");
        ros::NodeHandle nh;
        pub = nh.advertise<dlib_puppeteering::lm_points>("/dlib_values", 1000);
        ros::Subscriber sub = nh.subscribe("/usb_cam_node/image_raw", 1, dlibCallback);
        ros::spin();
    }
    else{
        std::cout<<"Unable to locate home directory... \nplease set home directory..."<<endl;
    }
}

