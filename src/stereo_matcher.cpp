#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stereo.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <stdio.h>
#include <string.h>
//#include "opencv2/contrib/contrib.hpp"
using namespace message_filters;
using namespace cv;
using namespace std;

int numDisparities;
int blockSize;
int disp12MaxDiff;
int uniquenessRatio;
int speckleWindowSize;
int speckleRange;

void Callback(ros::NodeHandle &node_handle, const sensor_msgs::Image::ConstPtr &msg1, const sensor_msgs::Image::ConstPtr &msg2)
{
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;
    Mat img1, img2, g1, g2;
    Mat disp, disp8;
    namedWindow("left", WINDOW_AUTOSIZE);
    namedWindow("right", WINDOW_AUTOSIZE);
    namedWindow("disp", WINDOW_AUTOSIZE);

    cv_ptr1 = cv_bridge::toCvCopy(msg1,sensor_msgs::image_encodings::BGR8);
    cv_ptr2 = cv_bridge::toCvCopy(msg2,sensor_msgs::image_encodings::BGR8);

    img1 = cv_ptr1->image;
    img2 = cv_ptr2->image;

    cvtColor(img1, g1, COLOR_BGR2GRAY);
    cvtColor(img2, g2, COLOR_BGR2GRAY);

    bool param_success = true; 

    param_success &= node_handle.getParam("numDisparities", numDisparities);
    param_success &= node_handle.getParam("SADWindowSize", blockSize);
    param_success &= node_handle.getParam("uniquenessRatio", uniquenessRatio);
    param_success &= node_handle.getParam("speckleWindowSize", speckleWindowSize);
    param_success &= node_handle.getParam("speckleRange", speckleRange);
    param_success &= node_handle.getParam("disp12MaxDiff", disp12MaxDiff);

    if(param_success){
        ROS_INFO("Parameters Loaded.");
    }

    Ptr<StereoSGBM> sgbm = StereoSGBM::create();

    sgbm->setMinDisparity(0);
    sgbm->setPreFilterCap(31);
    sgbm->setP1(8 * (2 * blockSize + 1) * (2 * blockSize + 1));
    sgbm->setP2(32 * (2 * blockSize + 1) * (2 * blockSize + 1));
    sgbm->setBlockSize((2 * blockSize + 1));
    sgbm->setNumDisparities(16 * numDisparities);
    sgbm->setUniquenessRatio(uniquenessRatio);
    sgbm->setSpeckleWindowSize(speckleWindowSize);
    sgbm->setSpeckleRange(speckleRange);
    sgbm->setDisp12MaxDiff(disp12MaxDiff);
    sgbm->compute(g1, g2, disp);

    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    imshow("left", img1);
    imshow("right", img2);
    imshow("disp", disp8);

    waitKey(5000);
    destroyAllWindows();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_matcher");
    ros::NodeHandle nh_stereo_match;
    message_filters::Subscriber<sensor_msgs::Image> input1(nh_stereo_match, "/narrow_stereo_textured/left/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> input2(nh_stereo_match, "/narrow_stereo_textured/right/image_raw", 10);
    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(input1, input2, 10);
    ROS_INFO("Going into callback..");
    sync.registerCallback(boost::bind(&Callback,boost::ref(nh_stereo_match), _1, _2));

    ros::spin();
    return 0;
}