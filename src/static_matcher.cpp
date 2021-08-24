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

using namespace cv;
using namespace std;



int main(int argc, char **argv)
{   
    ros::init(argc, argv, "static_matcher");
    ros::NodeHandle nh_static;

    int numDisparities;
    int blockSize;
    int disp12MaxDiff;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int mindisaprity;
    int prefilterCap;
    string name1;
    string name2;

    Mat img1, img2, disp;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create();
    namedWindow("MyWindow", WINDOW_NORMAL);

    cout<<"Images Loaded\n";

    ros::Rate loop_rate(10);
    while(ros::ok()){
        bool param_success = true; 

        param_success &= nh_static.getParam("numDisparities", numDisparities);
        param_success &= nh_static.getParam("SADWindowSize", blockSize);
        param_success &= nh_static.getParam("uniquenessRatio", uniquenessRatio);
        param_success &= nh_static.getParam("speckleWindowSize", speckleWindowSize);
        param_success &= nh_static.getParam("speckleRange", speckleRange);
        param_success &= nh_static.getParam("disp12MaxDiff", disp12MaxDiff);
        param_success &= nh_static.getParam("mindisaprity", mindisaprity);
        param_success &= nh_static.getParam("prefilterCap", prefilterCap);
        param_success &= nh_static.getParam("name1", name1);
        param_success &= nh_static.getParam("name2", name2);

        if(param_success){
            ROS_INFO("Parameters Loaded.");
        }
        img1 = imread(name1,1);
        img2 = imread(name2,1);

        cout<<"Images Loaded."<<name1<<endl;

        sgbm->setMinDisparity(mindisaprity);
        sgbm->setPreFilterCap(prefilterCap);
        sgbm->setP1(8 * (2 * blockSize + 1) * (2 * blockSize + 1));
        sgbm->setP2(32 * (2 * blockSize + 1) * (2 * blockSize + 1));
        sgbm->setBlockSize((2 * blockSize + 1));
        sgbm->setNumDisparities(16 * numDisparities);
        sgbm->setUniquenessRatio(uniquenessRatio);
        sgbm->setSpeckleWindowSize(speckleWindowSize);
        sgbm->setSpeckleRange(speckleRange);
        sgbm->setDisp12MaxDiff(disp12MaxDiff);
        sgbm->compute(img1, img2, disp);

        normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);

        imshow("MyWindow", disp);
        waitKey(10);
    }
    
    loop_rate.sleep();

    destroyAllWindows();
    return 0;
}