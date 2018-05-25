#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void kinect_callback(sensor_msgs::ImageConstPtr msgs){
    cv::imshow("kinect",cv_bridge::toCvCopy(msgs)->image);
    cv::waitKey(1);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"kinect_viewer");
    ros::NodeHandle n;

    // TODO
    // write a code of subscriber that gets message from "/camera/depth/image_raw" topic and then call kinect_callback function
    // it's just a simple line.
    ros::Subscriber sub = n.subscribe("/camera/depth/image_raw", 1, kinect_callback);

    cv::namedWindow("kinect");

    ros::spin();
    return 0;
}
