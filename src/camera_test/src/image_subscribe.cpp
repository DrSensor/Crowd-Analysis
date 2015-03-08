#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

void showCallback(const cv_bridge::CvImage& img_msg)
{
    cv::imshow("image", img_msg.image);
    cv::waitKey(5);
}

int main(int argc, char *argv[])
{

    return 0;
}
