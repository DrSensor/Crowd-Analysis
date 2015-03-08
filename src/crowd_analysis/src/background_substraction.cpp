#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <AdaptiveBackgroundLearning.h>

double fps=30;
cv::Mat src, img_buf;
cv_bridge::CvImage imgFG_msg;
cv_bridge::CvImage imgBG_msg;

IBGS *bgs;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        src = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "background_substraction");
    ros::NodeHandle node;

    argc--; argv++;
    while( argc && *argv[0] == '-' )
    {
        if( !strcmp(*argv, "-fps") && ( argc > 1 ) )
        {
            fps = atof(*(argv+1));
            printf("With %ffps\n",fps);
            argc--; argv++;
        }
        argc--; argv++;
    }

    bgs = new AdaptiveBackgroundLearning;
    imgBG_msg.encoding = "bgr8";
    imgFG_msg.encoding = "bgr8";

    ros::Publisher imgFG_pub = node.advertise<sensor_msgs::Image>
            ("camera/foreground_img", 1000);
    ros::Publisher imgBG_pub = node.advertise<sensor_msgs::Image>
                ("camera/background_img", 1000);
    image_transport::ImageTransport it(node);
    image_transport::Subscriber img_sub = it.subscribe("camera/image", 10, imageCallback);

    ros::Rate loop_rate(fps);

    while (ros::ok())
    {
        if ( ! src.empty()) {
            bgs->process(src, img_buf, imgBG_msg.image);
            cv::Mat element = cv::getStructuringElement( 2, cv::Size( 3, 3 ), cv::Point( 1, 1 ) );
            cv::morphologyEx( img_buf, img_buf, CV_MOP_OPEN, element );
            cv::cvtColor(img_buf,img_buf,CV_GRAY2BGR);
            src &= img_buf;
            imgFG_msg.image = src;
        }

        imgFG_pub.publish(imgFG_msg.toImageMsg());
        imgBG_pub.publish(imgBG_msg.toImageMsg());

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
