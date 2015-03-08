#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt16.h>
#include <opencv2/objdetect/objdetect.hpp>

double fps=30;
int max_detected = -1;
std::string file_dataset = "haarcascade_upperbody.xml";

cv_bridge::CvImage img_msg;
std_msgs::UInt16 people_count;

cv::CascadeClassifier classifier;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        img_msg.image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "people_counting");
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
        if( !strcmp(*argv, "-crowd_dataset") && ( argc > 1 ) )
        {
            file_dataset = *(argv+1);
            printf("Using dataset form %s\n",file_dataset.c_str());
            argc--; argv++;
        }
        argc--; argv++;
    }

    std::vector<cv::Rect> objects;
    std::vector<cv::Scalar> colors;
    cv::RNG rng(12345);

    img_msg.encoding = "bgr8";

    if ( ! classifier.load(file_dataset))
        ROS_ERROR("Can't open %s", file_dataset.c_str());

    ros::Publisher img_pub = node.advertise<sensor_msgs::Image>
            ("crowd/count_img", 1000);
    ros::Publisher count_pub = node.advertise<std_msgs::UInt16>
            ("crowd/people_count", 1000);

    image_transport::ImageTransport it(node);
    image_transport::Subscriber img_sub = it.subscribe("camera/image", 1, imageCallback);

    ros::Rate loop_rate(fps);

    while (ros::ok())
    {
        if (! img_msg.image.empty()) {
            classifier.detectMultiScale(img_msg.image, objects, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
            for (int j = 0; j < objects.size(); ++j) {
                if ( j > max_detected) {
                    colors.push_back(cv::Scalar(rng.uniform(0,255),rng.uniform(0, 255),rng.uniform(0, 255)));
                    max_detected = j;
                }
                cv::rectangle(img_msg.image, objects[j], colors[j], 2);
            }
            people_count.data = (u_int16_t)objects.size();
        }
        img_pub.publish(img_msg.toImageMsg());
        count_pub.publish(people_count);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
