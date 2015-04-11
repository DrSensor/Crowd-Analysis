#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>




#include <curl/curl.h>

//function to retrieve the image as Cv::Mat data type
cv::Mat curlImg(const char *url);
double fps=30;

std::string url_param, filename_param;
int device_param=-1;
int width_param=640, height_param=480;

bool isSnapshot=false;
bool isCamOpened=false;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_bridge");
    ros::NodeHandle node;

    isSnapshot = ros::param::get("url", url_param);
    ros::param::param<int>("device", device_param, -1);
    ros::param::param<int>("width", width_param, 640);
    ros::param::param<int>("height", height_param, 480);
    ros::param::param<double>("fps", fps, 28);

//     argc--; argv++;
//     while( argc && *argv[0] == '-' )
//     {
//         if( !strcmp(*argv, "-url") && ( argc > 1 ) )
//         {
//             ROS_INFO("Opening ");
//             isSnapshot=true;
//             url_image = *(argv+1);
//             printf("Using image sequence form %s\n",url_image.c_str());
//             argc--; argv++;
//         }
//         else if( !strcmp(*argv, "-device") && ( argc > 1 ) )
//         {
//             ROS_INFO("Opening ");
//             isSnapshot=false;
// //            topic_name.append(*(argv+1));
//             device = atoi(*(argv+1));
//             printf("Using image sequence form %s\n",*(argv+1));
//             argc--; argv++;
//         }
//         if( !strcmp(*argv, "-fps") && ( argc > 1 ) )
//         {
//             fps = atof(*(argv+1));
//             printf("With %ffps\n",fps);
//             argc--; argv++;
//         }
//         argc--; argv++;
//     }

    ros::Publisher image_pub = node.advertise<sensor_msgs::Image>("camera", 100);
    ros::Rate loop_rate(fps);

    cv_bridge::CvImage img_msg;
    cv::VideoCapture cam;

    img_msg.encoding = "bgr8";

    if (!isSnapshot) {
        if (ros::param::get("filename", filename_param)) 
            isCamOpened = cam.open(filename_param);
        else
            isCamOpened = cam.open(device_param);
        cam.set(CV_CAP_PROP_FPS, fps*1.3);
        if(isCamOpened) {
            cam.set(CV_CAP_PROP_FRAME_HEIGHT, height_param);
            cam.set(CV_CAP_PROP_FRAME_WIDTH, width_param);
        }
    }

    if (isCamOpened)
        ROS_INFO("Opening /dev/video%d %ffps %dx%d", device_param, fps,
                 (int)cam.get(CV_CAP_PROP_FRAME_WIDTH), (int)cam.get(CV_CAP_PROP_FRAME_HEIGHT));
    else if ( ! isSnapshot) ROS_ERROR("Unknown error");

    while (ros::ok())
    {
        if (isCamOpened) cam >> img_msg.image;
        else {
            if (!isSnapshot) ROS_ERROR("it's not snapshot");
            img_msg.image = curlImg(url_param.c_str());
        }

        image_pub.publish(img_msg.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

//curl writefunction to be passed as a parameter
size_t write_data(char *ptr, size_t size, size_t nmemb, void *userdata) {
    std::ostringstream *stream = (std::ostringstream*)userdata;
    size_t count = size * nmemb;
    stream->write(ptr, count);
    return count;
}

cv::Mat curlImg(const char* url)
{
    CURL *curl;
    CURLcode res;
    std::ostringstream stream;
    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, url); //the JPEG Frame url
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data); // pass the writefunction
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &stream); // pass the stream ptr when the writefunction is called
    res = curl_easy_perform(curl); // start curl
    std::string output = stream.str(); // convert the stream into a string
    curl_easy_cleanup(curl); // cleanup
    std::vector<char> data = std::vector<char>( output.begin(), output.end() ); //convert string into a vector
    cv::Mat data_mat = cv::Mat(data); // create the cv::Mat datatype from the vector
    cv::Mat image = cv::imdecode(data_mat,1); //read an image from memory buffer
    return image;
}
