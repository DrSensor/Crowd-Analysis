#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <curl/curl.h>

//function to retrieve the image as Cv::Mat data type
cv::Mat curlImg(const char *url);

std::string url_param, filename_param;
int device_param=-1;
int width_param, height_param;
double fps;
int playback_limit, playback_counter=0;

bool isSnapshot=false;
bool isCamOpened=false;
int frame_count=0;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_bridge");
    ros::NodeHandle node;

    isSnapshot = node.getParam(ros::this_node::getName() + "/url", url_param);
    // node.param<int>("device", device_param, -1);
    node.param<int>(ros::this_node::getName() + "/width", width_param, 640);
    node.param<int>(ros::this_node::getName() + "/height", height_param, 480);
    node.param<double>(ros::this_node::getName() + "/fps", fps, 28);
    node.param<int>(ros::this_node::getName() + "/playback_limit", playback_limit, 1);

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
    cv::VideoCapture capture;

    img_msg.encoding = "bgr8";

    if (!isSnapshot) {
        if (node.getParam(ros::this_node::getName() + "/filename", filename_param)) 
            isCamOpened = capture.open(filename_param);
        else if (node.getParam(ros::this_node::getName() + "/device", device_param)) {
            isCamOpened = capture.open(device_param);
            playback_limit = false;
        }
        else {
            ROS_ERROR("camera_bridge fail!!, using default /dev/video%d", device_param);
            isCamOpened = capture.open(device_param);
            playback_limit = false;
        }
        capture.set(CV_CAP_PROP_FPS, fps*1.3);
        if(isCamOpened) {
            capture.set(CV_CAP_PROP_FRAME_HEIGHT, height_param);
            capture.set(CV_CAP_PROP_FRAME_WIDTH, width_param);
        }
    }

    if (isCamOpened)
        ROS_INFO("Opening /dev/video%d %ffps %dx%d", device_param, fps,
                 (int)capture.get(CV_CAP_PROP_FRAME_WIDTH), (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    else if ( ! isSnapshot) ROS_ERROR("Unknown error");
    else ROS_INFO("Opening stream %s", url_param.c_str());

    while (ros::ok())
    {
        if (isCamOpened) {
            capture >> img_msg.image;
            frame_count++;
            if ((frame_count == capture.get(CV_CAP_PROP_FRAME_COUNT))) {
                frame_count = 0;
                playback_counter++;
                if (playback_limit != playback_counter) 
                    capture.set(CV_CAP_PROP_POS_FRAMES, 0);
            }
        }
        else {
            if (!isSnapshot) ROS_ERROR("it's not snapshot url");
            img_msg.image = curlImg(url_param.c_str());
        }

        // if (!img_msg.image.empty()) 
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
