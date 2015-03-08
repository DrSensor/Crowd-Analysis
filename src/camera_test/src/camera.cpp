#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt16.h>
#include <opencv2/highgui/highgui.hpp>

#include <AdaptiveBackgroundLearning.h>

#include <curl/curl.h>

#define LOOP_RATE_ARG   1
#define CAMERA_ARG      2

//function to retrieve the image as Cv::Mat data type
cv::Mat curlImg(const char *url);
IBGS *bgs;
cv::CascadeClassifier classifier;
std::string emotion_dataset;
int max_detected = -1;
bool isSnapshot=false;
bool isCamOpened=false;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_test");
    ros::NodeHandle node;

    std::string topic_name = "crowd/image";
    std::string file_dataset = "haarcascade_upperbody.xml";
    std::string url_image;

    argc--; argv++;
    while( argc && *argv[0] == '-' )
    {
        if( !strcmp(*argv, "-url") && ( argc > 1 ) )
        {
            ROS_INFO("Opening ");
            isSnapshot=true;
            url_image = *(argv+1);
            printf("Using image sequence form %s\n",url_image.c_str());
            argc--; argv++;
        }
        else if( !strcmp(*argv, "-device") && ( argc > 1 ) )
        {
            ROS_INFO("Opening ");
            isSnapshot=false;
            topic_name.append(*(argv+1));
            printf("Using image sequence form %s\n",*(argv+1));
            argc--; argv++;
        }
        if( !strcmp(*argv, "-crowd_dataset") && ( argc > 1 ) )
        {
            file_dataset = *(argv+1);
            printf("Using dataset form %s\n",file_dataset.c_str());
            argc--; argv++;
        }
        if( !strcmp(*argv, "-emotion_dataset") && ( argc > 1 ) )
        {
            file_dataset = *(argv+1);
            printf("Using dataset form %s\n",emotion_dataset.c_str());
            argc--; argv++;
        }
        argc--; argv++;
    }

    ros::Publisher image_pub = node.advertise<sensor_msgs::Image>
            (topic_name, 1000);
    ros::Publisher count_pub = node.advertise<std_msgs::UInt16>
            ("crowd/people_count", 1000);
    ros::Rate loop_rate(1000/atof(argv[LOOP_RATE_ARG]));

    cv_bridge::CvImage img_msg;
    std_msgs::UInt16 people_count;

    cv::VideoCapture cam;
    cv::Mat img_fg, img_bg;
    std::vector<cv::Rect> objects;
    std::vector<cv::Scalar> colors;
    cv::RNG rng(12345);
    bgs = new AdaptiveBackgroundLearning;

    img_msg.encoding = "bgr8";

    if ( ! classifier.load(file_dataset))
        ROS_ERROR("Can't open %s", file_dataset.c_str());

    if (!isSnapshot) {
        if(isdigit(argv[CAMERA_ARG][0])) isCamOpened = cam.open(atoi(argv[CAMERA_ARG]));
        else isCamOpened = cam.open(argv[CAMERA_ARG]);
        cam.set(CV_CAP_PROP_FPS, atof(argv[LOOP_RATE_ARG]));
        if(isCamOpened) {
            cam.set(CV_CAP_PROP_FRAME_HEIGHT, atoi(argv[3]));
            cam.set(CV_CAP_PROP_FRAME_WIDTH, atoi(argv[4]));
        }
    }

    if (isCamOpened)
        ROS_INFO("Opening /dev/%s %dfps %dx%d",
                 topic_name.c_str(), (int)cam.get(CV_CAP_PROP_FRAME_WIDTH), (int)cam.get(CV_CAP_PROP_FRAME_HEIGHT),
                 atoi(argv[LOOP_RATE_ARG])
                 );
    else ROS_ERROR("Can't open /dev/%s", topic_name.c_str());

    while (ros::ok())
    {
        if (isCamOpened) cam >> img_msg.image;
        else img_msg.image = curlImg(url_image.c_str());

//        bgs->process(img_msg.image, img_fg, img_bg);
//        cv::Mat element = cv::getStructuringElement( 2, cv::Size( 3, 3 ), cv::Point( 1, 1 ) );
//        cv::morphologyEx( img_fg, img_fg, CV_MOP_OPEN, element );
//        cv::cvtColor(img_fg,img_fg,CV_GRAY2BGR);
//        img_msg.image &= img_fg;

        classifier.detectMultiScale(img_msg.image, objects, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
        for (int j = 0; j < objects.size(); ++j) {
            if ( j > max_detected) {
                colors.push_back(cv::Scalar(rng.uniform(0,255),rng.uniform(0, 255),rng.uniform(0, 255)));
                max_detected = j;
            }
            cv::rectangle(img_msg.image, objects[j], colors[j], 2);
        }
        people_count.data = (u_int16_t)objects.size();
        image_pub.publish(img_msg.toImageMsg());
        count_pub.publish(people_count);
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
