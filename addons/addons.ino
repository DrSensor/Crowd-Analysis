#include <ros.h>
#include <std_msgs/Float32.h>


//Set up the ros node and publisher
std_msgs::Float32 sound_msg, gas_msg;
ros::Publisher pub_sound("crowd/sound", &sound_msg);
ros::Publisher pub_gas("crowd/CO", &gas_msg);
ros::NodeHandle nh;

const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

float Ro = 10000.0;    // this has to be tuned 10K Ohm
int ledPin = 13;    // select the pin for the LED
int val = 0;        // variable to store the value coming from the sensor
float Vrl = 0.0;
float Rs = 0.0;
float ratio = 0.0;

void setup()
{
    nh.initNode();
    nh.advertise(pub_sound);
    nh.advertise(pub_gas);
}

long publisher_timer=0;

void loop()
{
    unsigned long startMillis= millis(); // Start of sample window
    unsigned int peakToPeak = 0; // peak-to-peak level

    unsigned int signalMax = 0;
    unsigned int signalMin = 1024;

    // collect data for 50 mS
    while (millis() - startMillis < sampleWindow)
    {
        sample = analogRead(A1);
        if (sample < 1024) // toss out spurious readings
        {
            if (sample > signalMax)
            {
                signalMax = sample; // save just the max levels
            }
            else if (sample < signalMin)
            {
                signalMin = sample; // save just the min levels
            }
        }
    }
    peakToPeak = signalMax - signalMin; // max - min = peak-peak amplitude
    double volts = (peakToPeak * 3.3) / 1024; // convert to volts

    val = analogRead(A0);     // read the value from the analog sensor
    Vrl = val * ( 5.00 / 1024.0  );      // V
    Rs = 20000 * ( 5.00 - Vrl) / Vrl ;   // Ohm
    ratio =  Rs/Ro;

    sound_msg.data = (float)volts;
    gas_msg.data = (float)get_CO (ratio);
    pub_sound.publish(&sound_msg);
    pub_gas.publish(&gas_msg);

    nh.spinOnce();
}

// get CO ppm
float get_CO (float ratio){
    float ppm = 0.0;
    ppm = 37143 * pow (ratio, -3.178);
    return ppm;
}

