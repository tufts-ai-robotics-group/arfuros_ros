/*****************************
Blinker.cpp
ARFUROS
Amel Hassan
AIR Lab
Tufts University
*****************************/

// This script advertises a message with paramters to control 
// blinker script in Unity 

#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <arfuros/BlinkerMsg.h>

#include <stdint.h>
#include <sstream>
#include <string>

// Parameters for message 
const std::string PATH = "fullpath";
<<<<<<< HEAD
const float DEV_THRES = 0.2f;
const int NUMPOINTS = 2;
=======
const float DEV_THRES = 0.1f;
const int NUMPOINTS = 4;
>>>>>>> c4072977da5957ec79201e873775d22b3815002c
const int OUTLOOK = 10;

const std::string TOPIC_OUT = "/ARFUROS/Blinker";
const int         FREQUENCY = 1;


ros::Publisher relativePub;

void publishMsg(){

    arfuros::BlinkerMsg message;
    message.path = PATH;
    message.numpoints_thresh = NUMPOINTS;
    message.outlook_thresh = OUTLOOK;
    message.deviation_thresh = DEV_THRES;

    relativePub.publish(message);
}

int main (int argc, char **argv){
    ros::init(argc, argv, "blinker_arfuros");
    ros::NodeHandle n;
    
    relativePub = n.advertise<arfuros::BlinkerMsg>(TOPIC_OUT, 5);
    
    ros::Rate rate(FREQUENCY);
    while(ros::ok()){
        ros::spinOnce();
        publishMsg();
        rate.sleep();   
    }
    
    return 0;
}