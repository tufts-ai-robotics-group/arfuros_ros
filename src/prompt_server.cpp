#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

#include <arfuros/PromptActionAction.h>
#include <arfuros/Prompt.h>
#include <arfuros/PromptFeedback.h>

#include <sstream>
#include <string>

typedef actionlib::SimpleActionServer<arfuros::PromptActionAction> Server;

const std::string PROMPT_OUT = "/ARFUROS/Prompt/PromptRequest";
const std::string PROMPT_IN = "/ARFUROS/Prompt/PromptFeedback";

ros::Publisher promptPub; 

Server *promptServer;
bool requestUnderway = false;
int id_counter = 0;

void execute(const arfuros::PromptActionGoalConstPtr& goal, Server* as)
{
    // Create the Request
    arfuros::Prompt request;
    request.prompt_id = id_counter;
    request.type = goal->prompt_type;
    request.text = goal->prompt_text;
    request.options = goal->options;
    request.display_duration = goal->max_wait_time;

    //Send the Request
    promptPub.publish(request);

    //Regulate Feedback/Success/Failure
    promptServer = as;
    requestUnderway = true;

    id_counter++;
}

void feedbackCallback(const arfuros::PromptFeedback::ConstPtr& inMsg){
    if(!requestUnderway){
        return;
    }

    if(inMsg->waiting){
        arfuros::PromptActionFeedback fb;
        fb.device_waiting = true;
        promptServer->publishFeedback(fb);
    }
    else{
        arfuros::PromptActionResult rp;
        rp.timeout = inMsg->timeout;
        rp.selected_option = inMsg->selected_option;
        promptServer->setSucceeded(rp);
        requestUnderway = false;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "prompt_server");
    ros::NodeHandle n;
    Server server(n, "prompt", boost::bind(&execute, _1, &server), false);
    server.start();

    promptPub = n.advertise<arfuros::Prompt>(PROMPT_OUT, 5);
    ros::Subscriber feedbackSub  = n.subscribe(PROMPT_IN, 5, feedbackCallback);

    ros::spin();
    return 0;
}
