#include <arfuros/PromptActionAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<arfuros::PromptActionAction> Server;

void execute(const arfuros::PromptActionGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prompt_server");
  ros::NodeHandle n;
  Server server(n, "prompt", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
