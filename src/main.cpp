#include "aliengo.hpp"
#include "io.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "aliengo_policy");
  std::string model_path;
  ros::param::get("model_path", model_path);
  print("Loading Model", model_path);
  AlienGo robot(model_path);
  print("Finished Loading");
  robot.initComm();
  ros::spin();
}