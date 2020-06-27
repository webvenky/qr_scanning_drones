#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "../include/console/console.h"
#include <math.h>
#include <chrono>
#include <thread>
#include <sstream>
using namespace std;

ros::Publisher *cam_pitch_pub, *cam_roll_pub;;

double eva_pos_x = 0, eva_pos_y = 0, eva_pos_z = 0, suav_altitude;
bool start_flag = false;
double max_roll = 2.007;  // HARD_CODING : Set this value (in rad) for maximum roll angle
// string alt_str;

// void evaCallback(const geometry_msgs::Point& msgIn)
// {
//   // cout<<"============= UPDATING TARGET POSE ================"<<endl;
//   std_msgs::Float64 msgOut;
//   eva_pos_x = msgIn.x;
//   eva_pos_y = msgIn.y;
//   eva_pos_z = msgIn.z;
//   start_flag = true;
//   // cout<<"EVA_X: "<<eva_pos_x<<" \t EVA_Y: "<<eva_pos_y<<" \t EVA_Z: "<<eva_pos_z<<endl; 
// }


void cameraCallback(const nav_msgs::Odometry& msgIn)
{


    std_msgs::Float64 msgOutPitch, msgOutRoll;
    double pos_x = msgIn.pose.pose.position.x;
    double pos_y = msgIn.pose.pose.position.y;
    double pos_z = msgIn.pose.pose.position.z;
    double quat_x = msgIn.pose.pose.orientation.x;
    double quat_y = msgIn.pose.pose.orientation.y;
    double quat_z = msgIn.pose.pose.orientation.z;
    double quat_w = msgIn.pose.pose.orientation.w;
    
    double angle = atan2(2*(quat_w*quat_z+quat_x*quat_y),(pow(quat_w,2)+pow(quat_x,2)-pow(quat_y,2)-pow(quat_z,2)));
    double pitch = (-asin(2*(quat_x*quat_z-quat_w*quat_y)));
    double roll =  atan2(2*(quat_w*quat_x+quat_z*quat_y),(pow(quat_w,2)-pow(quat_x,2)-pow(quat_y,2)+pow(quat_z,2)));

    cout<<"Roll: "<<roll<<"Pitch: "<<pitch<<"Yaw: "<<angle<<endl;


    double roll_compensation = -roll;
    double pitch_compensation = pitch;

    // // cout<<"quat_x="<<quat_x<<"quat_y="<<quat_y<<endl;

    // double diff_x = pos_x + cos(angle) * 100;  // eva_pos_x -pos_x;
    // double diff_y = pos_y + sin(angle) * 100;  // eva_pos_y -pos_y;
    // double diff_z = pos_z + 0;  // eva_pos_z -pos_z;

    // double new_x = (pow(quat_w,2) +pow(quat_x,2) -pow(quat_y,2) -pow(quat_z,2))*diff_x +
    //                (2*quat_x*quat_y + 2*quat_w*quat_z)*diff_y +
    //                (2*quat_x*quat_z - 2*quat_w*quat_y)* diff_z;
    // double new_y = (2*quat_x*quat_y - 2*quat_w*quat_z)*diff_x +
    //                (pow(quat_w,2) -pow(quat_x,2) +pow(quat_y,2) -pow(quat_z,2))*diff_y +
    //                (2*quat_z*quat_y + 2*quat_w*quat_x)*diff_z;
    // double new_z = (2*quat_x*quat_z + 2*quat_w*quat_y)*diff_x +
    //                (2*quat_z*quat_y - 2*quat_w*quat_x)*diff_y +
    //                (pow(quat_w,2) -pow(quat_x,2) -pow(quat_y,2) +pow(quat_z,2))*diff_z;

    // double mag_vector = sqrt( pow(new_x,2) + pow(new_y,2) );
    // double gradient_angle_x = atan2( (new_y) ,  new_x );  //PAN
    // double gradient_angle_y = atan2(   mag_vector ,  -(new_z) );   //TILT

    // if((gradient_angle_y)>=0)
    //   gradient_angle_y = min(max_tilt, gradient_angle_y);   
    // else
    //   gradient_angle_y = max(-max_tilt, gradient_angle_y);




    msgOutRoll.data = roll_compensation;
    // ROS_INFO_STREAM("Sending camera TILT set-point =>"<<msgOutTilt.data);
    cam_roll_pub->publish(msgOutRoll);

    // std::this_thread::sleep_for(std::chrono::seconds(3));
    msgOutPitch.data = pitch_compensation;
    // ROS_INFO_STREAM("Sending camera PAN set-point =>"<<msgOutPan.data);
    cam_pitch_pub->publish(msgOutPitch);


}



int main(int argc, const char** argv)
{

  console::info("Parsing input arguments for camera follower node.");
  string robot_name, target_name;
  console::parseArguments(argc,argv, "-robot", robot_name);
  // console::parseArguments(argc,argv, "-target", target_name);

  //string robot_id(argv[1]);

  ros::init(argc, (char **) argv, "cam_hold_horizontal_"+robot_name);

  string camera_pitch_str = "/";
  camera_pitch_str += robot_name;
  camera_pitch_str += "/camera_pitch_controller/command";

  string camera_roll_str = "/";
  camera_roll_str += robot_name;
  camera_roll_str += "/camera_roll_controller/command";

  string pose_str = "/";
  pose_str += robot_name;
  pose_str += "/ground_truth/odometry";

  // string eva_pose_str = "/";
  // eva_pose_str += robot_name;
  // eva_pose_str += "/cam_target";

  ros::NodeHandle n;

  cam_pitch_pub = new ros::Publisher(n.advertise<std_msgs::Float64>(camera_pitch_str, 1000));
  cam_roll_pub = new ros::Publisher(n.advertise<std_msgs::Float64>(camera_roll_str, 1000));

  ros::Rate loop_rate(100);

  ros::Subscriber sub = n.subscribe(pose_str, 100, cameraCallback);

  // ros::Subscriber sub_eva_pose = n.subscribe(eva_pose_str, 10, evaCallback);

  console::info("Camera follower node established.");

  // ros::spin();
  
  while (ros::ok())
  {

    ros::spinOnce();
  
    loop_rate.sleep();
    
  }

  delete cam_pitch_pub, cam_roll_pub  ;
  return 0;
}