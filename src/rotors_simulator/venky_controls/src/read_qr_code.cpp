#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/Image.h"
#include "../include/console/console.h"
#include <math.h>
#include <sstream>
#include <chrono>
#include <thread>
#include <quirc.h>
#include <array>
#include <stdio.h>

#include "../include/thirdparty/CImg.h"

using namespace std;
using namespace cimg_library;

double tgt_angle = 0;
bool start_flag = false, first_time_flag = true;


CImgDisplay main_disp;
CImg<double> cam_img;
CImg<uint8_t> cam_img_grey;
CImg<double> cam_img_target_bw;

struct quirc *qr;


void cameraCallback(const sensor_msgs::Image& msgIn)
{
  
  int height =  msgIn.height;
  int width = msgIn.width;
  string encoding = msgIn.encoding;
  unsigned int is_bigendian = msgIn.is_bigendian;
  int step = msgIn.step;
  msgIn.data;

  std::vector<int> x_px,y_px;

    if (first_time_flag)
  {
     main_disp.assign(width,height,"Camera_SUAV1", 0);
     cam_img.assign(width,height,1,3,0);
     cam_img_grey.assign(width,height,1,1,0);
     cam_img_target_bw.assign(width,height,1,3,0);
     first_time_flag = false;  
  }
  
  cam_img_target_bw.fill(0);

  cout<<"Height: "<<height<<", Width: "<<width<<endl;
  cout<<"Encoding: "<<encoding<<", is_bigendian: "<<is_bigendian<<endl;
  cout<<"Step: "<<step<<endl;
  std::vector<uint8_t> sample2;

  int counter = 0;
  for (int i = 0; i<height; i++)
  { 
    for(int j = 0; j<width; j++)
    {
      unsigned int R = (unsigned int) msgIn.data[i*step+3*j];
      unsigned int G = (unsigned int) msgIn.data[i*step+3*j+1];
      unsigned int B = (unsigned int) msgIn.data[i*step+3*j+2];
      
      cam_img(j,i,0,0) = R;
      cam_img(j,i,0,1) = G;
      cam_img(j,i,0,2) = B;
      uint8_t temp_ui = (uint8_t)((0.2126 * double(R) + 0.7152 * double(G) + 0.0722 * double(B)));
      // cout<<"Pixel("<<i+1<<","<<j+1<<") -> R:"<<R<<", G:"<<G<<", B:"<<B<<", Grey:"<<(int)(temp_ui)<<endl;
      cam_img_grey(j,i,0,0) = temp_ui; //(temp_ui<120)?0:254;
      //sample[counter] = temp_ui;
      sample2.push_back(temp_ui);
      counter++;
    }
  }

  main_disp.resize();
  main_disp.display(cam_img);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));


  uint8_t *image;
  char a;
  int w, h;
  image = quirc_begin(qr, &w, &h);

  uint8_t *image_end = image + sizeof(image[0])*(w*h-1);
  counter = 0; 

  for (int i = 0; i < w*h; i++)
  {
    image[i] = sample2[i];
  }


  quirc_end(qr);


  int num_codes;
  int i;
  num_codes = quirc_count(qr);
  if(num_codes == 0)
  {
    cout<<"Searching for QR codes... \n";
  }
  else
  {
  cout<<"Successfully detected: "<<num_codes<<" code(s)"<<endl;
  for (i = 0; i < num_codes; i++) {
    struct quirc_code code;
    struct quirc_data data;
    quirc_decode_error_t err;

    quirc_extract(qr, i, &code);

    /* Decoding stage */
    err = quirc_decode(&code, &data);
    if (err)
      printf("DECODE FAILED: %s\n", quirc_strerror(err));
    else
      printf("Data: %s\n", data.payload);
  }

  }


}

int main(int argc, const char** argv)
{
  console::info("Parsing input arguments.");
  string robot_id;
  console::parseArguments(argc,argv, "-robot", robot_id);

  ros::init(argc, (char **) argv, "cimg_display_"+robot_id);

  string img_str = "/";
  img_str += robot_id;
  img_str += "/camera1/image_raw";

  ros::NodeHandle nh;

  ros::Rate loop_rate(30);

  ros::Subscriber sub = nh.subscribe(img_str, 2, cameraCallback);

  qr = quirc_new();
  if (!qr) {
    perror("Failed to allocate memory");
    abort();
  }
  // Change resolution here also.  ((( HARD-CODING )))
  if (quirc_resize(qr, 640, 480) < 0) {
    perror("Failed to allocate video memory");
    abort();
  }

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  quirc_destroy(qr);

  return 0;
}


