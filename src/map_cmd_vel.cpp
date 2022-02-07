#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>

std_msgs::Int16MultiArray pwm_value;

void cmd_vel_callback(const geometry_msgs::Twist &twist_msg) {
  uint16_t ptn = 0;
  pwm_value.data.resize(4);
  int16_t pw[4];

  // /cmd_vel topic [m/s], [rad/s] 
  double x = twist_msg.linear.x;
  double y = twist_msg.linear.y;
  double z = twist_msg.angular.z;
  //printf("x %f y %f z %f\n", x,y,z);

  // distance between wheels [m]
  double dist_left_right = 0.115;
  double dist_front_back = 0.115;
  double d = dist_left_right/2 + dist_front_back/2;

  // wheel radius [m]
  double r = 0.045; 
  
  // amplitude
  double a = 2000;

  // maximum value for /motor_pwm
  int MAX_PW = 32767;

  // compute for each wheel
  double lf = a * ( x - y - d*z ) / r;   // left  front
  double rf = a * ( x + y + d*z ) / r;   // right front
  double lr = a * ( x + y - d*z ) / r;   // left  rear
  double rr = a * ( x - y + d*z ) / r;   // right rear
   
  //printf("lf %+10.0f rf %+10.0f lr %+10.0f rr %+10.0f\n", 
  //lf, rf, rf, rr);

  // rotate inverse direction for left motors
  pw[0] = std::max( -MAX_PW, std::min( MAX_PW, (int)( lf*(-1) ) ));
  pw[2] = std::max( -MAX_PW, std::min( MAX_PW, (int)( rf      ) ));
  pw[1] = std::max( -MAX_PW, std::min( MAX_PW, (int)( lr*(-1) ) ));
  pw[3] = std::max( -MAX_PW, std::min( MAX_PW, (int)( rr      ) ));
  //printf("pw0 %d pw1 %d pw2 %d pw3 %d\n",  pw[0], pw[1], pw[2], pw[3] );

  pwm_value.data[3] = pw[3];
  pwm_value.data[2] = pw[2];
  pwm_value.data[1] = pw[1];
  pwm_value.data[0] = pw[0];
}

int main(int argc, char **argv){
  ros::init(argc, argv, "map_pwm");
  ros::NodeHandle n;

  //publish
  ros::Publisher pub_pwm =
    n.advertise<std_msgs::Int16MultiArray>("motor_pwm",1000);

  //subscriibe
  ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 10, cmd_vel_callback);
  ros::Rate loop_rate(10);

  while (ros::ok()){
    pub_pwm.publish(pwm_value);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
