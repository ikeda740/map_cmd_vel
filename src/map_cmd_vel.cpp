#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>

// global variables
std_msgs::Int16MultiArray pwm_value;
int MAX_PW = 32767;     // maximum value of pwm_value (software limit) 
double amplitude;       // velocity amplitude (rosparam)
ros::Time time_updated; // last time cmd_vel_callback() is called

void cmd_vel_callback(const geometry_msgs::Twist &twist_msg) {
  uint16_t ptn = 0;
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
  
  // compute for each wheel
  double lf = amplitude * ( x - y - d*z ) / r;   // left  front
  double rf = amplitude * ( x + y + d*z ) / r;   // right front
  double lr = amplitude * ( x + y - d*z ) / r;   // left  rear
  double rr = amplitude * ( x - y + d*z ) / r;   // right rear
  
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
  
  time_updated = ros::Time::now();
}

void reset_pwm(){
  pwm_value.data[0] = 0;
  pwm_value.data[1] = 0;
  pwm_value.data[2] = 0;
  pwm_value.data[3] = 0;
}

int main(int argc, char **argv){
ros::init(argc, argv, "map_cmd_vel");
ros::NodeHandle nh;
ros::NodeHandle pnh("~");
  
  //parameter
  pnh.param("amplitude", amplitude, 1500.0);
  ROS_INFO("[%s] param:%.1f", ros::this_node::getName().c_str(), amplitude);
  
  //publish
    ros::Publisher pub_pwm =
  nh.advertise<std_msgs::Int16MultiArray>("motor_pwm",1000);
  
  //subscriibe
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 10, cmd_vel_callback);
ros::Rate loop_rate(10);

  // init
    time_updated = ros::Time::now();
  pwm_value.data.resize(4);
  reset_pwm();
  
while (ros::ok()){
ros::Time time_now = ros::Time::now();
    
    // if /cmd_vel seems to have stopped, reset pwm_value
    if ( time_now - time_updated > ros::Duration(1.0) ){
      reset_pwm();
    }
    pub_pwm.publish(pwm_value);
    
ros::spinOnce();
    loop_rate.sleep();
}
  return 0;
}
