#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#define PI 3.1415956

unsigned char r_buffer[20];//接收缓存

serial::Serial ser;

ros::Publisher odom_enc_pub;

// drive dependent parameters
float odom_encoder_coef_;
float odom_axle_track_;
float odom_angular_coef_;
float odom_traction_factor_;
float odom_covariance_0_ = 0.01;       //covariance_0 协方差0
float odom_covariance_35_ = 0.00;      //covariance_35 协方差35

uint8_t controller_freq = 10;
uint8_t serial_buffer[35];


const float ODOM_AXLE_TRACK_2WD = 14.375;
const float ODOM_ANGULAR_COEF_2WD = 1.0 / (ODOM_AXLE_TRACK_2WD * 2.54 / 100);  // rad per meter
const float ODOM_TRACTION_FACTOR_2WD = 0.610;      //牵引系数

const double kp = 1.1;

double Pulse2mms(int16_t pulse_vel)
{
  return pulse_vel / 100.0* PI * 67.33 / 0.1;
}


void publishOdometry(int16_t left, int16_t right)
{  // convert encoder readings to real world values and publish as Odometry

  static double left_dist = 0;
  static double right_dist = 0;
  static double pos_x = 0;
  static double pos_y = 0;
  static double theta = 0;
  static double past_time = 0;
  double net_vel = 0;
  double diff_vel = 0;
  double alpha = 0;
  double dt = 0;
  tf::Quaternion q_new;
  double left_vel = Pulse2mms(-left)/1000.0*kp;
  //double left_vel = left/100.0*1.0;
  // double right_vel = right/100.0*1.0;
  double right_vel = Pulse2mms(-right)/1000.0*kp;
  
  
  ros::Time ros_now_time = ros::Time::now();
  double now_time = ros_now_time.toSec();

  nav_msgs::Odometry odom_msg;
  
  dt = now_time - past_time;
  past_time = now_time;

  if (past_time != 0)
    {
      left_dist += left_vel * dt;
      right_dist += right_vel * dt;

      net_vel = 0.5 * (left_vel + right_vel);
      diff_vel = right_vel - left_vel;

      alpha = odom_angular_coef_ * diff_vel * odom_traction_factor_;

      pos_x = pos_x + net_vel * cos(theta) * dt;
      pos_y = pos_y + net_vel * sin(theta) * dt;
      theta = (theta + alpha * dt);

      q_new = tf::createQuaternionFromRPY(0, 0, theta);
      quaternionTFToMsg(q_new, odom_msg.pose.pose.orientation);
    }

  odom_msg.header.stamp = ros_now_time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";


  odom_msg.twist.twist.linear.x = net_vel;
  odom_msg.twist.twist.angular.z = alpha;

  // If not moving, trust the encoders completely
  // otherwise set them to the ROS param
  // if (net_vel == 0 && alpha == 0)
  //   {
  //     odom_msg.twist.covariance[0] = odom_covariance_0_ / 1e3;
  //     odom_msg.twist.covariance[7] = odom_covariance_0_ / 1e3;
  //     odom_msg.twist.covariance[35] = odom_covariance_35_ / 1e6;
  //   }
  // else
  //   {
  //     odom_msg.twist.covariance[0] = odom_covariance_0_;
  //     odom_msg.twist.covariance[7] = odom_covariance_0_;
  //     odom_msg.twist.covariance[35] = odom_covariance_35_;
  //   }

  odom_msg.pose.pose.position.x = pos_x;
  odom_msg.pose.pose.position.y = pos_y;

  odom_enc_pub.publish(odom_msg);
  return;
}

uint8_t car_driver_data[7];

void send_cmd(uint16_t motor_pwm,uint16_t servo_pwm)
{
  car_driver_data[0] = 0xAA;          //发送帧头
  car_driver_data[1] = motor_pwm;     //发送电机pwm
  motor_pwm >>= 8;
  car_driver_data[2] = motor_pwm;     

  car_driver_data[3] = servo_pwm;     //发送舵机pwm
  servo_pwm >>= 8;
  car_driver_data[4] = servo_pwm;

  uint8_t check = 0;        //发送校验和
  for(uint8_t i=1;i<=4;i++)
    {
      check += car_driver_data[i];
    }
  car_driver_data[5] = check;
  car_driver_data[6] = 0x55;        //发送帧尾

  ser.write(car_driver_data,sizeof(car_driver_data));
}

void TwistCallback(const geometry_msgs::Twist& twist)
{
  double angle;
  //ROS_INFO("x= %f", twist.linear.x);
  //ROS_INFO("z= %f", twist.angular.z);
  angle = 2500.0 - twist.angular.z * 2000.0 / 180.0;
  //ROS_INFO("angle= %d",uint16_t(angle));
  send_cmd(uint16_t(twist.linear.x),uint16_t(angle));
}

uint8_t one_byte_callBack(uint8_t rx_dat)
{
  static uint8_t i = 0, rebuf[20];

  rebuf[i] = rx_dat;

  if(rebuf[0] != 0xAA)
    {
      i = 0;
      return 2;
    }

  if(i >= 6)
    {
      if(rebuf[6] == 0x55)
	{
	  uint8_t check = 0;
	  for(int j=1;j<=4;j++)
	    {
	      check += rebuf[j];
	    }
	  
	  if(check != rebuf[5])
	    {
	      i = 0;
	      return 0;
	    }
	  else
	    {
	      int16_t right = (int16_t)(rebuf[2]<<8 | rebuf[1]);
	      int16_t left = (int16_t)(rebuf[4]<<8 | rebuf[3]);
	      publishOdometry(right,left);
	      ROS_INFO("----------------");
	      ROS_INFO("right %d",right);
	      ROS_INFO("left %d",left);
	      i = 0;
	      return 1;
	    }
	  
	}
      i = 0;
    }
  i++;
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "car_driver");
  ros::NodeHandle nh;

  odom_enc_pub = nh.advertise<nav_msgs::Odometry>("odom_encoder", 1);
  ros::Subscriber sub = nh.subscribe("/car/cmd_vel",1,TwistCallback);
  // timer1 = nh_.createTimer(ros::Duration((1.0)/), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    
  odom_angular_coef_ = ODOM_ANGULAR_COEF_2WD;
  odom_traction_factor_ = ODOM_TRACTION_FACTOR_2WD;
    
    try
      {
        ser.setPort("/dev/car");
        ser.setBaudrate(115200);
	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
      }
    catch (serial::IOException& e)
      {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
      }

    if(ser.isOpen()){
      ROS_INFO_STREAM("Serial Port initialized");
    }else{
      return -1;
    }

    //ros::Rate loop_rate(1000);
    uint8_t rx_byte;

    while(ros::ok())
      {

        if(ser.available())
	  {
            //ROS_INFO_STREAM("Reading from serial port");
	    ser.read(&rx_byte,1);
            if(one_byte_callBack(rx_byte) == 0)
	      {
                ROS_WARN("encoder data analysis failed, data has losed");
	      }
	  }
	ros::spinOnce();
        //loop_rate.sleep();
      }
}
