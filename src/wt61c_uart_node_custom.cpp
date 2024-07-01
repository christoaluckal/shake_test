
// #include "ros/ros.h"
// #include "wt61c_uart.h"
// #include "serial/serial.h"
// #include <time.h>
// #include <vector>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <time.h>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

#define PI 3.14159


class custom_Wt61cUart : public rclcpp::Node
{
	public:
		int baudrate_,index_;
		std::string com_;
		serial::Serial ser;
		
		double g_;
		std::vector<uint8_t> UartData_;
		
		std::string topic_pub_;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr wt61c_pub_;
		// rclcpp::Publisher wt61c_turtle_;
	
		// custom_Wt61cUart(int,int,std::string,std::string);
		custom_Wt61cUart(int bd, int index, std::string com, std::string topic_pub) : Node("wt61c_uart_node_custom")
		{
			this->baudrate_ = bd;
			this->index_ = index;
			this->com_ = com;
			this->topic_pub_ = topic_pub;

			this->wt61c_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(this->topic_pub_, 1);
		}
		~custom_Wt61cUart()
		{
			ser.close();
			std::cout<< "The Uart port has been closed."<<std::endl;
		}
		int UartInit()
		{
			try
			{
				ser.setPort(com_);
			
				ser.setBaudrate(baudrate_);
				serial::Timeout to = serial::Timeout::simpleTimeout(1000);
				ser.setTimeout(to);
				ser.open();
			}
			catch(serial::IOException& e)
			{
				std::cout << "Unable to open port. Please try again." << std::endl;
				return 1;
			}
			if(ser.isOpen())
			{
				std::cout << "The port initialize succeed." << std::endl;
				ser.flushInput();
				sleep(0.1);
				return 0;
			}
			else
				return 1;
		}
		int GetAndCheck()
		{
			int i,j;
			int sum = 0x55;

			while(UartData_.size()-index_<33) {
				while(ser.available()<33){
					// ROS_INFO("wait");
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "wait");
					}
				ser.read(UartData_,ser.available());
			}
			while(true) {
				if(UartData_[index_] ==0x55 && UartData_[index_+1] ==0x51) {
					//SRC check
					for (i= 1; i<10; i++)
						sum+= UartData_[index_+i];
					if(UartData_[index_+10] == sum%0x100)
						j = 1;
					sum = 0x55;
					for (i= 12; i<21; i++)
						sum+= UartData_[index_+i];
					if(UartData_[index_+21] == sum%0x100)
						j++;
					sum = 0x55;
					for (i= 23; i<32; i++)
						sum+= UartData_[index_+i];
					if(UartData_[index_+32] == sum%0x100)
						j++;
					if (j = 3) {
						// ROS_INFO("Yes,I got a complete package.");
						std::cout << "Yes,I got a complete package." << std::endl;
						return 0;
					}
					else {
						sum = 0x55;
						index_++;
					}
				}
				else
					index_++;
				while(UartData_.size()-index_-32<33) {
					while(ser.available()<33) {}
					ser.read(UartData_,ser.available());
				}
			}	
		}

		int TranslateAndPub()
		{
			sensor_msgs::msg::Imu wt61c_imu;
			double linear_acceleration[2],angular_velocity[2],orientation[2];

			wt61c_imu.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
			wt61c_imu.header.frame_id = "wt61c_uart";
			tf2::Quaternion quate;

			linear_acceleration[0] = ( short (UartData_[index_+ 3]<< 8 | UartData_[index_+ 2]))/ 32768.0* 16.0* 9.8;
			linear_acceleration[1] = ( short (UartData_[index_+ 5]<< 8 | UartData_[index_+ 4]))/ 32768.0* 16.0* 9.8;
			linear_acceleration[2] = ( short (UartData_[index_+ 7]<< 8 | UartData_[index_+ 6]))/ 32768.0* 16.0* 9.8;
			wt61c_imu.linear_acceleration.x = linear_acceleration[0];
			wt61c_imu.linear_acceleration.y = linear_acceleration[1];
			wt61c_imu.linear_acceleration.z = linear_acceleration[2];

			angular_velocity[0] = (short (UartData_[index_+ 14]<< 8 | UartData_[index_+ 13]))/ 32768.0* 2000* PI/180;
			angular_velocity[1] = (short (UartData_[index_+ 16]<< 8 | UartData_[index_+ 15]))/ 32768.0* 2000* PI/180;
			angular_velocity[2] = (short (UartData_[index_+ 18]<< 8 | UartData_[index_+ 17]))/ 32768.0* 2000* PI/180;
			wt61c_imu.angular_velocity.x = angular_velocity[0];
			wt61c_imu.angular_velocity.y = angular_velocity[1];
			wt61c_imu.angular_velocity.z = angular_velocity[2];
			
			orientation[0] = (short(UartData_[index_+ 25]<< 8 | UartData_[index_+ 24]))/ 32768.0* PI;
			orientation[1] = (short(UartData_[index_+ 27]<< 8 | UartData_[index_+ 26]))/ 32768.0* PI;
			orientation[2] = (short(UartData_[index_+ 29]<< 8 | UartData_[index_+ 28]))/ 32768.0* PI;

			quate.setRPY(orientation[0], orientation[1], orientation[2]);
			wt61c_imu.orientation.x = quate[0];
			wt61c_imu.orientation.y = quate[1];
			wt61c_imu.orientation.z = quate[2];
			wt61c_imu.orientation.w = quate[3];

			// wt61c_pub_.publish(wt61c_imu);
			wt61c_pub_->publish(wt61c_imu);
			
			//delate the old date	
			index_ =index_+ 32;
			UartData_.erase(UartData_.begin(),UartData_.begin()+index_);
			index_ = 0;
			// ROS_INFO("The data has been pub.");	
			std::cout << "The data has been pub." << std::endl;
			// wt61c_turtle_.publish(turtle);
		}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	int baudrate = 115200;

    custom_Wt61cUart wt61cuart_0(baudrate,0,"/dev/ttyUSB0","/imu0");


    while(wt61cuart_0.UartInit()){
	sleep(1);
	}      //declare the uart port

	// while(ros::ok()){

    // wt61cuart_0.GetAndCheck();
	// wt61cuart_0.TranslateAndPub();
	// }
	// return 0;

	while(rclcpp::ok()){

	wt61cuart_0.GetAndCheck();
	wt61cuart_0.TranslateAndPub();
	}
	return 0;
}
