#ifndef WT61CUART_H
#define WT61CUART_H

#include <vector>
#include <math.h>
#include <time.h>

// #include "ros/ros.h"
// #include "serial/serial.h"
// #include "std_msgs/String.h"
// #include "std_msgs/Empty.h"
// #include "sensor_msgs/Imu.h"
// // #include "geometry_msgs/Twist.h"
// #include "tf/LinearMath/Quaternion.h"

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"


#define PI 3.14159

namespace WTU {

	class Wt61cUart {
	private:
		int baudrate_,index_;
		std::string com_;
		serial::Serial ser;
		
		double g_;
		std::vector<uint8_t> UartData_;
		
		std::string topic_pub_;
		// rclcpp::Publisher wt61c_pub_;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr wt61c_pub_;
		// rclcpp::Publisher wt61c_turtle_;
	public:
		Wt61cUart();
		~Wt61cUart();
		int TranslateAndPub();
		int UartInit();
		int GetAndCheck();
	};

	class custom_Wt61cUart{
	public:
		int baudrate_,index_;
		std::string com_;
		serial::Serial ser;
		
		double g_;
		std::vector<uint8_t> UartData_;
		
		std::string topic_pub_;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr wt61c_pub_;
		// rclcpp::Publisher wt61c_turtle_;
	
		custom_Wt61cUart(int,int,std::string,std::string);
		~custom_Wt61cUart();
		int TranslateAndPub();
		int UartInit();
		int GetAndCheck();
	};
}
#endif