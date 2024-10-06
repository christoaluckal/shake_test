

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <time.h>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "wt61c_uart.h"

#define PI 3.14159


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	int baudrate = 115200;

    WTU::custom_Wt61cUart wt61cuart_0(baudrate,0,"/dev/ttyUSB0","/imu0");
	

    while(wt61cuart_0.UartInit()){
	sleep(1);
	}      //declare the uart port



	while(rclcpp::ok()){

	wt61cuart_0.GetAndCheck();
	wt61cuart_0.TranslateAndPub();

	

	}
	return 0;
}

