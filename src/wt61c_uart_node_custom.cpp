
#include "ros/ros.h"
#include "wt61c_uart.h"
#include "serial/serial.h"
#include <time.h>
#include <vector>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Wt61cUart_node");

	ros::NodeHandle n;

	int baudrate = 115200;

    WTU::custom_Wt61cUart wt61cuart_0(n,baudrate,0,"/dev/wt61c_link0","/imu0");
	WTU::custom_Wt61cUart wt61cuart_1(n,baudrate,0,"/dev/wt61c_link1","/imu1");
	WTU::custom_Wt61cUart wt61cuart_2(n,baudrate,0,"/dev/wt61c_link2","/imu2");
	WTU::custom_Wt61cUart wt61cuart_3(n,baudrate,0,"/dev/wt61c_link3","/imu3");
	

    while(wt61cuart_0.UartInit()){
	sleep(1);
	}      //declare the uart port

	while(wt61cuart_1.UartInit()){
	sleep(1);
	}      //declare the uart port

	while(wt61cuart_2.UartInit()){
	sleep(1);
	}      //declare the uart port

	while(wt61cuart_3.UartInit()){
	sleep(1);
	}      //declare the uart port



	

	ros::Rate loop_rate(1000);

	while(ros::ok()){

    wt61cuart_0.GetAndCheck();
	wt61cuart_0.TranslateAndPub();

	wt61cuart_1.GetAndCheck();
	wt61cuart_1.TranslateAndPub();

	wt61cuart_2.GetAndCheck();
	wt61cuart_2.TranslateAndPub();

	wt61cuart_3.GetAndCheck();
	wt61cuart_3.TranslateAndPub();

	

        loop_rate.sleep();
	}
	return 0;
}
