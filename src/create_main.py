
cpp_template = """
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

    location1!@#

    location2!@#

	

	ros::Rate loop_rate(1000);

	while(ros::ok()){

    location3!@#

        loop_rate.sleep();
	}
	return 0;
}
"""

cpp_ros2_template = """

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

    location1!@#

    location2!@#

	while(rclcpp::ok()){

	location3!@#

	}
	return 0;
}

"""

def create_main(count=4):
    global cpp_ros2_template
    location1 = ""
    location2 = ""
    location3 = ""
    for i in range(count):
        if i != 0:
            location1 += f"WTU::custom_Wt61cUart wt61cuart_{i}(baudrate,0,\"/dev/ttyUSB{i}\",\"/imu{i}\");\n\t"
            location2 += f"\twhile(wt61cuart_{i}.UartInit()){{\n\tsleep(1);\n\t}}      //declare the uart port\n\n"
            location3 += f"wt61cuart_{i}.GetAndCheck();\n\twt61cuart_{i}.TranslateAndPub();\n\n\t"
        else:
            location1 += f"WTU::custom_Wt61cUart wt61cuart_{i}(baudrate,0,\"/dev/ttyUSB0\",\"/imu0\");\n\t"
            location2 += f"while(wt61cuart_{i}.UartInit()){{\n\tsleep(1);\n\t}}      //declare the uart port\n\n"
            location3 += f"wt61cuart_{i}.GetAndCheck();\n\twt61cuart_{i}.TranslateAndPub();\n\n\t"

    cpp_ros2_template = cpp_ros2_template.replace("location1!@#", location1)
    cpp_ros2_template = cpp_ros2_template.replace("location2!@#", location2)
    cpp_ros2_template = cpp_ros2_template.replace("location3!@#", location3)
    # with open("src/main.cpp", "w") as f:
    #     f.write(cpp_template)

    print(cpp_ros2_template)

    with open("wt61c_uart_node_custom.cpp", "w") as f:
        f.write(cpp_ros2_template)

num = int(input("Enter the number of IMUs: "))
if num == 0:
    num = 1
create_main(num)
# create_main(4)