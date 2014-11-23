#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int fileDescriptor = -1;

struct LegChannel {
	std::string name;
	unsigned char channel;
	unsigned short min;
	unsigned short max;
	float minAngle;
	float maxAngle;
};

LegChannel channels[] =
	 {
	{ "joint_l_1_f1",	1,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_1_f2",	2,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_1_f3",	3,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_1_f4",	4,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_2_f1",	5,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_2_f2",	6,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_2_f3",	7,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_2_f4",	8,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_3_f1",	9,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_3_f2",	10,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_3_f3",	11,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_l_3_f4",	12,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_1_f1",	13,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_1_f2",	14,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_1_f3",	15,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_1_f4",	16,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_2_f1",	17,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_2_f2",	18,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_2_f3",	19,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_2_f4",	20,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_3_f1",	21,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_3_f2",	22,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_3_f3",	23,	600, 2400,	-M_PI_2,	M_PI_2 },
	{ "joint_r_3_f4",	24,	600, 2400,	-M_PI_2,	M_PI_2 },
};

bool isOpen()
{
    return fileDescriptor != -1;
}

void openPort(const std::string& portName)
{
	if (!isOpen())
	{
		fileDescriptor = open(portName.c_str(), O_RDWR | O_NOCTTY);
		ROS_INFO("Open port %d", fileDescriptor);
		printf("test");
	}
}

void closePort()
{
	if (isOpen())
	{
		close(fileDescriptor);
		fileDescriptor = -1;
	}
}

bool writeBytes(const unsigned char* data, unsigned int numBytesToWrite)
{
	for(int j = 0; j < numBytesToWrite; j++)
	{
	    printf("\\x%02X", data[j]);
	}
	printf("\r\n");
	return true;

	if (!isOpen())
	{
		return false;
	}

	ssize_t ret = write(fileDescriptor, data, numBytesToWrite);

	if (ret == -1 || ret != numBytesToWrite)
	{
		ROS_ERROR("Port writing. err=%d\n", errno);
		return false;
	}

	return true;
}

void joinStateMessage(sensor_msgs::JointStateConstPtr message)
{
	unsigned short count = message->position.size();
	unsigned char data[51];
	data[0] = 0x9F; // compact protocol
	data[1] = 24; // channel count
	data[2] = 0; // start from first channel

	for(int i = 0; i < count; i++)
	{
		unsigned char channel = 0;
		unsigned short val = 1500;

		for(unsigned char ch = 0; ch < 24; ch++)
		{
			LegChannel l = channels[ch];
			if (l.name == message->name[i])
			{
				channel = l.channel;
				float position = message->position[i];
				val = 4 * round((l.max - l.min) * (position - l.minAngle) / (l.maxAngle - l.minAngle) + l.min);
			}
		}

		if (channel != 0)
		{
			ROS_INFO("JOIN (%s - %d)=%d", message->name[i].c_str(), channel, val);
			data[channel * 2 + 1] = val & 0x7F;
			data[channel * 2 + 2] = (val >> 7) & 0x7F;
		}
	}

	writeBytes(data, 51);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "maestro");
	ros::NodeHandle node;

	openPort("/dev/ttyS2");

	ros::Subscriber joinstateSubscriber = node.subscribe("joint_states", 100, joinStateMessage);

	ros::spin();

	closePort();

    return 0;
}


