#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "pc_node/serializer.h"
#include "pc_node/types.h"
#include "custom_msgs/DataSensors.h"

#define _WINSOCK_DEPRECATED_NO_WARNINGS


void runClient(const uint8* buff, size_t buffSize){
	int conn;
    int response;
    WSADATA WSAData;
    SOCKET sock;
    SOCKADDR_IN serverAddr;
    socklen_t serverAddrLen;

    WSAStartup(MAKEWORD(2,2), &WSAData);

    sock = socket(AF_INET, SOCK_STREAM, 0);
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(50300);
    serverAddrLen = sizeof(serverAddr);

    conn = ::connect(sock, (SOCKADDR *)&serverAddr, serverAddrLen);

    if(conn >= 0){
        response = send(sock, (char*)buff, static_cast<int>(buffSize), 0);
        if(response == SOCKET_ERROR){
            std::cout << "L'envoi a echoue" << std::endl;
        }
    }else std::cout << "Connexion au serveur impossible (conn = " << conn << ")" << std::endl;

    closesocket(sock);
    WSACleanup();
}

void raspiEOMCallback(const custom_msgs::DataSensors::ConstPtr& msg){
	Serializer serial;
	uint8 index  = Index::EOM;
	
	ROS_INFO("[ INFO ] : Mesures terminees");
	
	serial.write(index);
	serial.write(msg->heights);
	serial.write(msg->airSpeed);
	serial.write(msg->airSpeedTimer);
	serial.write(msg->temperature);
	serial.write(msg->temperatureTimer);

	runClient(serial.buffer(), serial.bufferSize());
}

void processedMeasureCallback(const std_msgs::Int32::ConstPtr& msg){
	Serializer serial; 
	uint8 index = Index::U_IHM;
	
	ROS_INFO("[ INFO ] : mesure %d, OK", msg->data);
	
	serial.write(index); 
	serial.write(msg->data);
	
	runClient(serial.buffer(), serial.bufferSize());
}

void sensorInitCallback(const std_msgs::Bool::ConstPtr& msg){
	 
	if(msg->data){
		Serializer serial;
		uint8 index = Index::INIT_SENSOR_POS;
		
		ROS_INFO("[ INFO ] : Position des capteurs initialisee");
		
		serial.write(index);
		runClient(serial.buffer(), serial.bufferSize());
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "intermediary_node");
	ros::NodeHandle nh;
	ros::Subscriber sub_EOM = nh.subscribe("raspi_eom", 5, raspiEOMCallback);
	ros::Subscriber sub_processed_measure = nh.subscribe("processed_measure", 5, processedMeasureCallback); 
	ros::Subscriber sub_sensor_init_pos = nh.subscribe("sensor_init_pos", 5, sensorInitCallback);
	
	
	ROS_INFO("INTERMEDIARY NODE LAUNCHED ...");
	ros::spin();

	return 0;
}