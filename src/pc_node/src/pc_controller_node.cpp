#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <string>
#include <cstdint>
#include "custom_msgs/Parameters.h"
#include "custom_msgs/Heights.h"
#include "pc_node/serializer.h"
#include "pc_node/deserializer.h"
#include "pc_node/types.h"
#include "custom_msgs/DataSensors.h"
#include "custom_msgs/Delay.h"

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define BUFFER_MAX 20000
//#define DEBUG_

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_controller_node");
	ros::NodeHandle nh;
	
	//Publishers
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("velocities", 5);
	ros::Publisher pub_param = nh.advertise<custom_msgs::Parameters>("parameters", 5);
	ros::Publisher pub_heights = nh.advertise<custom_msgs::Heights>("heights_list", 5);
	ros::Publisher pub_raspi_go = nh.advertise<std_msgs::Bool>("raspi_go", 5);
	ros::Publisher pub_init_sensor_pos = nh.advertise<std_msgs::Bool>("init_sensor_pos", 5);
	ros::Publisher pub_delay = nh.advertise<custom_msgs::Delay>("delay", 5);
	ros::Publisher pub_aero_vel_out_type = nh.advertise<std_msgs::Int32>("volt_out_type", 5);
	ros::Publisher pub_stop_measure = nh.advertise<std_msgs::Bool>("stop_measure", 5);
	
	//Messages
	std_msgs::Bool raspi_go_msg;
	std_msgs::Bool init_sensor_position_msg;
	geometry_msgs::Twist velocities_msg;
	custom_msgs::Parameters param_msg;
	custom_msgs::Heights heights_msg;
	std_msgs::Int32 volt_out_type_msg;
	std_msgs::Bool stop_measure_msg;
	
	//Sockets
	WSADATA WSAData;
	SOCKET sock;
	SOCKET csock;
	SOCKET sockIHM;
	SOCKET sock_init_sensor_pos;
	SOCKADDR_IN ssin;
	SOCKADDR_IN csin;
	socklen_t ssinSize;
	uint8 buff[BUFFER_MAX];
	int error;
	int res;
	size_t nbBytes;
	
	raspi_go_msg.data = true;
	init_sensor_position_msg.data = false;
	
	//Initialisation des vitesses
	velocities_msg.linear.x = 0.;
	velocities_msg.linear.y = 0.;
	velocities_msg.linear.z = 0.;
	velocities_msg.angular.x = 0.;
	velocities_msg.angular.y = 0.;
	velocities_msg.angular.z = 0.;
	
    WSAStartup(MAKEWORD(2,2), &WSAData);
    sock = socket(AF_INET, SOCK_STREAM, 0);
	
	if(sock == INVALID_SOCKET){
#ifdef DEBUG_
		cout << "INVALID SOCKET ..." << endl;
#endif
		return -1;
	}
	
    ssin.sin_addr.s_addr = htonl(INADDR_ANY);
    ssin.sin_family = AF_INET;
    ssin.sin_port = htons(50300);
    error = ::bind(sock, (SOCKADDR *)&ssin, sizeof(ssin));
	
    listen(sock, 5);
	ssinSize = sizeof(csin);
	
	ROS_INFO("PC CONTROLLER NODE LAUNCHED ...");
	
	while(ros::ok())
	{
		csock = accept(sock, (SOCKADDR *)&csin, &ssinSize);
		
        if(csock != INVALID_SOCKET)
        {
			res = recv(csock, (char*)buff, BUFFER_MAX, 0);
			
#ifdef DEBUG_
			std::cout << "\n------------------------------------------------------------------------\n";
			ROS_INFO("recv = %d octet(s)", res);
#endif
			
			nbBytes = res;
			Deserializer deserializer(buff, nbBytes);
			uint8 index{0};
			
			deserializer.read(index);
#ifdef DEBUG_
			ROS_INFO("index = %d", (int)index);
#endif
			
			switch(index){
				case Index::VELOCITIES:
				{	
					float64 vx, vy, om;
					
					deserializer.read(vx);
					deserializer.read(vy);
					deserializer.read(om);
					ROS_INFO("Vx = %f ; Vy = %f ; theta_z = %f", vx, vy, om); 
					
					velocities_msg.linear.x = vx;
					velocities_msg.linear.y = vy;
					
					velocities_msg.angular.z = om;
					
					pub_vel.publish(velocities_msg);
					closesocket(csock);
					
					break;
				}
				
				case Index::PARAMETERS:
				{
					ParametersIHM data;
					
					deserializer.read(data.dutyCycle);
					deserializer.read(data.pulleyRadius);
					deserializer.read(data.motorAccuracy);
					deserializer.read(data.motorAccuracyDivider);
					deserializer.read(data.linVelMotorMast);
					
					param_msg.pulleyRadius = data.pulleyRadius;
					param_msg.motorAccuracy = data.motorAccuracy;
					param_msg.motorAccuracyDivider = data.motorAccuracyDivider;
					param_msg.dutyCycle = data.dutyCycle;
					param_msg.linVelMotorMast = data.linVelMotorMast;
#ifdef DEBUG_
					ROS_INFO("pulley radius = %f mm", data.pulleyRadius);
					ROS_INFO("motor accuracy = %f mm/pulse", data.motorAccuracy);
					ROS_INFO("motor accuracy divider = %d", data.motorAccuracyDivider);
					ROS_INFO("linear velocity = %d mm/s", data.linVelMotorMast);
					ROS_INFO("duty cycle = %d %%", data.dutyCycle);

					std::cout << "\n------------------------------------------------------------------------\n";
#endif
					
					pub_param.publish(param_msg);
					closesocket(csock);
					
					break;
				}
				
				case Index::HEIGHTS:
				{
					HeightsList data;
					int sz;
					
					deserializer.read(data.heights);
					sz = data.heights.size();
#ifdef DEBUG_
					for(int i = 0; i < sz; i++){
						ROS_INFO("h_%d = %d mm", i+1, data.heights[i]);
					}
					std::cout << "\n------------------------------------------------------------------------\n";
#endif
					
					heights_msg.heightList.clear();
					heights_msg.heightList = data.heights;
					
					pub_heights.publish(heights_msg);
					closesocket(csock);
					
					break;
				}
				
				case Index::DELAY:
				{
					custom_msgs::Delay delay;
					
					deserializer.read(delay.restDelay);
					deserializer.read(delay.measureDelay);
					deserializer.read(delay.periodDelay);
#ifdef DEBUG_
					ROS_INFO("temps de repos = %d s", delay.restDelay);
					ROS_INFO("duree de la mesure = %d s", delay.measureDelay);
					ROS_INFO("frequence de la mesure = %.2f s", delay.periodDelay);
					std::cout << "\n------------------------------------------------------------------------\n";
#endif
					
					pub_delay.publish(delay);
					closesocket(csock);
					break;
				}
				
				case Index::INIT_SENSOR_POS:
				{
					if(!init_sensor_position_msg.data){
						sock_init_sensor_pos = csock;
						init_sensor_position_msg.data = true;
						pub_init_sensor_pos.publish(init_sensor_position_msg);
					}
					else{
						uint8 state = 1;
						
						init_sensor_position_msg.data = false;
						send(sock_init_sensor_pos, (char*)&state, sizeof(state), 0);
						closesocket(sock_init_sensor_pos);
					}
					
					break;
				}
				
				case Index::RPI_GO:
				{
					pub_raspi_go.publish(raspi_go_msg);
					sockIHM = csock;
					break;
				}
				
				case Index::U_IHM:
				{
					int32 val;
					Serializer serial;
					
					deserializer.read(val);
					
					serial.write(index);
					serial.write(val);
					
					send(sockIHM, (char*)serial.buffer(), static_cast<int>(serial.bufferSize()), 0);
					
					break;
				}
				
				case Index::EOM:
				{
					Serializer serial;
					custom_msgs::DataSensors ds;
					
					ROS_INFO("Acquisition des donnees des capteurs terminee ...");
					ROS_INFO("Transmission des donnees vers l'IHM");
					
					deserializer.read(ds.heights);
					deserializer.read(ds.airSpeed);
					deserializer.read(ds.airSpeedTimer);
					deserializer.read(ds.temperature);
					deserializer.read(ds.temperatureTimer);
					
					serial.write(index);
					serial.write(ds.heights);
					serial.write(ds.airSpeed);
					serial.write(ds.airSpeedTimer);
					serial.write(ds.temperature);
					serial.write(ds.temperatureTimer);
					
					if((res = send(sockIHM, (char*)serial.buffer(), static_cast<int>(serial.bufferSize()), 0)) == SOCKET_ERROR){
						ROS_INFO("Donnees non transmises ...");
					}
					
					closesocket(sockIHM);
					break;
				}
				
				case Index::NEXT_MEASURE:
				{
					pub_raspi_go.publish(raspi_go_msg);
#ifdef DEBUG_
					ROS_INFO("NEXT MEASURE");
#endif
					break;
				}
				
				case Index::VEL_VOLT_OUT_TYPE:
				{
					deserializer.read(volt_out_type_msg.data);
					
					pub_aero_vel_out_type.publish(volt_out_type_msg);
					
					break;
				}
				
				case Index::STOP_MEASURE:
				{
					stop_measure_msg.data = true;
					pub_stop_measure.publish(stop_measure_msg);
					break;
				}
			}
        }
	}
	
	closesocket(sock);
	WSACleanup();
	return 0;
}
	
	