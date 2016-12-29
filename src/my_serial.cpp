///*
// * my_serial.cpp
// *
// *  Created on: Sep 13, 2016
// *      Author: odroid
// */
#include "my_serial.h"
#include "AttitudePosition.h"
using namespace std;
//
//serial::Serial my_serial("/dev/ttySAC0", 230400,
//		serial::Timeout::simpleTimeout(1000));
//
//float Pitch = 0.0, Yaw = 0.0, Roll = 0.0;
unsigned char data_to_send[50];
int Length = 0;
//
//bool is_print_character;
//
////void uartReadThread()
////{
////	size_t data_length = 0;
////	unsigned char sum = 0;
////	unsigned char data_buf[50] = { 0 };
////	int16_t temp = 0;
////	while (true)
////	{
////		if (flag_LX_target==0)
////		{
////			usleep(1000 * delay_ms);
////			continue;
////		}
////		sum = 0;
////		data_length = my_serial.available();
////		//cout << "data_length:" << data_length << endl;
////		if (data_length)
////		{
////			my_serial.read(data_buf, data_length);
////			for (size_t i = 0; i < (data_length - 1); i++)
////			{
////				sum += *(data_buf + i);
////			}
////
////			if (!(sum == *(data_buf + data_length - 1)))
////				continue;		//������sum
////			if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))
////				continue;		//������������
////			if (*(data_buf + 2) == 0x01)
////			{
////				state_num = data_buf[4];
////				target_num = data_buf[5];
////
////				temp = data_buf[6];
////				temp <<= 8;
////				temp |= data_buf[7];
////				Pitch = (float) temp / 10.0f;
////
////				temp = data_buf[8];
////				temp <<= 8;
////				temp |= data_buf[9];
////				Roll = (float) temp / 10.0f;
////
////				temp = data_buf[10];
////				temp <<= 8;
////				temp |= data_buf[11];
////				Yaw = (float) temp / 10.0f;
////
////				for(size_t i=0;i<10;i++)
////				{
////					ignore_char[i] = data_buf[12+i];
////				}
////
////			}
////		}
////		if (isTerminal)
////		{
////			break;
////		}
////		usleep(1000 * delay_ms);
////	}
////	cout << "thread quit" << endl;
////}

void uartSent()
{
	int _cnt = 0, i = 0, sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x21;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = Markers.size();
    data_to_send[_cnt++] = int(coordinate_camera.x)>>8;
    data_to_send[_cnt++] = int(coordinate_camera.x)%256;
    data_to_send[_cnt++] = int(coordinate_camera.y)>>8;
    data_to_send[_cnt++] = int(coordinate_camera.y)%256;
    data_to_send[_cnt++] = int(coordinate_camera.z)>>8;
    data_to_send[_cnt++] = int(coordinate_camera.z)%256;
    data_to_send[_cnt++] = int(attitude_camera.Pit)>>8;
    data_to_send[_cnt++] = int(attitude_camera.Pit)%256;
    data_to_send[_cnt++] = int(attitude_camera.Rol)>>8;
    data_to_send[_cnt++] = int(attitude_camera.Rol)%256;
    data_to_send[_cnt++] = int(attitude_camera.Yaw)>>8;
    data_to_send[_cnt++] = int(attitude_camera.Yaw)%256;
    data_to_send[_cnt++] = int(flow_out[0])>>8;
    data_to_send[_cnt++] = int(flow_out[0])%256;
    data_to_send[_cnt++] = int(flow_out[1])>>8;
    data_to_send[_cnt++] = int(flow_out[1])%256;


    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    Length = _cnt;

	serial_port.write(data_to_send, Length);
}
