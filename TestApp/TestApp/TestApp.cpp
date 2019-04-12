// ConsoleApplication1.cpp: определяет точку входа для консольного приложения.
//

#include "stdafx.h"
#include <stdlib.h>
#include "pony.h"
//#include <stdio.h>


void read_imu_from_file_aist();
void calibr();
void madgwick();
void write_quat_to_file();



int main()
{
	pony_add_plugin(read_imu_from_file_aist);
	pony_add_plugin(calibr);
	pony_add_plugin(madgwick);
	pony_add_plugin(write_quat_to_file);

	if (pony_init("{imu: in = ' ', }  {gnss: {glo: hnm hmjgf} in = 678 {gps: hnfgdg} } out = ' '"));
	{
		while (pony_step());
	}


	return 0;
}

void read_imu_from_file_aist()
{

}

void calibr()
{

}

void madgwick()
{

}

void write_quat_to_file()
{

}