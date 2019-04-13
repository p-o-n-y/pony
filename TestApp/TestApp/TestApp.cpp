// ConsoleApplication1.cpp: определяет точку входа для консольного приложения.
//

#include "stdafx.h"
#include <stdlib.h>
#include "pony.h"
//#include <stdio.h>


void read_imu_from_file_aist(void);
void calibr(void);
void madgwick(void);
void write_quat_to_file(void);



int main()
{
	pony_add_plugin(read_imu_from_file_aist);
	pony_add_plugin(calibr);
	pony_add_plugin(madgwick);
	pony_add_plugin(write_quat_to_file);

	if (pony_init("{imu: in = ' ', }  {gnss: {glo: hnm hmjgf} in = 678 {gps: hnfgdg} } ou\nt =\1 ' \t '"));
	{
		while (pony_step());
	}

	return 0;
}

void read_imu_from_file_aist(void)
{
	if (pony.bus.mode > 0)
	{

	}
	else if (pony.bus.mode == 0)
	{

	}
	else
	{

	}
}

void calibr(void)
{
	if (pony.bus.mode > 0)
	{

	}
	else if (pony.bus.mode == 0)
	{

	}
	else
	{

	}
}

void madgwick(void)
{
	if (pony.bus.mode > 0)
	{

	}
	else if (pony.bus.mode == 0)
	{

	}
	else
	{

	}
}

void write_quat_to_file(void)
{
	if (pony.bus.mode > 0)
	{

	}
	else if (pony.bus.mode == 0)
	{

	}
	else
	{

	}
}