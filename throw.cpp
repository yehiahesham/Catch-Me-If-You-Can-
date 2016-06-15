#include <iostream>
#include <stdio.h>
#include <string>
#include <time.h>
#include <stdlib.h>


FILE *arm;
int fd,k=0;

void Move_Arm()
{
	struct timespec clock={0,350000},p;
			//rotate 180 degrees
		for(k=0;k<35;k++)
		   {
			arm = fopen("/sys/class/gpio/gpio38/value","w+");
			fwrite("1", sizeof(char), 1, arm);
			fclose(arm);
			p.tv_sec = 0; p.tv_nsec= 20000000-clock.tv_nsec;
			nanosleep(&clock, NULL);


			//yerga3
			arm = fopen("/sys/class/gpio/gpio38/value","w+");
			fwrite("0", sizeof(char), 1, arm);
			fclose(arm);
			nanosleep(&p, NULL);
		   }
}


int main()
{
	// ARM setup
	system("echo 38 > /sys/class/gpio/export");
	system("echo out > /sys/class/gpio/gpio38/direction");

	Move_Arm();
        printf("ARM MOVED \n");
	return 0;
}

