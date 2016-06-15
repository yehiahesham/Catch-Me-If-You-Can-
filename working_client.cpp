#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>


#define threshold 15
#define LOWEST_DIS 3
#define LOWER_SPEED   0x10
#define NORMAL_SPEED  0x20
#define HIGHER_SPEED  0x40
#define ANGULAR_SPEED  0x30
#define maxBytes 1024


void terminate_with_error (const char * error_msg,int sock)
{
	perror("Error Binding Socket:");
	if (sock != -1)
	{
		close (sock);
		exit(1);
	}
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            	// 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}


void turn_left(int fd, char speed){
		char buffon[8];
		buffon[0] =  0xC6;
		buffon[1] = speed;
		buffon[2] =  0xC0;
		buffon[3] =0xFF;
		write(fd, buffon, 4);
}

void turn_right(int fd, char speed){
		char buffon[8];
		buffon[0] =  0xCE;
		buffon[1] = speed;
		buffon[2] =  0xC8;
		buffon[3] =0xFF;
		write(fd, buffon, 4);
}

void brake( int fd, int dir)
{
	char buffon[8];

	if ( dir == 0 )					// right
	{
		buffon[0] =  0xC0;
		buffon[1] =0xFF;
		write(fd, buffon, 2);
	}
	else if ( dir == 1 )				//left
	{
		buffon[0] = 0xC8;
		buffon[1] =0xFF;
		write(fd, buffon, 2);
	}
	else if ( dir == 2 )				//both
	{
		buffon[0] = 0xC0;
		buffon[2] = 0xC8;
		buffon[1] = buffon[3] = 0xFF;
		write(fd, buffon,4);
	}
}

void rotate(int fd, int angle){
	char buffon[8];
	char speed = ANGULAR_SPEED;
	char lspeed = 0x21;
    if(angle == 90){
      char buffon[8];
      buffon[0] = 0xCE;
      buffon[1] = speed;
      buffon[2] = 0xC5;
      buffon[3] = speed;
      write(fd, buffon, 4);
      //usleep(900000);
     usleep(450000);
    }
    else if(angle == 45){
      char buffon[8];
      buffon[0] = 0xCE;
      buffon[1] = lspeed;
      buffon[2] = 0xC5;
      buffon[3] = lspeed;
      write(fd, buffon, 4);
      usleep(450000);
    }
    else if(angle == -90){
      buffon[0] =  0xC6;
      buffon[1] = speed;
      buffon[2] = 0xCD;
      buffon[3] = speed;
      write(fd, buffon, 4);
      usleep(450000);
    }
    else if(angle == -45){
      buffon[0] =  0xC6;
      buffon[1] = lspeed;
      buffon[2] = 0xCD;
      buffon[3] = lspeed;
      write(fd, buffon, 4);
      usleep(450000);

    }
    else if(angle == 180){
      char buffon[8];
      buffon[0] = 0xCE;
      buffon[1] = speed;
      buffon[2] = 0xC5;
      buffon[3] = speed;
      write(fd, buffon, 4);
      usleep(1700000);
    }
	brake(fd, 2);
}

void accelerate(int fd, int dir, char speed){
	char buffon[8];

	if ( dir == 0 )					// right hard
	{
		buffon[0] =  0xC6;
		buffon[1] = speed;
		buffon[2] = 0xCD;
		buffon[3] = speed;
		write(fd, buffon, 4);
	}
  else if ( dir == 4 )					// right soft
  {
    buffon[0] =  0xC6;
    buffon[1] = speed;
    buffon[2] = 0xCE;
    buffon[3] = 0x10;
    write(fd, buffon, 4);
  }
	else if ( dir == 1 )				//left hard
	{
		buffon[0] = 0xCE;
		buffon[1] = speed;
		buffon[2] = 0xC5;
		buffon[3] = speed;
		write(fd, buffon, 4);
	}
  else if ( dir == 3 )				//left soft
  {
    buffon[0] = 0xCE;
    buffon[1] = speed;
    buffon[2] = 0xC6;
    buffon[3] = 0x10;
    write(fd, buffon, 4);
  }
	else if ( dir == 2 )				//both
	{
		buffon[0] = 0xC6;
		buffon[2] = 0xCE;
		buffon[1] = buffon[3] = speed;
		write(fd, buffon, 4);
	}
							//reverse
	else if (dir == 5)
	{
	buffon[0] = 0xC5;
	buffon[2] = 0xCD;
	buffon[1] = buffon[3] = speed;
	write(fd, buffon, 4);

	}

}



double diff(struct timespec start, struct  timespec end)
{
		double temp;
		temp = ((double)end.tv_sec + 1.0e-9*end.tv_nsec) - ((double)start.tv_sec + 1.0e-9*start.tv_nsec);
		return temp;
}


float Convertcm( long microsec)
{
	return 0.00226*(microsec);
}


int MAX(long double a, long double  b)
{
  if (abs(a-b)<=2)
        return 2;
  else if (a<b)
        return 1;
  else  //a>b
        return 0;
}

int main(void)
{
  //Dagu Setup

    FILE *UltraSonic1;
    FILE *UltraSonic2;
    int x,counter=0,gpio=22, flag=0, flag_right=0,flag_left=0;
    struct timespec  time0 ,  time1;
  	double difference;
  	long double cm, cmleft=0, cmright=0, cmleft1=0, cmright1=0;
  	char *portname = "/dev/ttyO3",sensor;
//    int SPEED=HIGHER_SPEED*1.2;
    int SPEED=LOWER_SPEED;

  	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  	if (fd < 0)
  	{
  			printf("error %d opening %s: %s", errno, portname, strerror (errno));
  			return -1;
  	}

  	set_interface_attribs (fd, B19200, 0);		// set speed to 115,200 bps, 8n1 (no parity)
  	set_blocking (fd, 0);				// set no blocking


//righthere
	//accelerate(fd, 2, ANGULAR_SPEED);
        //usleep(1700000);
	//brake(fd, 2);
        //printf("DOUBLE Turning right , NO SPEED\n" );





 //Simple Server

int sock;

struct sockaddr_in serverAddr;
struct sockaddr_in clientAddr;
socklen_t sin_size = sizeof(struct sockaddr_in);

if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) terminate_with_error("Error Creating Socket",sock);
int sock_opt = 1;
setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,(void *)&sock_opt,sizeof (sock_opt));
      serverAddr.sin_family = AF_INET;
      serverAddr.sin_port = htons(9999);
      serverAddr.sin_addr.s_addr = INADDR_ANY;
      bzero(&(serverAddr.sin_zero), 8);
if (bind(sock, (struct sockaddr *)&serverAddr, sizeof(struct sockaddr)) == -1) terminate_with_error("Error Binding",sock);

if (listen(sock, 10) == -1) terminate_with_error("Error Listening: ",sock);
int newsock = accept(sock, (struct sockaddr *)&clientAddr,&sin_size);

char buffer[maxBytes];

if ( newsock < 1 ) terminate_with_error("Error Accepting Socket",0);
  else
{
		//loop for Receiving multiple commands
    do
  {
      memset(buffer,0, maxBytes);
      int bytes_read = recv (newsock,buffer,maxBytes,0);
      if(bytes_read <= 0) {printf("bytes_read = %d\n",bytes_read ); perror("Error Receiving Message:"); return -2;}
     else
    {
		//	printf ("Received Message from %s:%d\n%s\n",(char *)inet_ntoa(clientAddr.sin_addr),clientAddr.sin_port,buffer);
			char * token;
			token = strtok ((char*)buffer,"\n");

			//loop of multiple commands Received
			while(token!=NULL)
			{

				//use of the token

				if(!strcmp(token, "Dagu_Stop"))
	      {
	          brake(fd, 2);
	          printf("STOPPING DAGU, SERVER\n");
	      }
	      else if(!strcmp(token, "Dagu_Move"))
	      {
						accelerate(fd, 2, 0x18);
						usleep(500000);
	        	accelerate(fd, 2, 0x15); //SPEED
	        	printf("MOVING DAGU, SERVER\n");
	      }
				else if(!strcmp(token, "Dagu_Reverse"))
				{
						accelerate(fd,5,0x18);
//accelerate(fd, 5, 0x13);
						usleep(500000);
//accelerate(fd, 2, 0x10);
						accelerate(fd,5,0x15);
						printf("DAGU MOVING BACKWARDS, SERVER \n");
				}
	      else if (!strcmp(token, "Dagu_Left"))  //double-rotation
	      {
								accelerate(fd, 0, ANGULAR_SPEED);
	              accelerate(fd, 0, ANGULAR_SPEED);
	              usleep(1160000);
	              printf("DOUBLE Turning left , NO SPEED\n" );
			 					brake(fd, 2);
	      }
	      else if (!strcmp(token, "Dagu_Right_45"))  //double-rotation
	      {
										printf("DAGU ROTATING RIGHT 45 \n");
	                rotate(fd, 45);
	      }
	      else if (!strcmp(token, "Dagu_Right_90"))//  double-rotation
	      {
	                rotate(fd, 90);
			printf("DAGU Turing right 90 \n");
	      }
	      else if (!strcmp(token, "Dagu_Left_45"))//  rotation
	      {
	                rotate(fd, -45);
                        printf("DAGU Turning Left 45\n");

	      }
	      else if (!strcmp(token, "Dagu_Left_90")) // double-rotation
	      {
	                rotate(fd, -90);
                        printf("DAGU Turing Left 90 \n");

	      }
	      else if (!strcmp(token, "Dagu_180")) // double-rotation
	      {
	                rotate(fd, 180);
                        printf("DAGU Turning 180 \n");

	      }
				else
	      {
	      	    printf ("COULDN'T deal with %s", token);
	      }
				//get next command
				token = strtok (NULL, "\n");
			}
		}
	}while(true);
}
close(newsock);
close(sock);
}
