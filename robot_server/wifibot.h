#include <pthread.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <termios.h>//for keyboard and serial port

#include <fcntl.h>
#include <linux/i2c-dev.h>

int simu=0;

#define NB_THREADS	12

void gestionnaire (int numero);

static int compteur = 0;
pthread_t thread [NB_THREADS];
int       i;
int       ret;

int dog=0;
pthread_mutex_t	mutex_dog = PTHREAD_MUTEX_INITIALIZER;

#define MAXPENDING 5    /* Maximum outstanding connection requests */

static int servSock;                    /* Socket descriptor for server */
static int clntSock;                    /* Socket descriptor for client */
static struct sockaddr_in echoServAddr; /* Local address */
static struct sockaddr_in echoClntAddr; /* Client address */
static unsigned short echoServPort=15000;     /* Server port */
static unsigned int clntLen;            /* Length of client address data structure */
static int recvMsgSize=0; 
static unsigned char buffso_rcv[32]; 
static unsigned char buffso_send[28];

int first=1;
static int connected=0;
int debug=0;
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
typedef struct 
{
	int SpeedFront;
	int robot_voltage;
	int IR;
	int IR2;
	long odometry;
	int robot_current;
	int robot_version;
}SensorData;

int Data_Of_Thread_In = 1;            // Data of Thread 1
char buffso_rcvtcp[32]; 
char buffso_sendtcp[30];

int servermode = 0;
int watchdog = 0;
int cycles = 0;
unsigned char speed1 = 0;
unsigned char speed2 = 0;

char buffso_rcvIN[100]; 
char buffso_rcvOUT[100]; 
char buffso_sendIN[100];
char buffso_sendOUT[100];
char buffer_address[sizeof(struct sockaddr)] = "0.0.0.0";
char buffer_localip[sizeof(struct sockaddr)] = "0.0.0.0";

int tcpon=0;
SensorData dataL;
SensorData dataR;
struct termios oldoptions,newoptions;

void * Thread_TCP(void * lpParam);
void * Thread_TCP_Trooper_In(void * lpParam );
void * Thread_TCP_Trooper_In_Truck(void * lpParam );
void * Thread_TCP_Trooper_Out(void * lpParam );
void * Thread_RS232_33f(void * lpParam );
void * Thread_Dog(void * lpParam );

int SetRS232Motor33f(int hUSB,short speed1,short speed2,unsigned char SpeedFlag);
int SetRS232MotorPID33f(int hUSB,char speed1,char speed2,char pp,char ii,char dd,short maxspeed);
int StopMotorRS23233f(int hUSB);
int GetRS232Motor33f(int hUSB,SensorData *dataL,SensorData *dataR);
int SetRS232Motor33f_low_res(int hUSB,char speed1,char speed2);

//#define debug_msg
int hUSB=0;

pthread_mutex_t	mutex_send = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t	mutex_rcv = PTHREAD_MUTEX_INITIALIZER;

////Threads/////////////////////////////////////////////
int socket_in;
int socket_out;
int socket_tcp;
struct sockaddr_in myaddr_in;
struct sockaddr_in myaddr_out;
struct sockaddr_in myaddr_tcp;
struct sockaddr_in client_in;
struct sockaddr_in client_out;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int openrs232(const char *device, int baudrate);
int readrs232(int hUSB, unsigned char *buffer, unsigned int nNumberOfBytesToRead, unsigned int *lpNumberOfBytesRead);
int closers232();
int writers232(int hUSB, unsigned char *buffer, unsigned int nNumberOfBytesToWrite, unsigned int *lpNumberOfBytesWritten);
short crc16(unsigned char *adresse_tab , unsigned char taille_max);
