/*!
 * 2017-18, ETSII UPM, Madrid, Spain
 * Santiago Pariente 
 */
#include "khepera3.h" 
#include "commandline.h"
#include "measurement.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <arpa/inet.h>
#include <signal.h>
#include <string.h>
#include <netinet/in.h>

#define BUFLEN 32
#define SPEED_STEP 150000
#define SPEED_LIMIT 30000 //0.2m/s*150000

#define PORTIN 3090
#define PORTOUT 3091
#define BUFLEN2 64
#define PORTOUT_CS 3092
#define PORTOUT_EP 3093

#define SIZE 5
#define NUM_OF_US_SENSORS 5

#define WHEEL_RADIUS 0.021
#define KSPEED 144.01

// Algorithm parameters and results
struct sAlgorithm {
	struct {
		int port;
		int binary;
		int raw;
		int verbosity;
	} configuration;
};

// Algorithm instance
struct sAlgorithm algorithm;

//callback: when ctrl+C is pressed, stops the motors and ends the program
void signal_callback_handler(int signum){
	printf("Caught signal %d\n",signum);
	// Cleanup and close up stuff here
	khepera3_motor_stop(&khepera3.motor_left);
	khepera3_motor_stop(&khepera3.motor_right);
	//Terminate program
	exit(signum);
}

double getMedian(int *array){
	if (SIZE % 2) return array[SIZE/2];
	else return (array[SIZE/2] + array[SIZE/2 - 1]) / 2.0;
}

void bubbleSort(int list[], int n){
  long c, d, t;
  for (c = 0 ; c < ( n - 1 ); c++){
    for (d = 0 ; d < n - c - 1; d++){
      if (list[d] > list[d+1]){
        t         = list[d];
        list[d]   = list[d+1];
        list[d+1] = t;
      }
    }
  }
}

// Prints the help text.
void help() {
	printf("Listens on a UDP port and prints all received datagrams. By default, port 3000 is used.\n");
	printf("\n");
	printf("Usage: udp_receiver [OPTIONS]\n");
	printf("\n");
	printf("Options:\n");
	printf("  -p --port PORT        Use port PORT (default: 3000)\n");
	printf("  -b --binary           Switches to binary message display (see output)\n");
	printf("  --raw                 Prints only the raw packets (not encapsulated into NMEA messages)\n");
	printf("  -v --verbosity V      Sets the verbosity level (0=quiet, 1=default, 2=verbose, 3=very verbose, ...)\n");
	printf("\n");
	printf("Output:\n");
	printf("  Default mode:\n");
	printf("    $UDP, sender [host:port], data [text]\n");
	printf("  Binary mode:\n");
	printf("    $UDP, sender [host:port], data [hex notation]\n");
	printf("  In raw mode, only the data is printed (as text or in hex notation).\n");
	printf("\n");
}

// Initializes the algorithm.
void algorithm_init() {
	// Read command line options
	algorithm.configuration.port = commandline_option_value_int("-p", "--port", 3000);
	algorithm.configuration.binary = commandline_option_provided("-b", "--binary");
	algorithm.configuration.raw = commandline_option_provided(0, "--raw");
	algorithm.configuration.verbosity = commandline_option_value_int("-v", "--verbosity", 1);
}

// Runs the algorithm by calling the appropriate state function.
void algorithm_get_speed() {	//when it acts as client
	
	int sock;
	struct sockaddr_in address;
	struct hostent *host;
	int res;

	// Initialize the address
	address.sin_family = AF_INET;
	address.sin_port = htons(PORTOUT_CS);

	// Hostname resolution
	host = gethostbyname("192.168.1.20");
	if (! host) {
		printf("Unable to resolve hostname %s\n", "192.168.1.20");
		exit(2);
	}
	address.sin_addr.s_addr = *(unsigned long*)host->h_addr;

	// Create datagram socket and send the datagram
	sock = socket(AF_INET, SOCK_DGRAM, 0);

	int i=0;
	float SL=0.0, SR=0.0;	

	int measureL[5]={0}, ordered_measureL[5]={0};
	int measureR[5]={0}, ordered_measureR[5]={0};

	char buf[1024];
	
	// Process datagrams
	while (1) {

		khepera3_drive_get_current_speed();
		
		measureL[4]=khepera3.motor_left.current_speed;
		for(i=0;i<SIZE;i++) ordered_measureL[i]=measureL[i];
		bubbleSort(ordered_measureL, SIZE);
		SL= getMedian(ordered_measureL);
		measureL[0]=measureL[1];
		measureL[1]=measureL[2];
		measureL[2]=measureL[3];
		measureL[3]=measureL[4];

		measureR[4]=khepera3.motor_right.current_speed;
		for(i=0;i<SIZE;i++) ordered_measureR[i]=measureR[i];
		bubbleSort(ordered_measureR, SIZE);
		SR= getMedian(ordered_measureR);
		measureR[0]=measureR[1];
		measureR[1]=measureR[2];
		measureR[2]=measureR[3];
		measureR[3]=measureR[4];	
				
		fflush(stdin);
		//en lugar de enviar esto envío mis propios mensajes de infrared_proximity.
		//if(SL < 0) SL = 0;
		//if(SR < 0) SR = 0;

		printf("CS L%f R%f \n", SL, SR);

		// Format to rad/s	
		SL = SL / (WHEEL_RADIUS * KSPEED * 1000);
		SR = SR / (WHEEL_RADIUS * KSPEED * 1000);	
		
		snprintf(buf, BUFLEN2, "CS L%f R%f", SL, SR);
		res = sendto(sock, buf, BUFLEN2, 0, (struct sockaddr *) & address, sizeof(struct sockaddr_in));
		printf("CS L%f R%f \n", SL, SR);
		fflush(stdout);
	}

}

// Runs the algorithm by calling the appropriate state function.
void algorithm_get_encoders() {	//when it acts as client
	
	int sock;
	struct sockaddr_in address;
	struct hostent *host;
	int res;

	// Initialize the address
	address.sin_family = AF_INET;
	address.sin_port = htons(PORTOUT_EP);

	// Hostname resolution
	host = gethostbyname("192.168.1.20");
	if (! host) {
		printf("Unable to resolve hostname %s\n", "192.168.1.20");
		exit(2);
	}
	address.sin_addr.s_addr = *(unsigned long*)host->h_addr;

	// Create datagram socket and send the datagram
	sock = socket(AF_INET, SOCK_DGRAM, 0);

	int i=0;
	float EL=0.0, ER=0.0;	

	int measureL[5]={0}, ordered_measureL[5]={0};
	int measureR[5]={0}, ordered_measureR[5]={0};

	char buf[1024];
	
	// Process datagrams
	while (1) {

		khepera3_drive_get_current_position();
		
		measureL[4]=khepera3.motor_left.current_position;
		for(i=0;i<SIZE;i++) ordered_measureL[i]=measureL[i];
		bubbleSort(ordered_measureL, SIZE);
		EL= getMedian(ordered_measureL);
		measureL[0]=measureL[1];
		measureL[1]=measureL[2];
		measureL[2]=measureL[3];
		measureL[3]=measureL[4];

		measureR[4]=khepera3.motor_right.current_position;
		for(i=0;i<SIZE;i++) ordered_measureR[i]=measureR[i];
		bubbleSort(ordered_measureR, SIZE);
		ER= getMedian(ordered_measureR);
		measureR[0]=measureR[1];
		measureR[1]=measureR[2];
		measureR[2]=measureR[3];
		measureR[3]=measureR[4];	
				
		fflush(stdin);
		//en lugar de enviar esto envío mis propios mensajes de infrared_proximity.
		//if(SL < 0) SL = 0;
		//if(SR < 0) SR = 0;		
		
		snprintf(buf, BUFLEN2, "EP L%d R%d", (int) EL, (int) ER);
		res = sendto(sock, buf, BUFLEN2, 0, (struct sockaddr *) & address, sizeof(struct sockaddr_in));
		//printf("EP L%d R%d \n", (int) EL, (int) ER);
		fflush(stdout);
	}

}

// Runs the algorithm by calling the appropriate state function.
void algorithm_run_IR() {	//when it acts as client
	
	int sock;
	struct sockaddr_in address;
	struct hostent *host;
	int res;

	// Initialize the address
	address.sin_family = AF_INET;
	address.sin_port = htons(PORTOUT);

	// Hostname resolution
	host = gethostbyname("192.168.1.20");
	if (! host) {
		printf("Unable to resolve hostname %s\n", "192.168.1.20");
		exit(2);
	}
	address.sin_addr.s_addr = *(unsigned long*)host->h_addr;

	// Create datagram socket and send the datagram
	sock = socket(AF_INET, SOCK_DGRAM, 0);

	int i=0;
	float IRL=0.0, IRFSL=0.0, IRFL=0.0;
	float IRR=0.0, IRFSR=0.0, IRFR=0.0;	

	int measureL[5]={0}, ordered_measureL[5]={0};
	int measureFSL[5]={0}, ordered_measureFSL[5]={0};
	int measureFL[5]={0}, ordered_measureFL[5]={0};
	int measureFR[5]={0}, ordered_measureFR[5]={0};
	int measureFSR[5]={0}, ordered_measureFSR[5]={0};
	int measureR[5]={0}, ordered_measureR[5]={0};

	char buf[1024];
	
	// Process datagrams
	while (1) {

		// Take a measurement with the IR sensors in proximity mode
		// Note that this takes about 3 ms
		khepera3_infrared_proximity();
		measureL[4]=khepera3.infrared_proximity.sensor[cKhepera3SensorsInfrared_Left];
		for(i=0;i<SIZE;i++) ordered_measureL[i]=measureL[i];
		bubbleSort(ordered_measureL, SIZE);
		IRL= getMedian(ordered_measureL);
		measureL[0]=measureL[1];
		measureL[1]=measureL[2];
		measureL[2]=measureL[3];
		measureL[3]=measureL[4];

		measureFSL[4]=khepera3.infrared_proximity.sensor[cKhepera3SensorsInfrared_FrontSideLeft];
		for(i=0;i<SIZE;i++) ordered_measureFSL[i]=measureFSL[i];
		bubbleSort(ordered_measureFSL, SIZE);
		IRFSL= getMedian(ordered_measureFSL);
		measureFSL[0]=measureFSL[1];
		measureFSL[1]=measureFSL[2];
		measureFSL[2]=measureFSL[3];
		measureFSL[3]=measureFSL[4];	
		
		measureFL[4]=khepera3.infrared_proximity.sensor[cKhepera3SensorsInfrared_FrontLeft];
		for(i=0;i<SIZE;i++) ordered_measureFL[i]=measureFL[i];
		bubbleSort(ordered_measureFL, SIZE);
		IRFL= getMedian(ordered_measureFL);
		measureFL[0]=measureFL[1];
		measureFL[1]=measureFL[2];
		measureFL[2]=measureFL[3];
		measureFL[3]=measureFL[4];				

		measureFR[4]=khepera3.infrared_proximity.sensor[cKhepera3SensorsInfrared_FrontRight];
		for(i=0;i<SIZE;i++) ordered_measureFR[i]=measureFR[i];
		bubbleSort(ordered_measureFR, SIZE);
		IRFR= getMedian(ordered_measureFR);
		measureFR[0]=measureFR[1];
		measureFR[1]=measureFR[2];
		measureFR[2]=measureFR[3];
		measureFR[3]=measureFR[4];			
		
		measureFSR[4]=khepera3.infrared_proximity.sensor[cKhepera3SensorsInfrared_FrontSideRight];
		for(i=0;i<SIZE;i++) ordered_measureFSR[i]=measureFSR[i];
		bubbleSort(ordered_measureFSR, SIZE);
		IRFSR= getMedian(ordered_measureFSR);
		measureFSR[0]=measureFSR[1];
		measureFSR[1]=measureFSR[2];
		measureFSR[2]=measureFSR[3];
		measureFSR[3]=measureFSR[4];

		measureR[4]=khepera3.infrared_proximity.sensor[cKhepera3SensorsInfrared_Right];
		for(i=0;i<SIZE;i++) ordered_measureR[i]=measureR[i];
		bubbleSort(ordered_measureR, SIZE);
		IRR= getMedian(ordered_measureR);
		measureR[0]=measureR[1];
		measureR[1]=measureR[2];
		measureR[2]=measureR[3];
		measureR[3]=measureR[4];	
				
		fflush(stdin);
		//en lugar de enviar esto envío mis propios mensajes de infrared_proximity.
		if(IRL < 0) IRL = 0;
		if(IRFL < 0) IRFL = 0;
		if(IRFSL < 0) IRFSL = 0;
		if(IRR < 0) IRR = 0;
		if(IRFR < 0) IRFR = 0;
		if(IRFSR < 0) IRFSR = 0;
		
		snprintf(buf, BUFLEN2, "I L%d FSL%d FL%d FR%d FSR%d R%d", (int) IRL, (int) IRFSL, (int) IRFL, (int) IRFR, (int) IRFSR, (int) IRR);
		res = sendto(sock, buf, BUFLEN2, 0, (struct sockaddr *) & address, sizeof(struct sockaddr_in));

		//printf("I L%d FSL%d FL%d FR%d FSR%d R%d \n", (int) IRL, (int) IRFSL, (int) IRFL, (int) IRFR, (int) IRFSR, (int) IRR);
		fflush(stdout);
		usleep(33000);

	}

}

void algorithm_run_driver() {
	//when it acts as server
	int sock;
	struct sockaddr_in address;
	struct sockaddr_in sender;
	char recv_buffer[BUFLEN];
	int recv_len;
	socklen_t sender_len = sizeof(struct sockaddr_in);
	int res;
	int i;
	int speed_left=0, speed_right=0, j=0;
	char str_speed_left[BUFLEN/2]={0}, str_speed_right[BUFLEN/2];
	
	char buf[1024];
	
	//left:
	if (khepera3_motor_initialize(&(khepera3.motor_left))) {
		printf("Left motor initialized.\n");
	} 
	else {
		printf("Left motor: initialization failed.\n");
	}
	
	//right:
	if (khepera3_motor_initialize(&(khepera3.motor_right))) {
		printf("Right motor initialized.\n");
	} 
	else {
		printf("Right motor: initialization failed.\n");
	}
	
	//start control (normal) mode
	khepera3_drive_start();

	khepera3_drive_set_speed(0, 0);
	
	// Initialize the address - checked
	address.sin_family = AF_INET;
	address.sin_port = htons(PORTIN);
	address.sin_addr.s_addr = htonl(INADDR_ANY);

	// Create socket and bind it - checked
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock<0) {
		printf("Could not create socket: Error %d\n", errno);
		exit(1);
	}
	
	res=bind(sock, (struct sockaddr *)&address, sizeof(address));
	if (res==-1) { //checked
		printf("Could not bind socket to port %d: Error %d\n", algorithm.configuration.port, errno);
		exit(1);
	}

	// Process datagrams
	while (1) {
		//////////Acting as a server
		// Wait for next datagram - checked	
		recv_len = recvfrom(sock, recv_buffer, BUFLEN, 0, (struct sockaddr *)&sender, &sender_len);
		
		if (recv_len==-1){ //checked
		printf("Could not receive connection: Error %d\n", errno);
		exit(1);
		}

		i=0;
		while(recv_buffer[i]){
			j=0;
			//reads left speed and copies it to str_speed_left
			if(recv_buffer[i]=='L'){
				while(recv_buffer[i+1]!=' ' && recv_buffer[i]){
					i++;
					str_speed_left[j]=recv_buffer[i];
					j++;

				}
				str_speed_left[j]=0; //null character
				i++;//to go to R letter
			}
			
			j=0;

			if(recv_buffer[i]=='R'){
				while(recv_buffer[i+1]!=' ' && recv_buffer[i]){
					i++;
					str_speed_right[j]=recv_buffer[i];
					j++;

				}
			}

			i++;
		}
						
			fflush(stdout);

			speed_left=atof(str_speed_left) * WHEEL_RADIUS * KSPEED * 1000; //converts string to float
			speed_right=atof(str_speed_right) * WHEEL_RADIUS * KSPEED * 1000; //converts string to float
	
			if (speed_left > SPEED_LIMIT) speed_left = SPEED_LIMIT;
			if (speed_right > SPEED_LIMIT) speed_right = SPEED_LIMIT;
			if (speed_left < -SPEED_LIMIT) speed_left = -SPEED_LIMIT;
			if (speed_right < -SPEED_LIMIT) speed_right = -SPEED_LIMIT;
	
			khepera3_drive_set_speed(speed_left, speed_right);

			fflush(stdin);
		
		fflush(stdout);
	}

}


// Main program.
int main(int argc, char *argv[]) {
	// Command line parsing
	commandline_init();
	commandline_option_register("-b", "--binary", cCommandLine_Option);
	commandline_option_register(0, "--raw", cCommandLine_Option);
	commandline_parse(argc, argv);
	measurement_init();
	//This is in case we terminate the program: stops the motors
	signal(SIGINT, signal_callback_handler);

	// Help
	if (commandline_option_provided("-h", "--help")) {
		help();
		exit(1);
	}

	// Initialization
	algorithm_init();
	
	khepera3_init();
	khepera3_ultrasound_enable(31);

	// Run
	switch(fork()){
		case -1:
			printf("ERROR: No process created \n");
			return 1;
		case 0:
			printf("Init run \n");
			algorithm_run_driver();
			break;
		default:
			switch(fork()){
				case -1:
					printf("ERROR: No process created \n");
					return 1;
				case 0:
					printf("Init IR \n");
					algorithm_run_IR();
					break;
				default:
					switch(fork()){
						case -1:
							printf("ERROR: No process created \n");
							return 1;
						case 0:
							printf("Init speed \n");
							algorithm_get_speed();
							break;
						default:
							printf("Init enc \n");
							algorithm_get_encoders();
					}
			}
	}

	return 0;
}
