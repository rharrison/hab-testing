// ubxEmulate.c - a program to emulate uBlox ubx_vav_pvt from a GPS
// it expects to be given an input file containing the ubx protocol and it will
// send out the ubx message every time it is polled by the COM port to do so
//
 
#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>  /* Standard stuff like exit */
#include<windows.h>

DWORD readFromSerialPort(HANDLE hSerial, unsigned char * buffer, int maxBuffer);
DWORD writeToSerialPort(HANDLE hSerial, unsigned char * buffer, int length);
void closeSerialPort(HANDLE hSerial);
unsigned char buffer_out[1000];
unsigned char buffer_in[1000];
int Status;


int main (int argc, char **argv) 
{
	
	HANDLE hComm;

	hComm = CreateFile("COM3",       //port name
                      GENERIC_READ | GENERIC_WRITE, //Read/Write
                      0,                            // No Sharing
                      NULL,                         // No Security
                      OPEN_EXISTING,// Open existing port only
                      0,            // Non Overlapped I/O
                      NULL);        // Null for Comm Devices

	if (hComm == INVALID_HANDLE_VALUE) {
		printf("Error in opening serial port");
		exit (-1);
	}
	else
		printf("opening serial port successful");
	
	DCB dcbSerialParams = { 0 }; // Initializing DCB structure

	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

	GetCommState(hComm, &dcbSerialParams);

	dcbSerialParams.BaudRate = CBR_9600;  // Setting BaudRate = 9600
    //dcbSerialParams.BaudRate = CBR_57600;  // Setting BaudRate = 9600
	dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
	dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
	dcbSerialParams.Parity   = NOPARITY;  // Setting Parity = None
	
	if (!SetCommState(hComm, &dcbSerialParams))
	{
	printf("Error setting COMa state\r\n");
	return(0);
	}


	COMMTIMEOUTS timeouts={0};
	timeouts.ReadIntervalTimeout=50;
	timeouts.ReadTotalTimeoutConstant=50;
	timeouts.ReadTotalTimeoutMultiplier=10;
	timeouts.WriteTotalTimeoutConstant=50;
	timeouts.WriteTotalTimeoutMultiplier=10;
	if(!SetCommTimeouts(hComm, &timeouts)){
		//handle error
	}

	
	//Status = SetCommMask(hComm, EV_RXCHAR);
	
	FILE * fp;
	
	
	if (argc != 2) {
		fprintf(stderr,"\nUsage : %s <ubx binary file> < COM0 > COM0 \n", argv[0]);
		exit(-1);
	}
	
	
	if ( !(fp = fopen(argv[1],"rb"))) {
		fprintf(stderr,"\nFile '%s' not found\n", argv[1]);
		exit(-1);
	}
	
	// the main loop
	unsigned char c = 0x00;
	DWORD dwEventMask; 
	
    // while ((c=getc(stdin)) !='q')
	while(1)
	{
		int i=0;
		
		//Status = WaitCommEvent(hComm, &dwEventMask, NULL);  	
		if (strstr(buffer_in, "#")){
				Sleep(100);
				
				// for (i=0; i<100;i++) {
					fread(buffer_out,sizeof(unsigned char),100,fp);
					// printf("%02x ",buffer_out[0]);
				//	for (i=0; i<100;i++) {
				//		printf("%02x ",buffer_out[i]);
				//		if (i%16 ==15) {printf("\n");}
				//	}
					writeToSerialPort (hComm,buffer_out,100);
				
					readFromSerialPort (hComm,buffer_in,sizeof(buffer_in));
					printf ("%s",buffer_in);
			    // }
		}	else
		{
				readFromSerialPort (hComm,buffer_in,sizeof(buffer_in));
				printf ("%s",buffer_in);
				writeToSerialPort (hComm,&c,1);
		}
 	}

	CloseHandle(hComm);//Closing the Serial Port
	
	return 1; // normal termination
}

DWORD readFromSerialPort(HANDLE hSerial, unsigned char * buffer, int maxBuffer)
{
    DWORD dwBytesRead = 0;
	unsigned char tempchar;
	int i=0;
    
	do{
		ReadFile(hSerial, &tempchar, sizeof(tempchar), &dwBytesRead, NULL);
		if (dwBytesRead >0){
			buffer[i] = tempchar;// Store Tempchar into buffer
			i++;  //handle error
		}
    }while (dwBytesRead > 0 && i<maxBuffer-1);
    
	buffer[i]='\0';
	i++;
	return i;
}

DWORD writeToSerialPort(HANDLE hSerial, unsigned char * buffer, int length)
{

	DWORD dwBytesRead = 0;
	if(!WriteFile(hSerial, buffer, length, &dwBytesRead, NULL)){
		//handle error
	}
	return dwBytesRead;

}

void closeSerialPort(HANDLE hSerial)
{
	CloseHandle(hSerial);
}