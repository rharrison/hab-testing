// emulate.c - a program to emulate NMEA sentances from a GPS
// it expects to be given an input file similar to a GPS log.
// it calculates and adds NMEA checksums and paces the output as if it were being sent in real time.
//
// emulate is a unix 'filter' i.e. it reads from standard input and write to standard output
// any arguments are ignored 
//
// use with command line re-direction to output to serial port
// dos e.g. emulate <gps.log >COM2:
// to send gps.log to the COM2 serial port (adding checksums and re-timing on the way)
//
// the dos "mode" command can be used to change the serial port speed.
// e.g. mode COM2:9600,N,8,1 
// to set to 9600baud, No parity,8 data bits, 1 stop
//
 
#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>  /* Standard stuff like exit */
#include <time.h>
#include <string.h>  /* String function definitions */
#include <math.h>
 
// radians to degrees
#define DEGREES(x) ((x) * 57.295779513082320877) 
// degrees to radians
#define RADIANS(x) ((x) / 57.295779513082320877)
 
#define BUF_SIZE 256	// size of input buffer
 
#define NONE	0
#define GPGGA	1
#define GPRMC	2
#define GPVTG	3

int main (int argc, char **argv) ;
 
char buf[BUF_SIZE];		// input buffer
 
double BaseSec = 0.0;	// set to time of first valid reading in file
double BaseLat = 0.0;	// set to Latitude of first valid reading in file
double BaseLon = 0.0;	// set to Longtitude time of first valid reading in file
 
// **************************************************************************************
//
// This routines constructs the "dynamic" KML file
//
//
// lat,lon,alt and description set for modes 1 and 2 
// first call (kml_state == 0) causes jus the file header etc to be written 
// the 2nd call (kml_state == 1) writes the launch position
// subsiquent calls (kml_state == 2) write the current position
 
long lastpos;
FILE *fpkml;
int kml_state = 0;
char *kml_file = "livekml.kml";
 
void kml_gen(double lat, double lon, double alt, char * description)
 
{
	switch(kml_state)
	{
	case 0: // this section written on startup
		if((fpkml = fopen(kml_file,"w")) == NULL) // create file - wipe existing contens
			return;	// cant open it now - maybe next-time
 
		fprintf(fpkml,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(fpkml,"<kml xmlns=\"http://earth.google.com/kml/2.1\">\n");
		fprintf(fpkml,"<Document>\n");
 
		fprintf(fpkml,"<Style id=\"track\">\n");
		fprintf(fpkml,"<LineStyle> <color>fff010c0</color> </LineStyle>\n");
		fprintf(fpkml,"<PolyStyle> <color>3fc00880</color> </PolyStyle>\n");
		fprintf(fpkml,"</Style>\n");
 
		fprintf(fpkml,"<Style id=\"place\">\n");
		fprintf(fpkml,"<IconStyle> <scale>1</scale> <Icon> <href>http://weather.uwyo.edu/icons/purple.gif</href> </Icon> </IconStyle>\n");
		fprintf(fpkml,"</Style>\n");
 
		lastpos = ftell(fpkml); // rewind to here to add launch placemark and initial position
		kml_state++; // state 1 next time
		break;
 
	case 1: // this section written when initial position placemark when it is known
		if((fpkml = fopen(kml_file,"r+")) == NULL) // open exiting file
			return;	// cant open it now - maybe next-time
 
		fseek(fpkml,lastpos,SEEK_SET); // rewind to write over final section of state 0 call
 
		fprintf(fpkml,"<Placemark> <name>Launch</name> <styleUrl>#place</styleUrl>\n");
		fprintf(fpkml,"<description>%s</description>\n",description);
		fprintf(fpkml,"<Point> <altitudeMode>absolute</altitudeMode>\n");
		fprintf(fpkml,"<coordinates>%f,%f,%f</coordinates>\n",lon,lat,alt);
		fprintf(fpkml,"</Point>\n</Placemark>\n");
 
		fprintf(fpkml,"<Placemark> <name>Flight Path</name> <styleUrl>#track</styleUrl>\n");
		fprintf(fpkml,"<LineString> <extrude>1</extrude> <altitudeMode>absolute</altitudeMode>\n");
		fprintf(fpkml,"<coordinates>\n");
		fprintf(fpkml,"%f,%f,%f\n",lon,lat,alt);
 
		lastpos = ftell(fpkml); // rewind to here to add next co-ordinate
 
		fprintf(fpkml,"</coordinates>\n</LineString>\n</Placemark>\n"); // gets overwritten in subsiquent calls
		kml_state++; // state 2 next time
		break;
 
	case 2: // this section writes an additional co-ordinate to the file and current position placemark
		if((fpkml = fopen(kml_file,"r+")) == NULL) // open exiting file
			return;	// cant open it now - maybe next-time
 
		fseek(fpkml,lastpos,SEEK_SET);
 
		fprintf(fpkml,"%f,%f,%f\n",lon,lat,alt);
 
		lastpos = ftell(fpkml); // rewind to here to add next co-ordinate
 
		fprintf(fpkml,"</coordinates>\n</LineString>\n</Placemark>\n"); // gets overwritten in subsiquent calls
 
		fprintf(fpkml,"<Placemark> <name>Position Now</name> <styleUrl>#place</styleUrl>\n");
		fprintf(fpkml,"<description>%s</description>\n",description);
		fprintf(fpkml,"<Point> <altitudeMode>absolute</altitudeMode>\n");
		fprintf(fpkml,"<coordinates>%f,%f,%f</coordinates>\n",lon,lat,alt);
		fprintf(fpkml,"</Point>\n</Placemark>\n");
		break; // case 2 next time 
	}
 
	// this bit always written 
	fprintf(fpkml,"</Document>\n");
	fprintf(fpkml,"</kml>\n");
 
	fclose(fpkml); // ensure file is written to disk
}
 
 
// read each line from the input
int read_input_line(char *pch, int size)
{
	do
	{
		if (fgets(pch, size, stdin) == NULL)
			return 0;
	}
	while(buf[0] != '$'); // loop until NMEA valid line read
 
	return 1; // line read OK
}
 
 
// calculate a CRC for the line of input
void re_crc(char *pch)
{
	unsigned char crc;
 
	if (*pch != '$') 
		return;		// does not start with '$' - so cant CRC
 
	pch++;			// skip '$'
	crc = 0;
 
	// scan between '$' and '*' (or until CR LF or EOL)
	while ((*pch != '*') && (*pch != '\0') && (*pch != '\r') && (*pch != '\n'))
	{ // checksum calcualtion done over characters between '$' and '*'
		crc ^= *pch;
		pch++;
	}
 
	// re-write (or add checksum)
 
	sprintf(pch,"*%02X\r\n",(unsigned int)crc);
}
 
// Claculate Course & Distance
 
void Calc_Vector(double FromLat, double FromLon, double ToLat, double ToLon, double *pCourse, double *pDistance)
{
	double DeltaLat, DeltaLon;
	double Course;
 
	DeltaLat = ToLat - FromLat;
	DeltaLon = ToLon - FromLon;
 
	Course = atan2(DeltaLat,DeltaLon);	// result is +PI radians (cw) to -PI radians (ccw) from x (Longtitude) axis
	Course = DEGREES(Course);			// convert radians to degrees
	if (Course <= 90.0)
		*pCourse = 90.0 - Course;
	else
		*pCourse = 450.0 - Course;		// convert to 0 - 360 clockwise from north
 
	DeltaLon = cos(RADIANS((FromLat + ToLat) / 2.0)) * DeltaLon; // adjust lontitude to equator equivelent
	*pDistance = (sqrt((DeltaLat * DeltaLat) + (DeltaLon * DeltaLon)) * 111.1949266); // result in Km
}
 
 
// degrees to 16 point compass conversion
// returns a pointer to 3 characters (4 if you want the comma)
// containing the corresponding compass point to the 
// input bearing (0 - 360 degrees relative to north)
 
char points[] = "  N,NNE, NE,ENE,  E,ESE, SE,SSE,  S,SSW, SW,WSW,  W,WNW, NW,NNW,"; 
 
char *deg_to_compass16(double bearing)
{
	int point;
 
	point = (int)((bearing + 11.25) / 22.5); // calculate a compas point 0 - 16 for 0 - 360
	point &= 0x000F; // wrap 16 to 0 
	return &points[point * 4];   
}
 
 
// convert Hours, Minutes & Seconds (in minute) to just decimal seconds
//
double Time_to_Sec(int Hours, int Minutes, double Seconds)
{
	return Seconds + ((double)Minutes * 60.0) + ((double)Hours * 3600.0);
}
 
// convert degress, minutes and directiion into signed decimal degrees
double DegMin_to_Deg(double Deg, double Min, char Dir)
 
{
	Deg += (Min /= 60.0); // convert minutes to degrees
 
	if ((Dir == 'N') || (Dir == 'E'))
		return Deg;
	else
		return -Deg; // Assume 'W' or 'S'
}
 
double next_time = 0.0;
 
int parse_NMEA(char *pch)
{
	int i;
 
	int Hour, Minute;
	double Second;
	double LatDeg, LonDeg;
	double LatMin, LonMin;
	char LatDir, LonDir;
	int NSats;
	double HDOP;
	double Alt;
	char Val;
	char FixQual;
	double SpeedKn;
	double SpeedKph;
	double Course;
	int	Day, Month, Year;
 
	double Distance;
	double Bearing;
 
	i = sscanf(pch,"$GPGGA,%2d%2d%lf,%2lf%lf,%c,%3lf%lf,%c,%c,%d,%lf,%lf,M,",
		&Hour,&Minute,&Second,&LatDeg,&LatMin,&LatDir,&LonDeg,&LonMin,&LonDir,&FixQual,&NSats,&HDOP,&Alt);
 
	if (i > 0)
	{ // some fileds converted
		Second = Time_to_Sec(Hour,Minute,Second);		// convert to decimal seconds
		LatDeg = DegMin_to_Deg(LatDeg,LatMin,LatDir);	// convert to decimal degrees Latitude
		LonDeg = DegMin_to_Deg(LonDeg,LonMin,LonDir);	// convert to decimal degrees Longtitude
 
		if ((Second != 0.0) && (LatDeg != 0.0) && (LonDeg != 0.0))
		{ // valid position
			if(BaseSec == 0.0)
			{
				BaseSec = Second;	// capture first time in file
				BaseLat = LatDeg;	// first Latitude
				BaseLon = LonDeg;	// first Longtitude
				kml_gen(LatDeg,LonDeg,Alt,"Launch!"); // first positions
				next_time = Second + 20.0;
			}
			else
			{
				if (Second >= next_time)
				{
					kml_gen(LatDeg,LonDeg,Alt,"HereNoW!"); // subsiquent positions
					next_time = Second + 20.0;
				}
			}
		}
 
		Calc_Vector(BaseLat, BaseLon, LatDeg, LonDeg, &Bearing, &Distance); // calculate bearin & distance
 
		fprintf(stderr,"\nTi=%4.0f Po=%.4f,%.4f Al=%5.0f Be=%5.1f %.3s Km=%.1f",
			Second - BaseSec,LatDeg,LonDeg,Alt,Bearing,deg_to_compass16(Bearing),Distance);
 
		return GPGGA;
	}
 
	i = sscanf(pch,"$GPRMC,%2d%2d%lf,%c,%2lf%lf,%c,%3lf%lf,%c,%lf,%lf,%2d%2d%2d,",
		&Hour,&Minute,&Second,&Val,&LatDeg,&LatMin,&LatDir,&LonDeg,&LonMin,&LonDir,&SpeedKn,&Course,&Day,&Month,&Year);
 
	if (i > 0)
	{ // some fields converted
		fprintf(stderr," Co=%5.1f %.3s Kh=%.1f",Course,deg_to_compass16(Course),SpeedKn * 1.852);
 
		return GPRMC;
	}
 
	i = sscanf(pch,"$GPVTG,%lf,T,,,%lf,N,%lf,K*", &Course,&SpeedKn,&SpeedKph);
 
	if (i > 0)
	{ // some fileds converted
		return GPVTG;
	}
 
	return NONE;
}
 
// write the line to the output
void write_serial_io(char *pch)
{
		fputs(pch,stdout);
		fflush(stdout);
}
 
// ******************************************************************************************
// *************************************** Main *********************************************
 
 
// lines are played out in pseudo real time - GPGGA messages are held until 1 second has elapsed since the previous
// (re)calcualtes NMEA checksum
 
int main (int argc, char **argv) 
{
	time_t now;			// set to current time
	long gga_count;
	time_t epoch;		// time the output started
 
 
	now = epoch = time(NULL); // cpature the start time
	gga_count = 0l;
 
	kml_gen(0.0,0.0,0.0,""); // create KML file etc. (state 0)
 
	// the main loop
    while (read_input_line(buf, sizeof(buf)))			// Loop until end of file read - read standard input
    {	
		re_crc(buf);					// re-calculate CRC and add
 
		if (parse_NMEA(buf) == GPGGA)	// parse input (and do output messages)
		{	// pace $GPGGA messages to one per second
			gga_count++;
			while ((now - epoch) < gga_count)
				now = time(NULL); // wait until elapsed time in seconds catches up with number of gga messages
		}					
 
		write_serial_io(buf);				// write to standard output
    }
 
	return 0; // normal termination
}