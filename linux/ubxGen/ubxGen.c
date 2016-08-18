// Read KML on standard in
// Output GPS Ublox UBX protocol to stdout
 
// Assume:- 
//	Ascent rate is linear (5.0 m/sec) 
//	Descent rate is Logarithmic dependant on height (5.0 m/sec at ground level) 
//
// 
// The algorithm works as follows:
//	read a pair of co-ordinates from the KML (each Longitude, Latitude, Altitude)
// 	the first is deemed to be the "from" co-ordinate the next the "to" coordinate
// 	if the "to" coordinate is above the "from" co-ordinate then the flight is deemed to be Ascending
//	if the if the "to" coordinate is below the "from" coordinate then the flight is deemed to be Descending
// 	Estimate the time taken to fly between the two coordinates from the geometric
//	 mean of velocities at "from" and "to" altitudes.
//	using linear interpolation calculate the position and altitude in 1 second steps between "from" and "to"
//	based on the geometric mean velocity.
//
//	When the "to" coordinate is reached :-
//		the "to" coordinate is moved to the "from" co-ordinate
//	 	the next KML co-ordinate is fetched into the "to" co-ordinate
//	the process continues until the current position reaches the last coordinate in the KML 
//
//	Course and direction are calculated between the "from" and "to" co-ordinates and apply 
//  to all samples between the points.
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
 
// radians to degrees
#define DEGREES(x) ((x) * 57.295779513082320877) 
// degrees to radians
#define RADIANS(x) ((x) / 57.295779513082320877)
 
#define LOG_BASE 1.4142135623730950488
#define LOG_POWER 5300.0
 
time_t Now;					// the time of starting this program 
 
char buf[200];
 
static void set_checksum (unsigned char * buffer, int size) ;

void Output_UBX(time_t Time, double Lat, double Lon, double Alt, double Course, double Speed)
{

	unsigned char buffer[100];

	struct tm *ptm;
	
	typedef struct  t_ubx_nav_pvt {
		unsigned short int header;
		unsigned char class;
		unsigned char id;
		unsigned short int length;
		unsigned long iTOW;
		unsigned short int year;
		unsigned char month;
		unsigned char day;
		unsigned char hour;
		unsigned char min;
		unsigned char sec;
		unsigned char valid;
		unsigned long tAcc;
		long nano;
		unsigned char fixType;
		unsigned char flags;
		unsigned char flags2;
		unsigned char numSV;
		long lon;
		long lat;
		long height;
		long hMSL;
		unsigned long hAcc;
		unsigned long vAcc;
		long velN;
		long velE;
		long velD;
		long gSpeed;
		long headMot;
		unsigned long sAcc;
		unsigned long headAcc;
		unsigned short int pDOP;
		unsigned char reserved1 [6];
		long headVeh;
		unsigned char reserved2 [4];
		unsigned char ck_a;
		unsigned char ck_b;
	} t_ubx_nav_pov;
	
	t_ubx_nav_pov ubx_nav_pvt;

	unsigned char c = (unsigned  char) 0xFA;
	
	ptm = gmtime(&Time);
	
	unsigned int position = 0;
	int i = 0;
	
	ubx_nav_pvt.header = (unsigned short int) 0x62 << 8 | 0xB5;
	ubx_nav_pvt.class = 0x01;
	ubx_nav_pvt.id = 0x07;
	ubx_nav_pvt.length = (unsigned short int) 0x00 << 8 | 0x5C;
	ubx_nav_pvt.iTOW = 0x00;
	ubx_nav_pvt.year = (unsigned short int) (1900+ptm->tm_year);
	ubx_nav_pvt.month = (unsigned char) (1+ptm->tm_mon);
	ubx_nav_pvt.day = (unsigned char) (ptm->tm_mday);
	ubx_nav_pvt.hour = (unsigned char) (ptm->tm_hour);
	ubx_nav_pvt.min = (unsigned char) (ptm->tm_min);
	ubx_nav_pvt.sec = (unsigned char) (ptm->tm_sec);
	ubx_nav_pvt.valid = 0b01000111;
	ubx_nav_pvt.tAcc = 0xFF;
	ubx_nav_pvt.nano = 0xFF;
	ubx_nav_pvt.fixType = 0x03;
	ubx_nav_pvt.flags = 0x03;
	ubx_nav_pvt.flags2 = 0x0A;
	ubx_nav_pvt.numSV = 0x0B;
	ubx_nav_pvt.lon = (long)round(Lat*10000000);
	ubx_nav_pvt.lat = (long)round(Lon*10000000);
	ubx_nav_pvt.height = (long)round(Alt*1000); //Stored in mm not m
	ubx_nav_pvt.hMSL = (long)round(Alt*1000);
	ubx_nav_pvt.hAcc = 0x00;
	ubx_nav_pvt.vAcc = 0x00;
	ubx_nav_pvt.velN = 0x00;
	ubx_nav_pvt.velE = 0x00;
	ubx_nav_pvt.velD = 0x00;
	ubx_nav_pvt.gSpeed = (long)round(Speed*1000);
	ubx_nav_pvt.headMot = (long)round(Course*100000);
	ubx_nav_pvt.sAcc = 0xFD;
	ubx_nav_pvt.headAcc = 0xFE;
	ubx_nav_pvt.pDOP = 0xFF;
	
	for (i=0;i<6;i++) {
		memcpy(ubx_nav_pvt.reserved1+i,&c,1);
	}
	
	ubx_nav_pvt.headVeh = (long)round(Course*100000);
	
	for (i=0;i<4;i++) {
		memcpy(ubx_nav_pvt.reserved2+i,&c,1);
	}
	
	fprintf(stderr,"%f, %f, %f, %f, %f, %ld\n",Lat,Lon,Alt,Speed,Course, (long)round(Course*100000));

	memcpy(buffer + position ,&ubx_nav_pvt.header, sizeof(ubx_nav_pvt.header)); position+=sizeof(ubx_nav_pvt.header);
	memcpy(buffer + position ,&ubx_nav_pvt.class, sizeof(ubx_nav_pvt.class)); position+=sizeof(ubx_nav_pvt.class);
	memcpy(buffer + position ,&ubx_nav_pvt.id, sizeof(ubx_nav_pvt.id)); position+=sizeof(ubx_nav_pvt.id);
	memcpy(buffer + position ,&ubx_nav_pvt.length, sizeof(ubx_nav_pvt.length)); position+=sizeof(ubx_nav_pvt.length);
    memcpy(buffer + position ,&ubx_nav_pvt.iTOW, sizeof(ubx_nav_pvt.iTOW)); position+=sizeof(ubx_nav_pvt.iTOW);
	memcpy(buffer + position ,&ubx_nav_pvt.year, sizeof(ubx_nav_pvt.year)); position+=sizeof(ubx_nav_pvt.year);
	memcpy(buffer + position ,&ubx_nav_pvt.month, sizeof(ubx_nav_pvt.month)); position+=sizeof(ubx_nav_pvt.month);
	memcpy(buffer + position ,&ubx_nav_pvt.day, sizeof(ubx_nav_pvt.day)); position+=sizeof(ubx_nav_pvt.day);
	memcpy(buffer + position ,&ubx_nav_pvt.hour, sizeof(ubx_nav_pvt.hour)); position+=sizeof(ubx_nav_pvt.hour);
	memcpy(buffer + position ,&ubx_nav_pvt.min, sizeof(ubx_nav_pvt.min)); position+=sizeof(ubx_nav_pvt.min);
	memcpy(buffer + position ,&ubx_nav_pvt.sec, sizeof(ubx_nav_pvt.sec)); position+=sizeof(ubx_nav_pvt.sec);
	memcpy(buffer + position ,&ubx_nav_pvt.valid, sizeof(ubx_nav_pvt.valid)); position+=sizeof(ubx_nav_pvt.valid);
	memcpy(buffer + position ,&ubx_nav_pvt.tAcc, sizeof(ubx_nav_pvt.tAcc)); position+=sizeof(ubx_nav_pvt.tAcc);
	memcpy(buffer + position ,&ubx_nav_pvt.nano, sizeof(ubx_nav_pvt.nano)); position+=sizeof(ubx_nav_pvt.nano);
	memcpy(buffer + position ,&ubx_nav_pvt.fixType, sizeof(ubx_nav_pvt.fixType)); position+=sizeof(ubx_nav_pvt.fixType);
	memcpy(buffer + position ,&ubx_nav_pvt.flags, sizeof(ubx_nav_pvt.flags)); position+=sizeof(ubx_nav_pvt.flags);
	memcpy(buffer + position ,&ubx_nav_pvt.flags2, sizeof(ubx_nav_pvt.flags2)); position+=sizeof(ubx_nav_pvt.flags2);
	memcpy(buffer + position ,&ubx_nav_pvt.numSV, sizeof(ubx_nav_pvt.numSV)); position+=sizeof(ubx_nav_pvt.numSV);
	memcpy(buffer + position ,&ubx_nav_pvt.lon, sizeof(ubx_nav_pvt.lon)); position+=sizeof(ubx_nav_pvt.lon);
	memcpy(buffer + position ,&ubx_nav_pvt.lat, sizeof(ubx_nav_pvt.lat)); position+=sizeof(ubx_nav_pvt.lat);
	memcpy(buffer + position ,&ubx_nav_pvt.height, sizeof(ubx_nav_pvt.height)); position+=sizeof(ubx_nav_pvt.height);
	memcpy(buffer + position ,&ubx_nav_pvt.hMSL, sizeof(ubx_nav_pvt.hMSL)); position+=sizeof(ubx_nav_pvt.hMSL);
	memcpy(buffer + position ,&ubx_nav_pvt.hAcc, sizeof(ubx_nav_pvt.hAcc)); position+=sizeof(ubx_nav_pvt.hAcc);
	memcpy(buffer + position ,&ubx_nav_pvt.vAcc, sizeof(ubx_nav_pvt.vAcc)); position+=sizeof(ubx_nav_pvt.vAcc);
	memcpy(buffer + position ,&ubx_nav_pvt.velN, sizeof(ubx_nav_pvt.velN)); position+=sizeof(ubx_nav_pvt.velN);
	memcpy(buffer + position ,&ubx_nav_pvt.velE, sizeof(ubx_nav_pvt.velE)); position+=sizeof(ubx_nav_pvt.velE);
	memcpy(buffer + position ,&ubx_nav_pvt.velD, sizeof(ubx_nav_pvt.velD)); position+=sizeof(ubx_nav_pvt.velD);
	memcpy(buffer + position ,&ubx_nav_pvt.gSpeed, sizeof(ubx_nav_pvt.gSpeed)); position+=sizeof(ubx_nav_pvt.gSpeed);
	memcpy(buffer + position ,&ubx_nav_pvt.headMot, sizeof(ubx_nav_pvt.headMot)); position+=sizeof(ubx_nav_pvt.headMot);
	memcpy(buffer + position ,&ubx_nav_pvt.sAcc, sizeof(ubx_nav_pvt.sAcc)); position+=sizeof(ubx_nav_pvt.sAcc);
	memcpy(buffer + position ,&ubx_nav_pvt.headAcc, sizeof(ubx_nav_pvt.headAcc)); position+=sizeof(ubx_nav_pvt.headAcc);
	memcpy(buffer + position ,&ubx_nav_pvt.pDOP, sizeof(ubx_nav_pvt.pDOP)); position+=sizeof(ubx_nav_pvt.pDOP);
	memcpy(buffer + position ,&ubx_nav_pvt.reserved1, sizeof(ubx_nav_pvt.reserved1)); position+=sizeof(ubx_nav_pvt.reserved1);
	memcpy(buffer + position ,&ubx_nav_pvt.headVeh, sizeof(ubx_nav_pvt.headVeh)); position+=sizeof(ubx_nav_pvt.headVeh);
	memcpy(buffer + position ,&ubx_nav_pvt.reserved2, sizeof(ubx_nav_pvt.reserved2)); position+=sizeof(ubx_nav_pvt.reserved2);

/*
	fprintf(stderr,"\n");
	
	for (i=0;i<100;i++) {
		fprintf(stderr,"%02x ",buffer[i]);
		if (i%10==9) {fprintf(stderr,"\n");}
	} 
	*/
	set_checksum (buffer,100);
	
	FILE * fp;
	
	if ( !(fp = fopen("ubx.bin","ab"))) {
		fprintf(stderr,"\nErorr\n");
		exit(-1);
	}
	
	for (i=0;i<100;i++) {
		fputc (buffer[i], fp);
	}		
	
	fclose(fp);
}
 
 
 static void set_checksum (unsigned char * buffer, int size) {
	
	/* This will put the calculated checksum on the end of your message
	 * Make sure you have left two bytes at the end of your message
	 * for the checksum !
	 */
	
	unsigned char CK_A = 0;
	unsigned char CK_B = 0;
	
	int i;
	
	for( i=2; i<size-2; i++)
	{
		CK_A = CK_A + buffer[i];
		CK_B = CK_B + CK_A;
	}
	
	buffer[i++] = CK_A;
	buffer[i++] = CK_B;
	
}
 
// do a KML segment between two coordinates
 
void do_segment(double FromLat, double FromLon, double FromAlt, double ToLat, double ToLon, double ToAlt)
{
	double Rate;			// ascent(+ve) / decent(-ve) rate meters per second (for segment)
	int Duration;			// calculated segment duration in seconds
	double Elapsed;			// floating point duration
	double Lat,Lon,Alt;		// clacualted for each step
	double	Course,Speed;	// calculated for entire segment
	double Distance;		// distance between coordinates
	double DeltaLat,DeltaLon,DeltaAlt;	// Latitude, Longtitude and Altitude differences
	double AdjLon;			// Adjusted Longtitude difference
	int i;					// counter
 
	DeltaAlt = (ToAlt - FromAlt);
	DeltaLat = (ToLat - FromLat);
	DeltaLon = (ToLon - FromLon);
 
	if (DeltaAlt >= 0)
	{ // ascending
		Rate = 5.0;
	}
	else 
	{ // descending - calculate the geometric mean of the expected velocities at To and From altitude
		Rate = -5.0 * sqrt(pow(LOG_BASE,(FromAlt / LOG_POWER)) * pow(LOG_BASE,(ToAlt / LOG_POWER))); 
	}
 
	// calcualte time (secs) between co-ordinates
	Elapsed = DeltaAlt / Rate; // always positive
	Duration = (int)Elapsed;

	/* RJH FIX tfor tight points to 1 second if les */
	if (Duration==0) {Duration = 1;}
	
	if ((Elapsed - (float)Duration) >= 0.5)
		Duration++; // round duration of segment to nearest integer
 
	// Calculate Course (degrees relative to north) for entire segment
	Course = atan2(DeltaLat,DeltaLon);	// result is +PI radians (cw) to -PI radians (ccw) from x (Longtitude) axis
 
	Course = DEGREES(Course);			// convert radians to degrees
	if (Course <= 90.0)
		Course = 90.0 - Course;
	else
		Course = 450.0 - Course;		// convert to 0 - 360 clockwise from north
 
	// Calculate Speed (m/sec) for entire segment
	AdjLon = cos(RADIANS((FromLat + ToLat) / 2.0)) * DeltaLon;
	Distance = (sqrt((DeltaLat * DeltaLat) + (AdjLon * AdjLon)) * 111194.9266); // result in meters
 
	Speed =  Distance / (double)Duration; // meters per second
 
	// calculate 1 second "step" sizes
	DeltaAlt /= (double)Duration;
	DeltaLat /= (double)Duration;
	DeltaLon /= (double)Duration;
 
	// now output the NMEA for each step between From and To (but excluding To - which is picked up on next segment)
	Lat = FromLat;
	Lon = FromLon;
	Alt = FromAlt;
	for (i = 0; i < Duration; i++)
	{
		Output_UBX(Now,Lat,Lon,Alt,Course,Speed);
		Now++;				// 1 second steps
		Lat += DeltaLat;
		Lon += DeltaLon;
		Alt += DeltaAlt;	
	}
}
 
// look for a particular string in the input 
// error if not found in input
//
 
void look_for(char *string)
 
{
	int i;
 
	while(1)
	{
		i = scanf("%128s",buf); // read a non whitespace from the input
		if (i != 1)
		{
			fprintf(stderr,"No %s found\n",string);
			exit(1); // abnormal termination
		}
		if (strcmp(buf,string) == 0)
		{
			return; // string found
		}
 
	}
}
 
 
// Read in .KML file (extract co-ordinate part)
// Make assumptions about Ascent and Decent Rates
// interpolate positions
// Output GPS in pseudo real time
 
 
int main(int argc, char **argv)
{
	int i;
	float FromLon,FromLat,FromAlt, ToLon,ToLat,ToAlt;
 
	look_for("<LineString>"); // look for 1st <LineString> token
 
	look_for("<coordinates>"); // look for subsiquent <coordinates> token
 
	FromLon = FromLat = FromAlt = ToLon = ToLat = ToAlt = 0.0;
 
	// get first LineString co-ordinate as launch position
	i = scanf("%f , %f , %f",&FromLon,&FromLat,&FromAlt);
	if (i !=3)
	{
		fprintf(stderr,"1st coordinate not 3D\n");
		return 1; // abnormal termination
	}
 
	Now = time(NULL); // use the current time as a reference
	// get subsiquent LineString co-ordinates
	
	int j =0;

	fprintf(stderr,"Processing KML coordinates  ");

	while(1)
	{
		
		fputc('.',stderr);

		i = scanf("%f , %f , %f",&ToLon,&ToLat,&ToAlt);
 
		if (i != 3) {
			break; // not co-ordinate
		}
		
		do_segment(FromLat,FromLon,FromAlt,ToLat,ToLon,ToAlt);
 
		FromLon = ToLon;
		FromLat = ToLat;
		FromAlt = ToAlt;
		j++;
	}

	Output_UBX(Now,ToLat,ToLon,ToAlt,0.0,0.0); // Final Position (assume stationary)
 
	look_for("</coordinates>"); // look for closing </coordinates> token
 
	look_for("</LineString>"); // look for closing </LineString> token

	fprintf(stderr,"\n");
 
	return 0;
}