// Read KML on standard in
// Output GPS simulation to standard out
 
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
 
char buf[128];
 
// calculate a CRC for the line of input
void do_crc(char *pch)
{
	unsigned char crc;
 
	if (*pch != '$') 
		return;		// does not start with '$' - so can't CRC
 
	pch++;			// skip '$'
	crc = 0;
 
	// scan between '$' and '*' (or until CR LF or EOL)
	while ((*pch != '*') && (*pch != '\0') && (*pch != '\r') && (*pch != '\n'))
	{ // checksum calcualtion done over characters between '$' and '*'
		crc ^= *pch;
		pch++;
	}
 
	// add or re-write checksum
 
	sprintf(pch,"*%02X\r\n",(unsigned int)crc);
}
 
 
// Speed in Kph 
// Course over ground relative to North
//
// In normal operation the GPS emulated sends the following sequence of messages
// $GPGGA, $GPRMC, $GPVTG
// $GPGGA, $GPGSA, $GPRMC, $GPVTG
// $GPGGA, $GPGSV  ... $GPGSV, $GPRMC, $GPVTG
 
 
// Output_NEMA - Output the position in NMEA
// Latitude & Longtitude in degrees, Altitude in meters Speed in meters/sec
 
void Output_NEMA(time_t Time, double Lat, double Lon, double Alt, double Course, double Speed)
{
	int LatDeg;					// latitude - degree part
	double LatMin;				// latitude - minute part
	char LatDir;				// latitude - direction N/S
	int LonDeg;					// longtitude - degree part
	double LonMin;				// longtitude - minute part
	char LonDir;				// longtitude - direction E/W
	struct tm *ptm;
 
	ptm = gmtime(&Time);
 
	if (Lat >= 0)
		LatDir = 'N';
	else
		LatDir = 'S';
 
	Lat = fabs(Lat);
	LatDeg = (int)(Lat);
	LatMin = (Lat - (double)LatDeg) * 60.0;
 
	if (Lon >= 0)
		LonDir = 'E';
	else
		LonDir = 'W';
 
	Lon = fabs(Lon);
	LonDeg = (int)(Lon);
	LonMin = (Lon - (double)LonDeg) * 60.0;
 
	// $GPGGA - 1st in epoc - 5 satellites in view, FixQual = 1, 45m Geoidal separation HDOP = 2.4
	sprintf(buf,"$GPGGA,%02d%02d%02d.000,%02d%07.4f,%c,%03d%07.4f,%c,1,05,02.4,%.1f,M,45.0,M,,*",
		ptm->tm_hour,ptm->tm_min,ptm->tm_sec,LatDeg,LatMin,LatDir,LonDeg,LonMin,LonDir,Alt);
	do_crc(buf); // add CRC to buf
	fputs(buf,stdout);
 
 
	switch((int)Time % 3)
	{ // include 'none' or $GPGSA or $GPGSV in 3 second cycle
	case 0:
		break;
 
	case 1:
		// 3D fix - 5 satellites (3,7,18,19 & 22) in view. PDOP = 3.3,HDOP = 2.4, VDOP = 2.3
		sprintf(buf,"$GPGSA,A,3,03,07,18,19,22,,,,,,,,3.3,2.4,2.3*");
		do_crc(buf); // add CRC to buf
		fputs(buf,stdout);
		break;
 
	case 2:
		// two lines og GPGSV messages - 1st line of 2, 8 satellites being tracked in total
		// 03,07 in view 11,12 being tracked
		sprintf(buf,"$GPGSV,2,1,08,03,89,276,30,07,63,181,22,11,,,,12,,,*");
		do_crc(buf); // add CRC to buf
		fputs(buf,stdout);
 
		// GPGSV 2nd line of 2, 8 satellites being tracked in total
		// 18,19,22 in view 27 being tracked
		sprintf(buf,"$GPGSV,2.2,08,18,73,111,35,19,33,057,27,22,57,173,37,27,,,*");
		do_crc(buf); // add CRC to buf
		fputs(buf,stdout);
		break;
	}
 
	//$GPRMC
	sprintf(buf,"$GPRMC,%02d%02d%02d.000,A,%02d%07.4f,%c,%03d%07.4f,%c,%.2f,%.2f,%02d%02d%02d,,,A*",
		ptm->tm_hour,ptm->tm_min,ptm->tm_sec,LatDeg,LatMin,LatDir,LonDeg,LonMin,LonDir,Speed * 1.943844,Course,ptm->tm_mday,ptm->tm_mon + 1,ptm->tm_year % 100);
	do_crc(buf); // add CRC to buf
	fputs(buf,stdout);
 
	// $GPVTG message last in epoc
	sprintf(buf,"$GPVTG,%.2f,T,,,%.2f,N,%.2f,K,A*",Course,Speed * 1.943844,Speed * 3.6);
	do_crc(buf); // add CRC to buf
	fputs(buf,stdout);
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
	Duration = 1;
	fprintf(stderr,"Duration %d",Duration);
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
		Output_NEMA(Now,Lat,Lon,Alt,Course,Speed);
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
			fprintf(stderr,"%s found\n",string);
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

	while(1==1)
	{
		
		fprintf(stderr,"processing %d\n",j);

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
 fprintf(stderr,"hello5\n");
	Output_NEMA(Now,ToLat,ToLon,ToAlt,0.0,0.0); // Final Position (assume stationary)
 
	look_for("</coordinates>"); // look for closing </coordinates> token
 
	look_for("</LineString>"); // look for closing </LineString> token
 
	return 0;
}