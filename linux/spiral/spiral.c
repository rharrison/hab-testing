#include <stdio.h>
#include <float.h>
#include <math.h>

#define NEWLINE "\n"


int main () {

		FILE *fp;
        double a,t,x,y;
        long i;

		fp=fopen("spiral.kml", "w");
		fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" NEWLINE);
		fprintf(fp, "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" NEWLINE);
		fprintf(fp, "  <Document>" NEWLINE);
		fprintf(fp, "    <name>The Icarus Project - Payload Test KML</name>" NEWLINE);
		fprintf(fp, "    <Style id=\"yellowLineGreenPoly\">" NEWLINE);
		fprintf(fp, "      <LineStyle>" NEWLINE);
		fprintf(fp, "        <color>7f00ffff</color>" NEWLINE);
		fprintf(fp, "        <width>4</width>" NEWLINE);
		fprintf(fp, "      </LineStyle>" NEWLINE);
		fprintf(fp, "      <PolyStyle>" NEWLINE);
		fprintf(fp, "        <color>7f00ff00</color>" NEWLINE);
		fprintf(fp, "      </PolyStyle>" NEWLINE);
		fprintf(fp, "    </Style>" NEWLINE);
		fprintf(fp, "    <Placemark>" NEWLINE);
		fprintf(fp, "      <name>The Icarus Project - Test KML</name>" NEWLINE);
		fprintf(fp, "      <visibility>1</visibility>" NEWLINE);
		fprintf(fp, "      <description>This test file should be used in google earth whilst your payload " NEWLINE);
		fprintf(fp, "sends data using the associated nema or ubx files. Your payload " NEWLINE);
		fprintf(fp, "should follow this track perfectly. If not fix code and test again</description>" NEWLINE);
		fprintf(fp, "      <styleUrl>#yellowLineGreenPoly</styleUrl>" NEWLINE);
		fprintf(fp, "      <LineString>" NEWLINE);
		fprintf(fp, "        <extrude>1</extrude>" NEWLINE);
		fprintf(fp, "        <tessellate>1</tessellate>" NEWLINE);
		fprintf(fp, "        <altitudeMode>absolute</altitudeMode>" NEWLINE);
		fprintf(fp, "        <coordinates>" NEWLINE);

		a=0.001;
        i=0;
		x=0;
		y=0;
		t=0;

        while (x<1 && y<180){
			

                t=i/10.0; // NB use decimal notation or integer division

                x=a*t*cos(t);
                y=a*t*sin(t);

				//fprintf (fp,"a=%f t=%f ",a,t);
                fprintf (fp, "          %f,%f,%ld" NEWLINE,x,y,i*1);
                i++;
				
				//a=a+0.0001;

        }

		
		fprintf(fp, "        </coordinates>" NEWLINE);
		fprintf(fp, "      </LineString>" NEWLINE);
		fprintf(fp, "    </Placemark>" NEWLINE);
		fprintf(fp, "  </Document>" NEWLINE);
		fprintf(fp, "</kml>" NEWLINE);

        return 0;
}
