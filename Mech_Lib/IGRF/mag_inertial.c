/*
Author: Gabriel Barbosa

Reference: All functions are pulled from geomag70.c found here:
	https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html
	
Date: 4/29/2019
*/

/*********INCLUDES**********/
#include "mag_inertial.h"
#include "IGRF12.h"
#include <stdio.h>
#include <stdlib.h>            
#include <string.h>
#include <ctype.h>
#include <math.h>


/*********DEFINES**********/
#define NaN log(-1.0)
#define FT2KM (1.0/0.0003048)
#define PI 3.141592654
#define RAD2DEG (180.0/PI)
#define MAXCOEF 210
#define EARTH_RADIUS 6371.2
#define DTR 0.01745329
#define RECL 81

#define MAXINBUFF RECL+14
/** Max size of in buffer **/

#define MAXREAD MAXINBUFF-2
/** Max to read 2 less than total size (just to be safe) **/

/**********TEST**********/
// #define IGRF_TEST


/*******************STRUCT************/
typedef struct
{
	float gh[MAXCOEF];//harmonic coefficients
	float north; // x
	float east;  // y
	float down;  // z
	float declination;
	float inclination;
	float horizontal;
	float total;
}IGRF;

/*********PRIVATE FUNCTIONS**********/
int extrapsh(IGRF* igrf,double date);
void shva13(IGRF* igrf, double lat, double lon, double alt, int nmax);
void dihf (IGRF* igrf);
double JD_2_decdate(double JD);



/***********PUBLIC FUNCTIONS**********/
void get_mag_inertial(double JD, float longitude, float latitude, float altitude, Matrix mag_NED)
{
	IGRF igrf;
	int nmax;
	
	double date = JD_2_decdate(JD);
	
	nmax = extrapsh(&igrf,date);
	
	shva13(&igrf, latitude, longitude, altitude/1000.0 + EARTH_RADIUS, nmax);
	
	dihf(&igrf);
	
	vectorSetXYZ(mag_NED, igrf.north, igrf.east, igrf.down);
	
	#ifdef IGRF_TEST
	printf("\nNorth: %f\nEast: %f\nDown: %f\n", igrf.north, igrf.east, igrf.down);
	 
	printf("\nDeclination: %f\nInclination: %f\nHorizontal: %f\nTotal: %f\n", igrf.declination*RAD2DEG, igrf.inclination*RAD2DEG, igrf.horizontal, igrf.total);
	#endif
	return;
}

/**************************************/


/*
funciton: Extrapolates linearly a spherical harmonic model with a
			rate-of-change model. 
			
input: 
	-IGRF struct 
	-date : the decimal year of the date for resulting model

output: 
	-updates gh[] in IGRF struct: Schmidt quasi-normal internal
									spherical harmonic coeffs.
	-nmax: maximum degree and order of resulting model
*/
int extrapsh(IGRF* igrf,double date)
{
	int nmax;
	int k, l;
	int ii;
	double factor;
	
	int nmax1 = 13; //number of Orders in 2015 model
	int nmax2 = 8;  //number of Degres in 2105 model
	double dte1 = 2015;
	
	factor = date - dte1;
	k = nmax2 * (nmax2 + 2);
	l = nmax1 * (nmax1 + 2);
	for(ii = k+1; ii <= l; ++ii)
	{
		igrf->gh[ii] = gh1[ii];
	}
	nmax = nmax1;
	for (ii = 1; ii <= k; ++ii)
	{
		igrf->gh[ii] = gh1[ii] + factor*gh2[ii];
	}
	return nmax;
}

/*
funciton: Calculates fiel components from spherical harmonic (sh)
			models. 
			
input: 
	-IGRF struct 
	-lat: north latitude, in degrees
	-lon: east longitude, in degrees
	-alt: altitude (radial distance from center of earth in km)
	-nmax: maximum degree and order of resulting model
	
output: 
	-updates IGRF struct:
		igrf->north: northward component (x)
		igrf->east: eastward component (y)
		igrf->down: verticle/downward component (z)
		
*/

void shva13(IGRF* igrf, double lat, double lon, double alt, int nmax)
{
	double argument;
	double slat, clat;//sin(lat) and cos(lat)
	double sd, cd;
	double sl[14];
	double cl[14];
	double p[119];
 	double q[119];
	double ratio;
	double power;
	double rr;
	double fn, fm;
	double aa, bb, cc;
	int ii,j,k,l,m,n;
	int npq;
	
	argument = lat * DTR;
	slat = sin( argument );
	
	if ((90.0 - lat) < 0.001)
	{
		aa = 89.999; // 300ft from North pole
	}
	else 
	{
		if ((90.0 + lat) < 0.001)
		{
			aa = -89.999; // 300ft from South pole
		}
		else
		{
			aa = lat;
		}
	}
	argument = aa * DTR;
	clat = cos( argument );
	argument = lon * DTR;
	sl[1] = sin( argument );
	cl[1] = cos( argument );
	igrf->north = 0;
	igrf->east = 0;
	igrf->down = 0;
	
	sd = 0.0;
	cd = 1.0;
	l = 1;
	n = 0;
	m = 1;
	npq = (nmax * (nmax + 3)) / 2;
	
	ratio = EARTH_RADIUS/alt;
	argument = 3.0;
	aa = sqrt( argument );
	p[1] = 2.0 * slat;
	p[2] = 2.0 * clat;
	p[3] = 4.5 * slat * slat - 1.5;
	p[4] = 3.0 * aa * clat * slat;
	q[1] = -clat;
	q[2] = slat;
	q[3] = -3.0 * clat * slat;
	q[4] = aa * (slat * slat - clat * clat);
	
	for (k = 1; k <= npq; ++k)
	{
		if (n < m)
		{
			m = 0;
			n = n + 1;
			argument = ratio;
			power =  n + 2;
			rr = pow(argument,power);
			fn = n;
		}
		fm = m;
		if (k >= 5)
		{
			if (m == n)
			{
				argument = (1.0 - 0.5/fm);
              	aa = sqrt( argument );
				j = k - n - 1;
				p[k] = (1.0 + 1.0/fm) * aa * clat * p[j];
				q[k] = aa * (clat * q[j] + slat/fm * p[j]);
				sl[m] = sl[m-1] * cl[1] + cl[m-1] * sl[1];
				cl[m] = cl[m-1] * cl[1] - sl[m-1] * sl[1];
			}
			else
			{
				argument = fn*fn - fm*fm;
				aa = sqrt( argument );
				argument = ((fn - 1.0)*(fn-1.0)) - (fm * fm);
				bb = sqrt( argument )/aa;
				cc = (2.0 * fn - 1.0)/aa;
				ii = k - n;
				j = k - 2 * n + 1;
				p[k] = (fn + 1.0) * (cc * slat/fn * p[ii] - bb/(fn - 1.0) * p[j]);
				q[k] = cc * (slat * q[ii] - clat/fn * p[ii]) - bb * q[j];
			}
		}
		aa = rr * igrf->gh[l];
		if (m == 0)
		{
			igrf->north = igrf->north + aa * q[k];
			igrf->down = igrf->down - aa * p[k];
			l += 1;
		}
		else
		{
			bb = rr * igrf->gh[l+1];
			cc = aa * cl[m] + bb * sl[m];
			igrf->north = igrf->north + cc * q[k];
			igrf->down = igrf->down - cc * p[k];
			if (clat > 0)
			{
				igrf->east = igrf->east + (aa * sl[m] - bb * cl[m]) * fm * p[k]/((fn + 1.0) * clat);
			}
			else
			{
				igrf->east = igrf->east + (aa * sl[m] - bb * cl[m]) * q[k] * slat;
			}
			l += 2;
		}
		m += 1;
	}
	aa = igrf->north;
	igrf->north = igrf->north * cd + igrf->down * sd;
	igrf->down = igrf->down * cd - aa * sd;
	return;
}

/*
funciton: Calculates field components from spherical harmonic (sh)
			models. 
			
input: 
	-IGRF struct 
		igrf->north: northward component (x)
		igrf->east: eastward component (y)
		igrf->down: verticle/downward component (z)
output: 
	-updates IGRF struct:
		igrf->declination: declination angle in degrees
		igrf->inclination: inclination angle in degrees
		igrf->horizontal: horizontal mag intensity in nT
		igrf->total: magnitude of the total mag intensity in nT
		
NOTE: MAY NEED TO UPDATE SO THAT VALUES GOES INTO ACS STRUCT
*/

void dihf (IGRF* igrf)
{
	int j;
	double sn;
	double h2;
	double hpx;
	double argument, argument2;

	sn = 0.0001;

	for (j = 1; j <= 1; ++j)
	{
	  h2 = igrf->north*igrf->north + igrf->east*igrf->east;
	  argument = h2;
	  igrf->horizontal = sqrt(argument);       /*calculate horizontal intensity */
	  argument = h2 + igrf->down*igrf->down;
	  igrf->total = sqrt(argument);      /*calculate total intensity */
	  if (igrf->total < sn)
		{
		  igrf->declination = NaN;       /* If d and i cannot be determined, */
		  igrf->inclination = NaN;              /*set equal to NaN         */
		}
	  else
		{
		  argument = igrf->down;
		  argument2 = igrf->horizontal;
		  igrf->inclination = atan2(argument,argument2);
		  if (igrf->horizontal < sn)
		    {
		      igrf->declination = NaN;
		    }
		  else
		    {
		      hpx = igrf->horizontal + igrf->north;
		      if (hpx < sn)
		        {
		          igrf->declination = PI;
		        }
		      else
		        {
		          argument = igrf->east;
		          argument2 = hpx;
		          igrf->declination = 2.0 * atan2(argument,argument2);
		        }
		    }
		}
	}

	return;
}



double JD_2_decdate(double JD)
{
	JD = JD - 2451545 + 0.5;
	int year = 2000;
	int days_per_year;
	while(1) {
		if(((year % 4) == 0) && (((year % 100) != 0) || ((year % 400) == 0))) {
			days_per_year = 366; // Leap year
		}
		else {
			days_per_year = 365;
		}
		if (JD < days_per_year)
		{
			break;
		}
		
		year++;
		JD -= days_per_year;
	}
	
	return (double)year + JD/days_per_year;
}

#ifdef IGRF_TEST

int main(void)
{
	// These should come from ACS struct
	double JD = 2458608.30832;
	
	float longitude = 0;
	float latitude = 0;
	float altitude = 6820.0;	
	
	Matrix igrf_ned = newMatrix(3,1);
	
	get_mag_inertial(JD, longitude, latitude, altitude, igrf_ned);

	return 0;
}

#endif
