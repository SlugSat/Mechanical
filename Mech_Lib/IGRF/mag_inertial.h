#include <stdio.h>
#include <stdlib.h>            
#include <string.h>
#include <ctype.h>
#include <math.h> 

/*
typedef struct
{
	float gh[MAXCOEF];//harmonic coefficients
	float gh1[MAXCOEF];
	float gh2[MAXCOEF];
	float north; // x
	float east;  // y
	float down;  // z
	float declination;
	float inclination;
	float horizontal;
	float total;
}IGRF;
*/

/*
* Input: takes in an ACS struct
* note: void input as placeholer
*
* void get_mag_inertial(ACStype* acs);
*/
void get_mag_inertial(void);
