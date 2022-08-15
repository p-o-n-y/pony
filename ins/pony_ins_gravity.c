// Aug-2022
/*	pony_ins_gravity 
	
	pony plugins for gravity model calculations:

	- pony_ins_gravity_constant 
		Takes the magnitude of average accelerometer output vector 
		and puts it into vertical gravity component, which then remains constant.
		Performs calculations for each initialized IMU device.
		Recommended for low-grade systems, especially when no reference coordinates available.

	- pony_ins_gravity_normal
		Computes conventional Earth normal gravity model as in GRS80, etc.,
		but takes Earth model constants from pony->imu_const variables.
		Accounts for both latitude and altitude, as well as for plumb line curvature above ellipsoid.
		Performs calculations for each initialized IMU device.
		Recommended for conventional navigation grade systems.

	- (planned) pony_ins_gravity_egm08
		Planned for future development.
	
*/


#include <stdlib.h>
#include <math.h>

#include "../pony.h"


// pony bus version check
#define PONY_INS_GRAVITY_BUS_VERSION_REQUIRED 18
#if PONY_BUS_VERSION < PONY_INS_GRAVITY_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif


// service functions
	// memory handling
void pony_ins_gravity_free_null(void** ptr);


/* pony_ins_gravity_constant - pony plugin
	
	Takes the magnitude of average accelerometer output vector 
	and puts it into vertical gravity component, which then remains constant.
	Performs calculations for each initialized IMU device.
	Recommended for low-grade systems, especially when no reference coordinates available.

	description:
		    [   0  ]
		g = [   0  ] = const,
		    [-|<f>|]
		where <f> is the specific force vector as measured by accelerometers
		averaged over an initial alignment period specified in the configuration string
	
	uses:
		pony->imu->t
		pony->imu->f
		pony->imu->f_valid

	changes:
		pony->imu->g
		pony->imu->g_valid

	cfg parameters:
		alignment
		or
		{imu: alignment} - imu initial alignment duration, sec
			type :   floating point
			range:   0+ to +inf (positive decimal)
			default: 60
			example: {imu: alignment = 300}
			note:    if specified for the particular device, the value overrides one from the common configuration settings (outside of groups)
*/
void pony_ins_gravity_constant(void) {

	const char   t0_token[] = "alignment"; // alignment duration parameter name in configuration
	const double t0_default = 60;          // default alignment duration

	static size_t ndev;    // number of IMU devices
	static double
		(*f0)[3]   = NULL, // average specific force vector measured by accelerometers for each device
		 *g3       = NULL, // its norm                                                 for each device
		 *t0       = NULL; // imu initial alignment duration                           for each device
	static unsigned long 
		 *n        = NULL; // number of measurements used so far                       for each device

	char   *cfg_ptr;       // pointer to a substring in configuration
	double  n1_n;          // (n - 1)/n
	size_t  i, dev;        // indices


	// validate IMU subsystem
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// allocate memory
		ndev = pony->imu_count;
		f0 = (double       (*)[3])calloc(ndev, sizeof(double[3]    ));
		g3 = (double        *    )calloc(ndev, sizeof(double       ));
		t0 = (double        *    )calloc(ndev, sizeof(double       ));
		n  = (unsigned long *    )calloc(ndev, sizeof(unsigned long));
		if (f0 == NULL || g3 == NULL || t0 == NULL || n == NULL) {
			pony->mode = -1;
			return;
		}
		// init for all devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// initial values
			n [dev] =  0;
			t0[dev] = -1;
			// zero average specific force
			for (i = 0; i < 3; i++)
				f0[dev][i] = 0;
			// parse alignment duration from configuration string, either from specific device configuration
			cfg_ptr = pony_locate_token(t0_token, pony->imu[dev].cfg, pony->imu[dev].cfglength, '=');
			// or from common settings
			if (cfg_ptr == NULL)
				cfg_ptr = pony_locate_token(t0_token, pony->cfg_settings, pony->settings_length, '=');
			// try parsing
			if (cfg_ptr != NULL)
				t0[dev] = atof(cfg_ptr);
			// if not found or invalid, set to default
			if (cfg_ptr == NULL || t0[dev] <= 0)
				t0[dev] = t0_default;
		}

	}

	else if (pony->mode < 0) {	// terminate
		
		// free allocated memory
		pony_ins_gravity_free_null((void**)(&f0));
		pony_ins_gravity_free_null((void**)(&g3));
		pony_ins_gravity_free_null((void**)(&t0));
		pony_ins_gravity_free_null((void**)(&n ));

	}

	else						// main cycle
	{
		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// validity flag down
			pony->imu[dev].g_valid = 0;

			if (pony->imu[dev].t <= t0[dev] && pony->imu[dev].f_valid) { // while alignment goes on, and accelerometer measurements are available
				n[dev]++;
				n1_n = (n[dev] - 1.0)/n[dev];
				for (i = 0; i < 3; i++)
					f0[dev][i] = f0[dev][i]*n1_n + pony->imu[dev].f[i]/n[dev]; // averaging components
				g3[dev] = -pony_linal_vnorm(f0[dev],3);                   // renew magnitude, negative
			}
			// zero Eastern and Northern components
			pony->imu[dev].g[0] = 0;
			pony->imu[dev].g[1] = 0;
			// vertical component, negative magnitude of average measured specific force vector
			pony->imu[dev].g[2] = g3[dev];
			// validity flag up
			pony->imu[dev].g_valid = 1;
		}

	}

}

/* pony_ins_gravity_normal - pony plugin
	
	Computes conventional Earth normal gravity model as in GRS80, etc.,
	but takes Earth model constants from pony->imu_const variables.
	Accounts for both latitude and altitude, as well as for plumb line curvature above ellipsoid.
	Performs calculations for each initialized IMU device.
	Recommended for conventional navigation grade systems.

	description:
		if pony->imu->sol.llh coordinates are valid, takes them as reference point to calculate gravity at
		else if pony->sol (hybrid solution) coordinates are valid, uses them
		otherwise takes latitude of pi/4 and zero altitude as reference point

		g[0] = gE = 0
		g[1] = gN = -fg*sin(2*lat)*h/a
		g[2] = gU = -ge*[1 + fg*sin(lat)^2 - f4/4*sin(2*lat)^2]*[1 - 2*(1 + f*cos(2*lat) + m)*h/a]

		where

		- lat   is  geographical latitude
		- h     is  geographical altitude above Earth ellipsoid
		- a     is  Earth ellipsoid semimajor axis
		- fg    is  Earth gravity flattening
		- ge    is  Earth gravity at equator
		- f     is  Earth ellipsoid flattening
		- m, f4 are Earth gravity model auxiliary constants
	
	uses:
		pony->imu->sol.llh
		pony->imu->sol.llh_valid
		pony->sol.llh
		pony->sol.llh_valid

	changes:
		pony->imu->g
		pony->imu->g_valid

	cfg parameters:
		none
*/
void pony_ins_gravity_normal(void) {

	static size_t ndev; // number of IMU devices
	static double
		lat,	        // geographical latitude		
		h,		        // geographical altitude from reference ellipsoid
		f    = 0,       // Earth ellipsoid flattening f = (a - b)/a
		m    = 0,       // gravitational parameter, m = [u^2 a^2 b]/[GM], ratio between centrifugal and gravitational accelerations on the equator of a shpere having the same mass and volume as the Earth does
		f4_4 = 0;       // coefficient for the second harmonic term, f4/4 = 5/2 f m - 1/2 f^2

	double 
		sinlat,	        //   sine of latitude
		sin2lat,        //   sine of twofold latitude
		cos2lat,        // cosine of twofold latitude
		h_a,	        // ratio between altitude and ellipsoid semimajor axis
		b_a;	        // ratio between Earth ellipsoid semiminor and semimajor axes, b/a = sqrt(1 - e^2)
	size_t dev;         // current IMU device


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init
		
		ndev = pony->imu_count;
		// ratio between Earth ellipsoid semiminor and semimajor axes
		b_a = sqrt(1 - pony->imu_const.e2); // b/a = sqrt(1 - e^2)
		// Earth ellipsoid flattening 
		f = 1 - b_a; // as from definitions: e^2 = (a^2 - b^2)/a^2, f = (a - b)/a
		// gravitational parameter, as derived from section 3 of Geodetic Reference System 80 by H. Moritz (GRS-80): 
			// from ge = GM/ab (1 - m - m/6 e'q0'/q0), 
			// and  gp = GM/a^2 (1 + m/3 e'q0'/q0), 
			// and  f + fg = u^2 b / ge (1 + e'/2 q0'/q0)
			// let  k   = 1/2 e'q0'/q0 = (f + fg)/(u^2 b) ge - 1, 
			// let  eps = 1 / (gp/ge*a/b - 1) = 1/((fg + 1)/sqrt(1 - e^2) - 1), 
			// then m   = 1/(k*(eps + 1/3) + eps + 1)
		f4_4 = (f + pony->imu_const.fg)/(pony->imu_const.u*pony->imu_const.u*pony->imu_const.a*b_a)*pony->imu_const.ge - 1; // use f4_4 variable instead of k
		m    = 1/((pony->imu_const.fg + 1)/b_a - 1);
		m    = 1/(f4_4*(m+1.0/3) + m + 1);
		// coefficient for the second harmonic term
		f4_4 = f/8*(5*m - f); // f4 = -1/2 f^2 + 5/2 f m, as in (2-115), Physical Geodesy, W.Heiskanen H.Moritz, 1993, p. 76, or section 3 of Geodetic Reference System 80 by H. Moritz (GRS-80), corrected for skipped minus sign
		// default values
		lat = pony->imu_const.pi/4; // mean latitude magnitude
		h   = 0;
	}

	else if (pony->mode < 0) {	// terminate
		// do nothing
	}

	else						// main cycle
	{
		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// validity flag down
			pony->imu[dev].g_valid = 0;
			// define latitude and altitude
			if (pony->imu[dev].sol.llh_valid) { // from imu data, if valid
				lat = pony->imu[dev].sol.llh[1];
				h   = pony->imu[dev].sol.llh[2];
			}
			else if (pony->sol.llh_valid) { // from hybrid solution, if valid
				lat = pony->sol.llh[1];
				h   = pony->sol.llh[2];
			}
			sinlat  = sin(  lat);
			sin2lat = sin(2*lat);
			cos2lat = cos(2*lat);
			h_a = h/(pony->imu_const.a + h);
			// Eastern component - zero for normal gravity
			pony->imu[dev].g[0] = 0;
			// Northern deflection with altitude above ellipsoid from plumb line curvature, as in (5-34), Physical Geodesy, W.Heiskanen H.Moritz, 1993, p. 196
			pony->imu[dev].g[1] = -pony->imu_const.ge*pony->imu_const.fg*sin2lat*h_a;
			// vertical component, as from section 3 of Geodetic Reference System 80 by H. Moritz (GRS-80)
			pony->imu[dev].g[2] = -pony->imu_const.ge*
				(1 + pony->imu_const.fg*sinlat*sinlat - f4_4*sin2lat*sin2lat)*
				(1 - 2*(1 + f*cos2lat + m)*h_a + h_a*h_a);
			// validity flag up
			pony->imu[dev].g_valid = 1;
		}

	}

}


// service functions
	// memory handling
void pony_ins_gravity_free_null(void** ptr)
{
	if (ptr == NULL || *ptr == NULL)
		return;

	free(*ptr);
	*ptr = NULL;
}
