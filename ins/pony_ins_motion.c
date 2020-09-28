// Sep-2020
/*	pony_ins_motion
	
	pony plugins for ins position and velocity algorithms:
	
	- pony_ins_motion_euler
		Numerically integrates Newton's second Law in local level navigation frame 
		using modified Euler's method (with midpoint for attitude matrix) over the Earth reference ellipsoid. 
		Updates velocity and geographical coordinates. 
		Not suitable near the Earth's poles, at outer-space altitudes, and/or over-Mach velocities.
		
	- (planned) pony_ins_motion_sculling
		Planned for future development
		
	- pony_ins_motion_vertical_damping
		Restrains vertical error exponential growth by either damping
		vertical velocity to zero, or to an external reference like
		altitude/rate derived from air data system and/or gnss, etc.
		Recommended when using normal gravity model and/or long navigation timeframe.
*/

#include <stdlib.h>
#include <math.h>

#include "../pony.h"

// pony bus version check
#define PONY_INS_MOTION_BUS_VERSION_REQUIRED 8
#if PONY_BUS_VERSION < PONY_INS_MOTION_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif

// service functions
double pony_ins_motion_parse_double(const char *token, char *src, const size_t len, double range[2], const double default_value);
void   pony_ins_motion_flip_sol_over_pole(pony_sol *sol);

/* pony_ins_motion_euler - pony plugin
	
	Numerically integrates Newton's second Law in local level navigation frame 
	using modified Euler's method (with midpoint for attitude matrix) over the Earth reference ellipsoid. 
	Updates velocity and geographical coordinates. 
	Not suitable near the Earth's poles, at outer-space altitudes, and/or over-Mach velocities.

	description:
	    
		V  (t+dt) = V  (t) + dt*([(W + 2u) x]*V (t) +  L^T(t+dt/2)*f + g)
		lon(t+dt) = lon(t) + dt*              Ve(t)/((Re + alt(t))*cos(lat(t)))
		lat(t+dt) = lat(t) + dt*              Vn(t)/( Rn + alt(t))
		alt(t+dt) = alt(t) + dt*              Vu(t)              
		    
		where
		    [Ve]          [  0   v3 -v2 ]
		V = [Vn], [v x] = [ -v3  0   v1 ] for v = W + 2u,
			[Vu]          [  v2 -v1  0  ]
		Rn, Re are curvature radii
		L is attitude matrix

		do not use at the Earth's poles
		
	uses:
		pony->imu->t
		pony->imu->sol.llh
		pony->imu->sol.llh_valid
		pony->imu->sol.v
		pony->imu->sol.v_valid
		pony->imu->sol.L
		pony->imu->sol.L_valid
		pony->imu->f
		pony->imu->f_valid
		pony->imu->w
		pony->imu->w_valid
		pony->imu->g
		pony->imu->g_valid
		pony->imu->W
		pony->imu->W_valid

	changes:
		pony->imu->sol.v
		pony->imu->sol.v_valid
		pony->imu->sol.llh
		pony->imu->sol.llh_valid

	cfg parameters:
		{imu: lon} - starting longitude, degrees
			type :   floating point
			range:   -180..+180
			default: 0
			example: {imu: lon = 37.6}
		{imu: lat} - starting latitude, degrees
			type :   floating point
			range:   -90..+90
			default: 0
			example: {imu: lat = 55.7}
		{imu: alt} - starting altitude, meters
			type :   floating point
			range:   -20000..+50000
			default: 0
			example: {imu: alt = 151.3}
*/
void pony_ins_motion_euler(void) {

	const char 
		lon_token[] = "lon",        // starting longitude parameter name in configuration
		lat_token[] = "lat",        // starting latitude  parameter name in configuration
		alt_token[] = "alt";        // starting altitude  parameter name in configuration

	const double 
		lon_range[] = {-180, +180},	// longitude range, 0 by default
		lat_range[] = { -90,  +90},	// latitude  range, 0 by default
		alt_range[] = {-20e3,50e3},	// altitude  range, 0 by default
		eps = 1.0/0x0100;			// 2^-8, guaranteed non-zero value in IEEE754 half-precision format

	static double t0  = -1;         // previous time

	double dt;                      // time step
	double
		sphi, cphi,	                // sine and cosine of latitude
		Rn_h, Re_h,	                // south-to-north and east-to-west curvature radii, altitude-adjusted
		e2s2, e4s4,	                // e^2*sin(phi)^2, (e^2*sin(phi)^2)^2
		dvrel[3],	                // proper acceleration in navigation frame
		dvcor[3],	                // Coriolis acceleration in navigation frame
		C_2[9];                     // intermediate matrix/vector for modified Euler component
	size_t i, j;                    // common index variables


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// drop validity flags
		pony->imu->sol.  v_valid = 0;
		pony->imu->sol.llh_valid = 0;		
		// parse parameters from configuration	
		pony->imu->sol.llh[0] = // starting longitude
			pony_ins_motion_parse_double(lon_token, pony->imu->cfg,pony->imu->cfglength, (double *)lon_range,0)
			/pony->imu_const.rad2deg; // degrees to radians
		pony->imu->sol.llh[1] = // starting latitude
			pony_ins_motion_parse_double(lat_token, pony->imu->cfg,pony->imu->cfglength, (double *)lat_range,0)
			/pony->imu_const.rad2deg; // degrees to radians
		pony->imu->sol.llh[2] = // starting altitude
			pony_ins_motion_parse_double(alt_token, pony->imu->cfg,pony->imu->cfglength, (double *)alt_range,0);
		// raise coordinates validity flag
		pony->imu->sol.llh_valid = 1;
		// zero velocity at start
		for (i = 0; i < 3; i++)
			pony->imu->sol.v[i] = 0;
		pony->imu->sol.v_valid = 1;
		// reset previous time
		t0 = -1;

	}

	else if (pony->mode < 0) {	// termination
		// do nothing
	}

	else						// main cycle
	{
		// check for crucial data initialized
		if (   !pony->imu->sol.  v_valid 
			|| !pony->imu->sol.llh_valid 
			|| !pony->imu->sol.  L_valid 
			|| !pony->imu->f_valid 
			|| !pony->imu->g_valid)
			return;
		// time variables
		if (t0 < 0) { // first touch
			t0 = pony->imu->t;
			return;
		}
		dt = pony->imu->t - t0;
		t0 = pony->imu->t;
		// ellipsoid geometry
		sphi = sin(pony->imu->sol.llh[1]);
		cphi = cos(pony->imu->sol.llh[1]);
		e2s2 = pony->imu_const.e2*sphi*sphi;
		e4s4 = e2s2*e2s2;					
		Re_h = pony->imu_const.a*(1 + e2s2/2 + 3*e4s4/8);	                                        // Taylor expansion within 0.5 m, not altitude-adjusted
		Rn_h = Re_h*(1 - pony->imu_const.e2)*(1 + e2s2 + e4s4 + e2s2*e4s4) + pony->imu->sol.llh[2]; // Taylor expansion within 0.5 m
		Re_h += pony->imu->sol.llh[2];						                                        // adjust for altitude
		// drop validity flags
		pony->imu->W_valid       = 0;
		pony->imu->sol.llh_valid = 0;
		pony->imu->sol.  v_valid = 0;
		// angular rate of navigation frame relative to the Earth
		pony->imu->W[0] = -pony->imu->sol.v[1]/Rn_h;
		pony->imu->W[1] =  pony->imu->sol.v[0]/Re_h;	
		if (cphi < eps) { // check for Earth pole proximity
			pony->imu->W[2] = 0;    // freeze
			pony->imu->W_valid = 0; // drop validity
		}
		else {
			pony->imu->W[2] = pony->imu->sol.v[0]/Re_h*sphi/cphi;
			pony->imu->W_valid = 1;
		}
		// velocity
			// Coriolis acceleration
		dvrel[0] = pony->imu->W[0];
		dvrel[1] = pony->imu->W[1] + 2*pony->imu_const.u*cphi;
		dvrel[2] = pony->imu->W[2] + 2*pony->imu_const.u*sphi;
		pony_linal_cross3x1(dvcor, pony->imu->sol.v, dvrel);
			// proper acceleration
		if (pony->imu->w_valid) { // if able to calculate attitude mid-point using gyroscopes
			for (i = 0; i < 3; i++)
				dvrel[i] = pony->imu->w[i]*dt/2; // midpoint rotation Euler vector
			pony_linal_eul2mat(C_2, dvrel); // midpoint attitude matrix factor
			for (i = 0; i < 3; i++) // multiply C_2*f, store in the first row of C_2
				for (j = 1, C_2[i] = C_2[i*3]*pony->imu->f[0]; j < 3; j++)
					C_2[i] += C_2[i*3+j]*pony->imu->f[j];
			pony_linal_mmul1T(dvrel, pony->imu->sol.L, C_2, 3, 3, 1); // dvrel = L^T(t+dt)*C_2(w*dt/2)*f
		}
		else // otherwise, go on with only the current attitude matrix
			pony_linal_mmul1T(dvrel, pony->imu->sol.L, pony->imu->f, 3, 3, 1);
			// velocity update
		for (i = 0; i < 3; i++)
			pony->imu->sol.v[i] += (dvcor[i] + dvrel[i] + pony->imu->g[i])*dt;
		pony->imu->sol.v_valid = 1;
		// coordinates
		if (cphi < eps) { // check for Earth pole proximity
			pony->imu->sol.llh[2] += pony->imu->sol.v[2]				*dt;
			pony->imu->sol.llh_valid = 0; // drop validity
		}
		else {
			pony->imu->sol.llh[0] += pony->imu->sol.v[0]/(Re_h*cphi)	*dt;
			pony->imu->sol.llh[1] += pony->imu->sol.v[1]/ Rn_h			*dt;
			pony->imu->sol.llh[2] += pony->imu->sol.v[2]				*dt;
			// flip latitude if crossed a pole
			if (pony->imu->sol.llh[1] < -pony->imu_const.pi/2) { // South pole
				pony->imu->sol.llh[1] = -pony->imu_const.pi - pony->imu->sol.llh[1];
				pony_ins_motion_flip_sol_over_pole(&(pony->imu->sol));			
			}
			if (pony->imu->sol.llh[1] > +pony->imu_const.pi/2) { // North pole
				pony->imu->sol.llh[1] = +pony->imu_const.pi - pony->imu->sol.llh[1];
				pony_ins_motion_flip_sol_over_pole(&(pony->imu->sol));			
			}
			// adjust longitude into range
			while (pony->imu->sol.llh[0] < -pony->imu_const.pi)
				pony->imu->sol.llh[0] += 2*pony->imu_const.pi;
			while (pony->imu->sol.llh[0] > +pony->imu_const.pi)
				pony->imu->sol.llh[0] -= 2*pony->imu_const.pi;
			pony->imu->sol.llh_valid = 1;
		}
	}

}

/* pony_ins_motion_vertical_damping - pony plugin
	
	Restrains vertical error exponential growth by either damping
	vertical velocity to zero, or to an external reference like
	altitude/rate derived from air data system and/or gnss, etc.
	Recommended when using normal gravity model and/or long navigation timeframe.

	description:
		performs vertical channel correction using
		- zero vertical velocity model
		- air data altitude and altitude rate of change
		- (planned) gnss-derived altitude and vertical velocity
		- etc.
								      [x]          [v]
		algorithm state vector is y = [v], dy/dt = [q], M[q^2] = vvs^2

	uses:
		pony->imu->t
		pony->air->alt
		pony->air->alt_valid
		pony->air->alt_std
		pony->air->vv
		pony->air->vv_valid
		pony->air->vv_std
		pony->imu->sol.llh[2]
		pony->imu->sol.llh_valid
		pony->imu->sol.  v[2]
		pony->imu->sol.  v_valid

	changes:
		pony->imu->sol.llh[2]
		pony->imu->sol.  v[2]

	cfg parameters:
		{imu: vertical_damping_stdev} - vertical velocity stdev (vvs), m/s
			type :   floating point
			range:   >0
			default: 2^20 (no damping)
			example: {imu: vertical_damping_stdev = 9e9}
			set vvs = 0 to force zero vertical velocity
			negative values result in default
*/
void pony_ins_motion_vertical_damping(void) {

	const char vvs_token[] = "vertical_damping_stdev"; // vertical velocity stdev parameter name in configuration

	const double 
		vvs_def = (double)(0x100000),                  // 2^20, vertical velocity stdev default value
		sqrt2   = 1.4142135623730951;                  // sqrt(2)

	static double 
		t0           = -1, // previous time
		vvs          =  0, // vertical velocity stdev
		air_alt_last =  0; // previous value of air altitude

	double 
		dt,	  // time step
		x,    // inertial altitude
		v,    // inertial vertical velocity
		z,    // reference information
		s,    // reference information a priori stdev
		h[2], // model coefficients
		y[2], // algorithm state vector
		S[3], // upper-triangular part of cobariance Cholesky factorization
		K[2], // Kalman gain
		w;    // weight


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// reset time
		t0 = -1;
		// parse vertical velocity stdev from configuration string
		vvs = pony_ins_motion_parse_double(vvs_token, pony->imu->cfg, pony->imu->cfglength, NULL, vvs_def);
		if (vvs < 0)
			vvs = vvs_def;

	}

	else if (pony->mode < 0) {	// termination
		// do nothing
	}
	else						// main cycle
	{
		// time variables
		if (t0 < 0) {
			t0 = pony->imu->t;
			if (pony->air != NULL && pony->air->alt_valid)
				air_alt_last = pony->air->alt;
			return;
		}
		dt = pony->imu->t - t0;
		t0 = pony->imu->t;
		if (dt <= 0)
			return;
		// estimation init 
		if (pony->imu->sol.llh_valid) x = pony->imu->sol.llh[2]; else x = 0;
		if (pony->imu->sol.  v_valid) v = pony->imu->sol.  v[2]; else v = 0;
		y[0] = x, y[1] = v;
		S[0] = vvs_def*dt, S[1] = 0, S[2] = 1;
		// zero vertical velocity
		if (pony->imu->sol.v_valid) {
			s = vvs;
			z = 0.0;
			h[0] = 0, h[1] = 1;
			pony_linal_kalman_update(y,S,K, z,h,s, 2);
		}
		// check for air data
		if (pony->air != NULL) {
			if (pony->air->alt_valid && pony->imu->sol.llh_valid) { // altitude data present
				// altitude
				s = sqrt2*( (pony->air->alt_std > 0) ? (pony->air->alt_std) : vvs );
				z = pony->air->alt + air_alt_last; // decorrelated with velocity information
				h[0] = 2, h[1] = -dt;
				pony_linal_kalman_update(y,S,K, z,h,s, 2);
				// altitude rate of change
				z = pony->air->alt - air_alt_last; // decorrelated with altitude information
				h[0] = 0, h[1] = dt;
				pony_linal_kalman_update(y,S,K, z,h,s, 2);
				air_alt_last = pony->air->alt;
			}
			if (pony->air->vv_valid && pony->imu->sol.v_valid) { // vertical velocity data present
				z = pony->air->vv - pony->imu->sol.v[2];
				s = (pony->air-> vv_std > 0) ? (pony->air-> vv_std) : vvs;
				pony_linal_kalman_update(y,S,K, z,h,s, 2);
			}
		}

		// check for gnss data
		//
		// TBD
		//

		// vertical channel correction
		s = sqrt(S[0]*S[0] + S[1]*S[1]);  // altitude stdev estimate
		w = s + S[2]*dt;                  // sum of altitude stdev and velocity contribution into stdev
		if (pony->imu->sol.llh_valid) 
			pony->imu->sol.llh[2] = y[0]; // replace altitude by its estimated value
		if (pony->imu->sol.  v_valid) {
			pony->imu->sol.llh[2] += 2*s      /w*(y[1]-v)*dt; // weighted correction to hold dx/dt = v
			pony->imu->sol.  v[2] = y[1]; // replace vertical velocity by its estimated value
		}
		if (pony->imu->sol.llh_valid) 
			pony->imu->sol.  v[2] += 2*S[2]*dt/w*(y[0]-x)/dt; // weighted correction to hold dx/dt = v

	}

}





// service functions
double pony_ins_motion_parse_double(const char *token, char *src, const size_t len, double *range, const double default_value) {

	char   *cfg_ptr; // pointer to a substring
	double  val;     // value

	// ensure variable to be initialized
	val = default_value;
	// parse token from src
	cfg_ptr = pony_locate_token(token, src, len, '=');
	if (cfg_ptr != NULL)
		val = atof(cfg_ptr);
	// if not found or out of range, set to default
	if ( cfg_ptr == NULL || (range != NULL && (range[0] > range[1] || val < range[0] || range[1] < val)) )
		val = default_value;
	return val;

}

void pony_ins_motion_flip_sol_over_pole(pony_sol *sol) {

	size_t i;

	// longitude
	sol->llh[0] +=  pony->imu_const.pi;
	// velocity
	sol->v  [0]  = -sol->v[0];
	sol->v  [1]  = -sol->v[1];
	// attitude matrix
	for (i = 0; i < 3; i++) {
		sol->L[i*3+0] = -sol->L[i*3+0];
		sol->L[i*3+1] = -sol->L[i*3+1];
	}
	// update quaternion
	pony_linal_mat2quat(sol->q  ,sol->L);
	// update angles
	pony_linal_mat2rpy (sol->rpy,sol->L);

}
