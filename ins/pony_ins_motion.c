// Aug-2021
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
		
	- pony_ins_motion_size_effect
		Compensates for accelerometer cluster size effect, i.e. the spatial separation 
		between accelerometer proof masses, also known as "accelerometer lever arm compensation".
		Recommended only for high-grade systems on dynamically maneuvering carriers.
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
double pony_ins_motion_parse_double(const char* token, char* src, const size_t len, double range[2], const double default_value);
char   pony_ins_motion_parse_double_2array(double* arr, const size_t outer_size, const size_t inner_size, const char* token, char *src, const size_t len, double range[2], double default_value);
void   pony_ins_motion_flip_sol_over_pole(pony_sol* sol);

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
			pony_ins_motion_parse_double(lon_token, pony->imu->cfg,pony->imu->cfglength, (double*)lon_range,0)
			/pony->imu_const.rad2deg; // degrees to radians
		pony->imu->sol.llh[1] = // starting latitude
			pony_ins_motion_parse_double(lat_token, pony->imu->cfg,pony->imu->cfglength, (double*)lat_range,0)
			/pony->imu_const.rad2deg; // degrees to radians
		pony->imu->sol.llh[2] = // starting altitude
			pony_ins_motion_parse_double(alt_token, pony->imu->cfg,pony->imu->cfglength, (double*)alt_range,0);
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
		Re_h = pony->imu_const.a*(1 + e2s2/2 + 3*e4s4/8);                                            // Taylor expansion within 0.5 m, not altitude-adjusted
		Rn_h = Re_h*(1 - pony->imu_const.e2)*(1 + e2s2 + e4s4 + e2s2*e4s4) + pony->imu->sol.llh[2]; // Taylor expansion within 0.5 m
		Re_h += pony->imu->sol.llh[2];                                                               // adjust for altitude
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
			pony->imu->sol.llh[2] += pony->imu->sol.v[2]            *dt;
			pony->imu->sol.llh_valid = 0; // drop validity
		}
		else {
			pony->imu->sol.llh[0] += pony->imu->sol.v[0]/(Re_h*cphi)*dt;
			pony->imu->sol.llh[1] += pony->imu->sol.v[1]/ Rn_h      *dt;
			pony->imu->sol.llh[2] += pony->imu->sol.v[2]            *dt;
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
		t0            = -1, // previous time
		t0_air        = -1, // previous time, air  data
		t0_gnss       = -1, // previous time, gnss data
		vvs           =  0, // vertical velocity stdev
		 air_alt_last =  0, // previous value of air altitude
		gnss_alt_last =  0; // previous value of air altitude

	double 
		dt,	     // time step
		dt_air,  // time step, air  data
		dt_gnss, // time step, gnss data
		x,       // inertial altitude
		v,       // inertial vertical velocity
		z,       // reference information
		s,       // reference information a priori stdev
		h[2],    // model coefficients
		y[2],    // algorithm state vector
		S[3],    // upper-triangular part of cobariance Cholesky factorization
		K[2],    // Kalman gain
		w;       // weight


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
			// time handling
			if (pony->air->alt_valid || pony->air->vv_valid) {
				if (t0_air < 0 && pony->air->alt_valid) {
					t0_air       = pony->imu->t;
					air_alt_last = pony->air->alt;
				}
				else {
					dt_air = pony->imu->t - t0_air;
					t0_air = pony->imu->t;
				}
			}
			// vertical channel update
			if (t0_air > 0 && dt_air > 0) {
				if (pony->air->alt_valid && pony->imu->sol.llh_valid) { // altitude data present
					// altitude
					s = sqrt2*( (pony->air->alt_std > 0) ? (pony->air->alt_std) : vvs );
					z = pony->air->alt + air_alt_last; // decorrelated with velocity information
					h[0] = 2, h[1] = -dt_air;
					pony_linal_kalman_update(y,S,K, z,h,s, 2);
					// altitude rate of change
					z = pony->air->alt - air_alt_last; // decorrelated with altitude information
					h[0] = 0, h[1] = dt_air;
					pony_linal_kalman_update(y,S,K, z,h,s, 2);
					air_alt_last = pony->air->alt;
				}
				if (pony->air->vv_valid && pony->imu->sol.v_valid) { // vertical velocity data present
					z = pony->air->vv - pony->imu->sol.v[2];
					s = (pony->air-> vv_std > 0) ? (pony->air-> vv_std) : vvs;
					h[0] = 0, h[1] = 1;
					pony_linal_kalman_update(y,S,K, z,h,s, 2);
				}
			}
		}

		// check for gnss data
		if (pony->gnss != NULL) {
			// time handling
			if (pony->gnss->sol.llh_valid || pony->gnss->sol.v_valid) {
				if (t0_gnss < 0 && pony->gnss->sol.llh_valid) {
					t0_gnss       = pony->imu->t;
					gnss_alt_last = pony->gnss->sol.llh[2];
				}
				else {
					dt_gnss = pony->imu->t - t0_gnss;
					t0_gnss = pony->imu->t;
				}
			}
			// vertical channel update
			if (t0_gnss > 0 && dt_gnss > 0) {
				if (pony->gnss->sol.llh_valid && pony->imu->sol.llh_valid) { // altitude data present
					// altitude
					s = sqrt2*( (pony->gnss->sol.x_std > 0) ? (pony->gnss->sol.x_std) : pony->gnss->settings.code_sigma );
					z = pony->gnss->sol.llh[2] + gnss_alt_last; // decorrelated with velocity information
					h[0] = 2, h[1] = -dt_gnss;
					pony_linal_kalman_update(y,S,K, z,h,s, 2);
					// altitude rate of change
					z = pony->gnss->sol.llh[2] - gnss_alt_last; // decorrelated with altitude information
					h[0] = 0, h[1] = dt_gnss;
					pony_linal_kalman_update(y,S,K, z,h,s, 2);
					gnss_alt_last = pony->gnss->sol.llh[2];
				}
				if (pony->gnss->sol.v_valid && pony->imu->sol.v_valid) { // vertical velocity data present
					z = pony->gnss->sol.v[2] - pony->imu->sol.v[2];
					s = (pony->gnss->sol.v_std > 0) ? (pony->gnss->sol.v_std) : pony->gnss->settings.doppler_sigma;
					h[0] = 0, h[1] = 1;
					pony_linal_kalman_update(y,S,K, z,h,s, 2);
				}
			}
		}

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

/* pony_ins_motion_size_effect - pony plugin
	
	Compensates for accelerometer cluster size effect, i.e. the spatial separation 
	between accelerometer proof masses, also known as "accelerometer lever arm compensation".
	Uses angular rate and computed angular acceleration, taken as rate increment over the time step.
	Recommended only for high-grade systems on dynamically maneuvering carriers.

	description:
	    
		f''x = f'x - [-(wy^2+wz^2)rxx + (wx wy)rxy + (wx wz)rxz + (dwy/dt)rxz - dwz/dt)rxy]
		f''y = f'y - [ (wy wx)ryx - (wz^2+wx^2)ryy + (wy wz)ryz + (dwz/dt)ryx - dwx/dt)ryz]
		f''z = f'z - [ (wz wx)rzx + (wz wy)rzy - (wx^2+wy^2)rzz + (dwx/dt)rzy - dwy/dt)rzx]
		    
		where

		f   - specific force
		w   - angular rate (omega)
		rij - i-th accelerometer proof mass coordinate along j-th instrumental axis
		
	uses:
		pony->imu->t
		pony->imu->f
		pony->imu->f_valid
		pony->imu->w
		pony->imu->w_valid

	changes:
		pony->imu->f

	cfg parameters:
		{imu: accel_pos} - accelerometer proof mass coordinates in instrumental frame, in meters
			type :   three floating point arrays of three numbers each, enclosed in nested square brackets
			range:   -1..+1
			default: [[0 0 0],[0 0 0],[0 0 0]]
			example: {imu: accel_pos = [[0 +0.022 -0.041] [-0.006 0 +0.023] [+0.022 -0.002 0]]}
			note:    either character from 0x01 to 0x20 (space), and also 0x2c (comma ',') and 0x3b (semicolon ';') may serve as separator
*/
void pony_ins_motion_size_effect(void) {

	const char 
		accel_pos_token[] = "accel_pos"; // accelerometer proof mass coordinates parameter name in configuration
	const double 
		pos_range[]       = {-1, +1},    // accelerometer proof mass coordinates range
		eps               = 1.0/0x4000;  // 2^-14, smallest positive normal number in IEEE754 half-precision format
									     
	static double 					     
		t0    = -1,                      // previous time
		w0[3] = {0,0,0},                 // previous angular rate to calculate acceleration
		r[]   = {0,0,0, 0,0,0, 0,0,0};   // accelerometer proof mass coordinates (lever arm coordinates)

	double 
		dt,              // time step
		 w[3],           // angular rate permuted
		dw[3];           // computed angular acceleration
	size_t i, j, i1, i2; // common index variables


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// parse parameters from configuration	
		pony_ins_motion_parse_double_2array(r,3,3, accel_pos_token, pony->imu->cfg,pony->imu->cfglength, (double*)pos_range,0);
		// zero angular acceleration and previous gyroscope measurements at start
		for (i = 0; i < 3; i++)
			dw[i] = 0;
		// reset previous time
		t0 = -1;

	}

	else if (pony->mode < 0) {	// termination
		// do nothing
	}

	else						// main cycle
	{

		// check for inertial data initialized
		if (0
			|| !pony->imu->f_valid 
			|| !pony->imu->w_valid)
			return;
		// time variables
		if (t0 < 0) { // first touch
			t0 = pony->imu->t;
			for (i = 0; i < 3; i++)
				w0[i] = pony->imu->w[i];
			return;
		}
		dt = pony->imu->t - t0;
		t0 = pony->imu->t;
		if (dt < eps)
			return; // do nothing if no proper time increment detected
		// angular acceleration
		for (i = 0; i < 3; i++) {
			dw[i] = (pony->imu->w[i] - w0[i])/dt;
			w0[i] =  pony->imu->w[i];
		}
		// size effect compensation
		for (i = 0, j = 0; i < 3; i++, j += 3) {
			i1 = (i+1)%3;
			i2 = (i+2)%3;
			w[0] = pony->imu->w[i ];
			w[1] = pony->imu->w[i1];
			w[2] = pony->imu->w[i2];
			pony->imu->f[i] -= -(w[1]*w[1] + w[2]*w[2])*r[j+i] + w[0]*w[1]*r[j+i1] + w[0]*w[2]*r[j+i2] + dw[i1]*r[j+i2] - dw[i2]*r[j+i1];
		}

	}

}





// service functions
double pony_ins_motion_parse_double(const char* token, char* src, const size_t len, double* range, const double default_value) {

	char*  src_ptr; // pointer to a substring
	double val;     // value

	// ensure variable to be initialized
	val = default_value;
	// parse token from src
	src_ptr = pony_locate_token(token, src, len, '=');
	if (src_ptr != NULL)
		val = atof(src_ptr);
	// if not found or out of range, set to default
	if ( src_ptr == NULL || (range != NULL && (range[0] > range[1] || val < range[0] || range[1] < val)) )
		val = default_value;
	return val;

}

char pony_ins_motion_parse_double_2array(double* arr, const size_t outer_size, const size_t inner_size, const char* token, char *src, const size_t len, double* range, double default_value)
{
	const char
		brc_open  = '[',
		brc_close = ']',
		space     = ' ',
		comma     = ',',
		semicol   = ';';

	char*  src_ptr;
	double val;
	size_t i, j, k, k0;

	// parse token from src
	src_ptr = pony_locate_token(token, src, len, '=');
	if (src_ptr == NULL)
		return 0;
	// go through until a bracket opens
	for (i = src_ptr - src; i < len && src[i] > 0 && src[i] <= space; i++); // next printable character
	if (i >= len || src[i] != brc_open) // no opening bracket
		return 0;
	// iterate over outer array size
	for (j = 0, k0 = 0; j < outer_size; j++, k0 += inner_size) {
		for (i++; i < len && src[i] > 0 && (src[i] <= space || src[i] == comma || src[i] == semicol); i++); // next printable character that is not a separator
		if (i >= len || src[i] != brc_open) // no opening bracket
			return 0;
		// parse inner values
		for (k = 0; k < inner_size; k++) {
			for (i++; i < len && src[i] > 0 && (src[i] <= space || src[i] == comma || src[i] == semicol); i++); // next printable character that is not a separator
			if (i >= len || src[i] == 0) // end of source string before all values have been parsed
				return 0;
			// ensure that value is initialized
			val = default_value;
			val = atof(src+i);
			// if out of range, set to default
			if (range != NULL && (range[0] > range[1] || val < range[0] || range[1] < val))
				val = default_value;
			// assign to array
			arr[k0 + k] = val;
			// skip until a separator appears
			for (; i < len && src[i] > space && src[i] != comma && src[i] != semicol && src[i] != brc_close; i++); // next separator or closing bracket
			if (i >= len || src[i] == 0) // end of source string before all values have been parsed
				return 0;
		}
		// go through until a bracket closes
		for (; i < len && src[i] > 0 && src[i] != brc_close; i++);
		if (i >= len || src[i] == 0) // end of source string before the array has been parsed
				return 0;
	}
	// go through until a bracket closes
	for (i++; i < len && src[i] > 0 && src[i] != brc_close; i++);
	if (i >= len || src[i] == 0) // end of source string before the array has been parsed
		return 0;
	else
		return 1; // closing bracket found

}

void pony_ins_motion_flip_sol_over_pole(pony_sol* sol) {

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
