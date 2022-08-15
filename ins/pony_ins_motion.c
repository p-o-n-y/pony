// Aug-2022
/*	pony_ins_motion
	
	pony plugins for ins position and velocity algorithms:
	
	- pony_ins_motion_euler
		Numerically integrates Newton's second Law in local level navigation frame
		using modified Euler's method (with midpoint for attitude matrix) over the Earth reference ellipsoid. 
		Updates velocity and geographical coordinates for each initialized IMU device with valid 
		accelerometer measurements, gravity vector, and current position, velocity and attitude matrix.
		Not suitable at outer-space altitudes, and/or over-Mach velocities.
		
	- (planned) pony_ins_motion_sculling
		Planned for future development
		
	- pony_ins_motion_vertical_damping
		Restrains vertical error exponential growth by either damping
		vertical velocity to zero, or to an external reference like
		altitude/rate derived from air data system and/or gnss, etc.
		Preforms the damping for all initialized IMU devices.
		Recommended when using normal gravity model and/or long navigation timeframe.
		
	- pony_ins_motion_size_effect
		Compensates for accelerometer cluster size effect, i.e. the spatial separation 
		between accelerometer proof masses, also known as "accelerometer lever arm compensation".
		Uses angular rate and computed angular acceleration, taken as rate increment over the time step.
		Preforms the compensation for all initialized IMU devices with accelerometer positions specified.
		Recommended only for high-grade systems on dynamically maneuvering carriers.
*/


#include <stdlib.h>
#include <math.h>

#include "../pony.h"


// pony bus version check
#define PONY_INS_MOTION_BUS_VERSION_REQUIRED 18
#if PONY_BUS_VERSION < PONY_INS_MOTION_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif


// service functions
	// navigation
double pony_ins_motion_pos_step_enu (double (*llh  )[3], double (*W    )[3], char *W_valid,    const double v[3], const double dt, const double sin_lat, const double cos_lat); // pre-calculated latitude trigonometry
void   pony_ins_motion_acc_enu2llazw(double (*dvcor)[3], double (*dvrel)[3], double (*dvg)[3],  const double W[3], const double V[3], const double f[3], const double g[3], const double sin_lat, const double cos_lat, const double sin_chi, const double cos_chi); // pre-calculated trigonometry
	// parsing
double pony_ins_motion_parse_double(const char* token, char* src, const size_t len, double range[2], const double default_value);
char   pony_ins_motion_parse_double_2array(double* arr, const size_t outer_size, const size_t inner_size, const char* token, char *src, const size_t len, double range[2], double default_value);
double pony_ins_motion_parse_double_imu_or_settings(const char* token, const size_t imu_dev, double range[2], const double default_value);
	// math
double pony_ins_motion_sign(double x) {return (x > 0) ? +1.0 : (x < 0 ? -1.0 : 0.0);}
	// memory handling
void pony_ins_motion_free_null(void** ptr);


/* pony_ins_motion_euler - pony plugin
	
	Numerically integrates Newton's second Law in local level navigation frame 
	using modified Euler's method (with midpoint for attitude matrix) over the Earth reference ellipsoid. 
	Updates velocity and geographical coordinates for each initialized IMU device with valid 
	accelerometer measurements, gravity vector, and current position, velocity and attitude matrix.
	Not suitable at outer-space altitudes, and/or over-Mach velocities.

	description:
	    
		V  (t+dt) = V  (t) + dt*([(W + 2u) x]*V (t) +  L^T(t+dt/2)*f + g) // via azimuth wandering reference frame
		lon(t+dt) = lon(t) + dt*              Ve(t)/((Re + alt(t))*cos(lat(t))) (1)
		lat(t+dt) = lat(t) + dt*              Vn(t)/( Rn + alt(t))              (1)
		alt(t+dt) = alt(t) + dt*              Vu(t)              
		    
		where
		    [Ve]          [  0   v3 -v2 ]
		V = [Vn], [v x] = [ -v3  0   v1 ] for v = W + 2u,
			[Vu]          [  v2 -v1  0  ]
		Rn, Re are curvature radii
		L is attitude matrix

		(1) near the Earth's poles, special geometrical derivations are used
		
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
		pony->imu->W
		pony->imu->W_valid

	cfg parameters:
		lon
		or
		{imu: lon} - starting longitude, degrees
			type :   floating point
			range:   -180..+180
			default: 0
			example: {imu: lon = 37.6}
			note:    if specified for the particular IMU device, the value overrides one from the common configuration settings (outside of groups)

		lat
		or
		{imu: lat} - starting latitude, degrees
			type :   floating point
			range:   -90..+90
			default: 0
			example: {imu: lat = 55.7}
			note:    if specified for the particular IMU device, the value overrides one from the common configuration settings (outside of groups)

		alt
		or
		{imu: alt} - starting altitude, meters
			type :   floating point
			range:   -20000..+50000
			default: 0
			example: {imu: alt = 151.3}
			note:    if specified for the particular IMU device, the value overrides one from the common configuration settings (outside of groups)
*/
void pony_ins_motion_euler(void) {

	enum {E, N, U};

	const char 
		lon_token[] = "lon",        // starting longitude parameter name in configuration
		lat_token[] = "lat",        // starting latitude  parameter name in configuration
		alt_token[] = "alt";        // starting altitude  parameter name in configuration
	const double 
		lon_range[] = {-360, +360},	// longitude range, 0 by default
		lat_range[] = { -90,  +90},	// latitude  range, 0 by default
		alt_range[] = {-20e3,50e3};	// altitude  range, 0 by default
	const char W_valid = 0x07;      // component-wise bitfield: three components

	static size_t ndev; // number of IMU devices
	static double 
		*chi  = NULL,   // azimuth angles, their sine and cosine for each IMU device
		*schi = NULL,
		*cchi = NULL,
		*t0   = NULL;   // previous times

	double
		f[3],       // proper acceleration in navigation frame
		Vx[3],      // relative velocity vector in azimuth wandering reference frame
		dvcor[3],   // Coriolis acceleration in azimuth wandering frame
		dvrel[3],   // proper   acceleration in azimuth wandering frame
		dvg  [3],   // gravity  acceleration in azimuth wandering frame
		C_2  [9],   // intermediate matrix/vector for midpoint attitude
		dlon,       // latitude and longitude increment
		sphi, cphi, // sine and cosine of latitude
		dt;         // time step
	size_t 
		i, j,       // common index variables
		dev;        // current IMU device


	// validate IMU subsystem
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// allocate memory
		ndev = pony->imu_count;
		 t0        = (double*)calloc(ndev, sizeof(double));
		 chi       = (double*)calloc(ndev, sizeof(double));
		schi       = (double*)calloc(ndev, sizeof(double));
		cchi       = (double*)calloc(ndev, sizeof(double));
		if (t0 == NULL || chi == NULL || schi == NULL || cchi == NULL) {
			pony->mode = -1;
			return;
		}
		// init for all devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// drop validity flags
			pony->imu[dev].sol.  v_valid = 0;
			pony->imu[dev].sol.llh_valid = 0;		
			// parse parameters from configuration	
			pony->imu[dev].sol.llh[E] = // starting longitude
				pony_ins_motion_parse_double_imu_or_settings(lon_token, dev, (double*)lon_range,0)/pony->imu_const.rad2deg; // degrees to radians
			pony->imu[dev].sol.llh[N] = // starting latitude
				pony_ins_motion_parse_double_imu_or_settings(lat_token, dev, (double*)lat_range,0)/pony->imu_const.rad2deg; // degrees to radians
			pony->imu[dev].sol.llh[U] = // starting altitude
				pony_ins_motion_parse_double_imu_or_settings(alt_token, dev, (double*)alt_range,0);
			// raise coordinates validity flag
			pony->imu[dev].sol.llh_valid = 1;
			// zero velocity at start
			for (i = 0; i < 3; i++)
				pony->imu[dev].sol.v[i] = 0;
			pony->imu[dev].sol.v_valid = 1;
			// reset previous time
			t0[dev] = -1;
		}

	}

	else if (pony->mode < 0) {	// termination
		
		// free allocated memory
		pony_ins_motion_free_null((void**)(& t0 ));
		pony_ins_motion_free_null((void**)(& chi));
		pony_ins_motion_free_null((void**)(&schi));
		pony_ins_motion_free_null((void**)(&cchi));

	}

	else						// regular processing
	{

		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// check for required data initialized
			if (   !pony->imu[dev].sol.  v_valid 
				|| !pony->imu[dev].sol.llh_valid)
				continue;
			// time variables
			if (t0[dev] < 0) { // first touch
				 t0 [dev] = pony->imu[dev].t;
				 chi[dev] = 0;
				schi[dev] = 0;
				cchi[dev] = 1;
				continue;
			}
			dt       = pony->imu[dev].t - t0[dev];
			t0[dev]  = pony->imu[dev].t;
			// drop validity flags
			pony->imu[dev].sol.llh_valid = 0;
			pony->imu[dev].sol.  v_valid = 0;
			pony->imu[dev].      W_valid = 0;
			// pre-calculated values
			sphi = sin(pony->imu[dev].sol.llh[N]);
			cphi = cos(pony->imu[dev].sol.llh[N]);
			// coordinates and navigation frame
			dlon = pony_ins_motion_pos_step_enu(
				&(pony->imu[dev].sol.llh), &(pony->imu[dev].W), &(pony->imu[dev].W_valid), // out
				pony->imu[dev].sol.v, dt, sphi, cphi); // in
			pony->imu[dev].sol.llh_valid = 1;
			// proper acceleration in navigation frame
			if (   !pony->imu[dev].sol.L_valid 
				|| !pony->imu[dev].    f_valid)
				continue;
			if (pony->imu[dev].w_valid) { // if able, calculate attitude mid-point using gyroscopes
				for (i = 0; i < 3; i++)
					dvrel[i] = pony->imu[dev].w[i]*dt/2; // midpoint rotation Euler vector
				pony_linal_eul2mat(C_2, dvrel); // midpoint attitude matrix factor
				for (i = 0; i < 3; i++) // multiply C_2*f, store in the first row of C_2
					for (j = 1, C_2[i] = C_2[i*3]*pony->imu[dev].f[0]; j < 3; j++)
						C_2[i] += C_2[i*3+j]*pony->imu[dev].f[j];
				pony_linal_mmul1T(f, pony->imu[dev].sol.L, C_2, 3, 3, 1); // dvrel = L^T(t+dt)*C_2(w*dt/2)*f
			}
			else // otherwise, go on with only the current attitude matrix
				pony_linal_mmul1T(f, pony->imu[dev].sol.L, pony->imu[dev].f, 3, 3, 1);
			// acceleration in azimuth wandering frame
			if (!pony->imu[dev].g_valid)
				continue;
			pony_ins_motion_acc_enu2llazw(
				&dvcor, &dvrel, &dvg, // out
				pony->imu[dev].W, pony->imu[dev].sol.v, f, pony->imu[dev].g, sphi, cphi, schi[dev], cchi[dev]); // in
			// velocity update in azimuth wandering frame
			Vx[0] =  pony->imu[dev].sol.v[E]*cchi[dev] + pony->imu[dev].sol.v[N]*schi[dev];
			Vx[1] = -pony->imu[dev].sol.v[E]*schi[dev] + pony->imu[dev].sol.v[N]*cchi[dev];
			Vx[U] =  pony->imu[dev].sol.v[U];
			for (i = 0; i < 3; i++)
				Vx[i] += (dvcor[i] + dvrel[i] + dvg[i])*dt;
			// update azimuth angle
			 chi[dev] -= dlon*sphi;
			schi[dev]  = sin(chi[dev]);
			cchi[dev]  = cos(chi[dev]);
			// rotate back into new azimuth
			pony->imu[dev].sol.v[E] = Vx[0]*cchi[dev] - Vx[1]*schi[dev];
			pony->imu[dev].sol.v[N] = Vx[0]*schi[dev] + Vx[1]*cchi[dev];
			pony->imu[dev].sol.v[U] = Vx[U];
			pony->imu[dev].sol.v_valid = 1;
		}

	}

}

/* pony_ins_motion_vertical_damping - pony plugin
	
	Restrains vertical error exponential growth by either damping
	vertical velocity to zero, or to an external reference like
	altitude/rate derived from air data system and/or gnss, etc.
	Preforms the damping for all initialized IMU devices.
	Recommended when using normal gravity model and/or long navigation timeframe.

	description:
		performs vertical channel correction using
		- zero vertical velocity model
		- air data altitude and altitude rate of change
		- gnss-derived altitude and vertical velocity from each initialized GNSS receiver
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
		vvs_def = (double)(0x100000), // 2^20, vertical velocity stdev default value
		sqrt2   = 1.4142135623730951, // sqrt(2)
		s0_h    = 10,                 // a priori stdev for the current altitude   
		s0_v    = 1;                  // a priori stdev for the current vertical velocity

	static size_t 
		ndev,                  // IMU device count
		n_gnss;                // number of GNSS receivers
	static double 
		* vvs           = NULL, // vertical velocity a priori stdev for each IMU device
		* t0            = NULL, // previous time					for each IMU device
		* t0_air        = NULL, // previous time, air  data         for each IMU device
		**t0_gnss       = NULL, // previous time, gnss data         for each IMU device, for each GNSS receiver
		*  air_alt_last = NULL, // previous value of air  altitude  for each IMU device
		**gnss_alt_last = NULL; // previous value of gnss altitude  for each IMU device, for each GNSS receiver

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
	size_t 
		i,        // general index
		r,        // current GNSS receiver id
		dev;      // current IMU device id


	// validate IMU subsystem
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// allocate memory
		ndev   = pony-> imu_count;
		n_gnss = pony->gnss_count;
		vvs           = (double* )calloc(ndev, sizeof(double ));
		t0            = (double* )calloc(ndev, sizeof(double ));
		t0_air        = (double* )calloc(ndev, sizeof(double ));
		t0_gnss       = (double**)calloc(ndev, sizeof(double*));
		 air_alt_last = (double* )calloc(ndev, sizeof(double ));
		gnss_alt_last = (double**)calloc(ndev, sizeof(double*));
		if (vvs == NULL || t0 == NULL || t0_gnss == NULL || gnss_alt_last == NULL) {
			pony->mode = -1;
			return;
		}
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			t0_gnss      [dev] = (double*)calloc(n_gnss, sizeof(double));
			gnss_alt_last[dev] = (double*)calloc(n_gnss, sizeof(double));
			if (t0_gnss[dev] == NULL || gnss_alt_last[dev] == NULL) {
				pony->mode = -1;
				return;
			}
		}
		// init for each IMU device
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// reset time
			t0    [dev] = -1;
			t0_air[dev] = -1;
			for (r = 0; r < n_gnss; r++)
				t0_gnss[dev][r] = -1;
			// parse vertical velocity stdev from configuration string
			vvs[dev] = pony_ins_motion_parse_double(vvs_token, pony->imu[dev].cfg, pony->imu[dev].cfglength, NULL, vvs_def);
			if (vvs[dev] < 0)
				vvs[dev] = vvs_def;
		}

	}

	else if (pony->mode < 0) {	// termination
		
		// free allocated memory
		pony_ins_motion_free_null((void**)(&vvs         ));
		pony_ins_motion_free_null((void**)(&t0          ));
		pony_ins_motion_free_null((void**)(&t0_air      ));
		pony_ins_motion_free_null((void**)(&air_alt_last));
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			pony_ins_motion_free_null((void**)(&(t0_gnss      [dev])));
			pony_ins_motion_free_null((void**)(&(gnss_alt_last[dev])));
		}

	}

	else						// regular processing
	{
		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL || (!pony->imu[dev].sol.llh_valid && !pony->imu[dev].sol.v_valid))
				continue;
			// time variables
			if (t0[dev] < 0) {
				t0[dev] = pony->imu[dev].t;
				// air data variables
				if (pony->air != NULL && pony->air->alt_valid)
					air_alt_last[dev] = pony->air->alt;
				continue;
			}
			dt      = pony->imu[dev].t - t0[dev];
			t0[dev] = pony->imu[dev].t;
			if (dt < 0)
				continue;
			// estimation init 
			if (pony->imu[dev].sol.llh_valid) x = pony->imu[dev].sol.llh[2]; else x = 0;
			if (pony->imu[dev].sol.  v_valid) v = pony->imu[dev].sol.  v[2]; else v = 0;
			y[0] = x, y[1] = v;
			S[0] = s0_h, S[1] = 0, S[2] = s0_v;
			// zero vertical velocity update
			if (pony->imu[dev].sol.v_valid) {
				s = vvs[dev];
				z = 0.0;
				h[0] = 0, h[1] = 1;
				pony_linal_kalman_update(y,S,K, z,h,s, 2);
			}
			
			// check for air data
			if (pony->air != NULL && (pony->air->alt_valid || pony->air->vv_valid)) {
				// time handling
				if (t0_air[dev] < 0 && pony->air->alt_valid) {
					air_alt_last[dev] = pony->air->alt;
					dt_air       = 0; // vertical velocity will not be observable
				}
				else
					dt_air = pony->imu[dev].t - t0_air[dev];
				t0_air[dev] = pony->imu[dev].t;
				// vertical channel update
				if (pony->air->alt_valid && pony->imu[dev].sol.llh_valid) { // altitude data present
					// altitude
					s = sqrt2*( (pony->air->alt_std > 0) ? (pony->air->alt_std) : vvs[dev] ); // a priori covariance: either specified, or the same as for inertial data
					z = pony->air->alt + air_alt_last[dev]; // decorrelated with velocity information
					h[0] = 2, h[1] = -dt_air;
					pony_linal_kalman_update(y,S,K, z,h,s, 2);
					// altitude rate of change
					z = pony->air->alt - air_alt_last[dev]; // decorrelated with altitude information
					h[0] = 0, h[1] = dt_air;
					pony_linal_kalman_update(y,S,K, z,h,s, 2);
					air_alt_last[dev] = pony->air->alt;
				}
				if (pony->air->vv_valid && pony->imu[dev].sol.v_valid) { // vertical velocity data present
					z = pony->air->vv - pony->imu[dev].sol.v[2];
					s = (pony->air-> vv_std > 0) ? (pony->air-> vv_std) : vvs[dev]; // a priori covariance: either specified, or the same as for inertial data
					h[0] = 0, h[1] = 1;
					pony_linal_kalman_update(y,S,K, z,h,s, 2);
				}
			}
			// check for gnss data
			if (pony->gnss != NULL)
				for (r = 0; r < n_gnss; r++) {
					// skip uninitialized subsystems, or ones with no solution ready
					if (pony->gnss[r].cfg == NULL || (!pony->gnss[r].sol.llh_valid && !pony->gnss[r].sol.v_valid))
						continue;
					// time handling
					if (t0_gnss[dev][r] < 0 && pony->gnss[r].sol.llh_valid) {
						gnss_alt_last[dev][r] = pony->gnss[r].sol.llh[2];
						dt_gnss               = 0; // vertical velocity will not be observable
					}
					else
						dt_gnss = pony->imu[dev].t - t0_gnss[dev][r];
					t0_gnss[dev][r] = pony->imu[dev].t;
					// vertical channel update
					if (pony->gnss[r].sol.llh_valid && pony->imu[dev].sol.llh_valid) { // altitude data present
						// altitude
						s = sqrt2*( (pony->gnss[r].sol.x_std > 0) ? (pony->gnss[r].sol.x_std) : pony->gnss[r].settings.code_sigma ); // a priori covariance: either provided with solution, or the same as for measurements
						z = pony->gnss[r].sol.llh[2] + gnss_alt_last[dev][r]; // decorrelated with velocity information
						h[0] = 2, h[1] = -dt_gnss;
						pony_linal_kalman_update(y,S,K, z,h,s, 2);
						// altitude rate of change
						z = pony->gnss->sol.llh[2] - gnss_alt_last[dev][r]; // decorrelated with altitude information
						h[0] = 0, h[1] = dt_gnss;
						pony_linal_kalman_update(y,S,K, z,h,s, 2);
						gnss_alt_last[dev][r] = pony->gnss->sol.llh[2];
					}
					if (pony->gnss[r].sol.v_valid && pony->imu[dev].sol.v_valid) { // vertical velocity data present
						z = pony->gnss[r].sol.v[2] - pony->imu[dev].sol.v[2];
						s = (pony->gnss[r].sol.v_std > 0) ? (pony->gnss[r].sol.v_std) : pony->gnss[r].settings.doppler_sigma; // a priori covariance: either provided with solution, or the same as for measurements
						h[0] = 0, h[1] = 1;
						pony_linal_kalman_update(y,S,K, z,h,s, 2);
					}
				}
			// vertical channel correction
			s = sqrt(S[0]*S[0] + S[1]*S[1]);  // altitude stdev estimate
			w = s + S[2]*dt;                  // sum of altitude stdev and velocity contribution into stdev
			if (pony->imu[dev].sol.llh_valid) 
				pony->imu[dev].sol.llh[2] = y[0]; // replace altitude by its estimated value
			if (pony->imu[dev].sol.  v_valid) {
				pony->imu[dev].sol.llh[2] += 2*s      /w*(y[1]-v)*dt; // weighted correction to hold dx/dt = v
				pony->imu[dev].sol.  v[2] = y[1]; // replace vertical velocity by its estimated value
			}
			if (pony->imu[dev].sol.llh_valid) 
				pony->imu[dev].sol.  v[2] += 2*S[2]*dt/w*(y[0]-x); // weighted correction to hold dx/dt = v
		}

	}

}

/* pony_ins_motion_size_effect - pony plugin
	
	Compensates for accelerometer cluster size effect, i.e. the spatial separation 
	between accelerometer proof masses, also known as "accelerometer lever arm compensation".
	Uses angular rate and computed angular acceleration, taken as rate increment over the time step.
	Preforms the compensation for all initialized IMU devices with accelerometer positions specified.
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
void pony_ins_motion_size_effect(void) 
{

	const char 
		accel_pos_token[] = "accel_pos"; // accelerometer proof mass coordinates parameter name in configuration
	const double 
		pos_range[]       = {-1, +1},    // accelerometer proof mass coordinates range
		eps               = 1.0/0x4000;  // 2^-14, smallest positive normal number in IEEE754 half-precision format
				
	static size_t ndev;  // number of IMU devices
	static double 					     
		 *t0     = NULL, // previous time												 for each IMU device
		(*w0)[3] = NULL, // previous angular rate to calculate acceleration				 for each IMU device
		(*r )[9] = NULL; // accelerometer proof mass coordinates (lever arm coordinates) for each IMU device

	double 
		dt,              // time step
		*w,              // angular rate shortcut
		dw[3];           // computed angular acceleration
	size_t 
		i, j, i1, i2,    // common index variables
		dev;             // current IMU device


	// validate IMU subsystem
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// allocate memory
		ndev = pony-> imu_count;
		t0 = (double *    )calloc(ndev, sizeof(double   ));
		w0 = (double(*)[3])calloc(ndev, sizeof(double[3]));
		r  = (double(*)[9])calloc(ndev, sizeof(double[9]));
		if (t0 == NULL || w0 == NULL || r == NULL) {
			pony->mode = -1;
			return;
		}
		// init for each IMU device
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// parse parameters from configuration	
			pony_ins_motion_parse_double_2array(r[dev],3,3, accel_pos_token, pony->imu[dev].cfg,pony->imu[dev].cfglength, (double*)pos_range,0);
			// reset previous time
			t0[dev] = -1;
		}

	}

	else if (pony->mode < 0) {	// termination
		
		// free allocated memory
		pony_ins_motion_free_null((void**)(&t0));
		pony_ins_motion_free_null((void**)(&w0));
		pony_ins_motion_free_null((void**)(&r ));

	}

	else						// regular processing
	{
		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems, or ones with no inertial measurements ready
			if (pony->imu[dev].cfg == NULL || !pony->imu[dev].f_valid || !pony->imu[dev].w_valid)
				continue;
			w = pony->imu[dev].w;
			// time variables
			if (t0[dev] < 0) { // first touch
				t0[dev] = pony->imu[dev].t;
				for (i = 0; i < 3; i++)
					w0[dev][i] = w[i];
				continue;
			}
			dt      = pony->imu[dev].t - t0[dev];
			t0[dev] = pony->imu[dev].t;
			if (dt < eps)
				return; // do nothing if no proper time increment detected
			// angular acceleration
			for (i = 0; i < 3; i++) {
				dw     [i] = (w[i] - w0[dev][i])/dt;
				w0[dev][i] =  w[i];
			}
			// size effect compensation
			for (i = 0, j = 0; i < 3; i++, j += 3) {
				i1 = (i+1)%3;
				i2 = (i+2)%3;
				pony->imu[dev].f[i] -= -(w[i1]*w[i1] + w[i2]*w[i2])*r[dev][j+i] + w[i]*w[i1]*r[dev][j+i1] + w[i]*w[i2]*r[dev][j+i2] + dw[i1]*r[dev][j+i2] - dw[i2]*r[dev][j+i1];
			}
		}

	}

}





// service functions
	// navigation
double pony_ins_motion_pos_step_enu(
	double (*llh)[3], double (*W)[3], char *W_valid, // out
	const double v[3], const double dt, const double sin_lat, const double cos_lat) // in
{
	enum {E, N, U};

	const double
		eps = 1.0/0x0100; // 2^-8, guaranteed non-zero value in IEEE754 half-precision format

	double 
		dx[3], phi,   // coordinate increment and latitude
		Rn_h, Re_h,   // south-to-north and east-to-west curvature radii, altitude-adjusted
		e2s2, e4s4,   // e^2*sin(phi)^2, (e^2*sin(phi)^2)^2
		l0, l1, dlon; // distances to pole at start and end of the time step
	size_t i;

	phi = (*llh)[N];
	// ellipsoid geometry
	e2s2 = pony->imu_const.e2*sin_lat*sin_lat;
	e4s4 = e2s2*e2s2;					
	Re_h = pony->imu_const.a*(1 + e2s2/2 + 3*e4s4/8);                            // Taylor expansion within 0.5 m, not altitude-adjusted
	Rn_h = Re_h*(1 - pony->imu_const.e2)*(1 + e2s2 + e4s4 + e2s2*e4s4) + (*llh)[U]; // Taylor expansion within 0.5 m
	Re_h += (*llh)[U];                                                           // adjust for altitude
	// position increment
	for (i = 0; i < 3; i++)
		dx[i] = v[i]*dt;
	// angular rate of navigation frame relative to the Earth
	(*W)[E] = -v[N]/Rn_h, *W_valid |= ~(0x01<<E);
	(*W)[N] =  v[E]/Re_h, *W_valid |= ~(0x01<<N);
	// position update
	if (cos_lat < eps) { // check for Earth pole proximity
		// solve triangle
		l0 = (pony->imu_const.pi_2 - fabs(phi))*Rn_h; // distance to pole at start
		l1 = l0 - pony_ins_motion_sign(phi)*dx[N];
		l1 = sqrt(l1*l1 + dx[E]*dx[E]); // not less than |dx_E|, distance to pole at end
		dlon = (l1 > 0) ? asin(dx[E]/l1) : 0.0;
		(*llh)[N] = pony_ins_motion_sign(phi)*(pony->imu_const.pi_2 - l1/Rn_h);
		// navigation frame
		(*W)[U] = 0, *W_valid &= ~(0x01<<U); // freeze, drop validity for the third component
	}
	else {
		// coordinates
		dlon = dx[E]/(Re_h*cos_lat);
		(*llh)[N] += dx[N]/Rn_h;
		// navigation frame
		(*W)[U] = (*W)[N]*sin_lat/cos_lat;
		*W_valid |= ~(0x01<<U);
	}
	(*llh)[E] += dlon;
	(*llh)[U] += dx[U];
	// adjust longitude into range
	while ((*llh)[E] < -pony->imu_const.pi) (*llh)[E] += pony->imu_const.pi2;
	while ((*llh)[E] > +pony->imu_const.pi) (*llh)[E] -= pony->imu_const.pi2;

	return dlon;
}

void pony_ins_motion_acc_enu2llazw(
	double (*dvcor)[3], double (*dvrel)[3], double (*dvg)[3], // out
	const double W[3], const double V[3], const double f[3], const double g[3], const double sin_lat, const double cos_lat, const double sin_chi, const double cos_chi) // in
{
	enum {E, N, U, dim, hdim = N+1, hdim2 = hdim*hdim};
	
	double 
		u[dim], Wu2[dim],
		C[hdim2];
	size_t i;

	// Earth angular velocity vector in East-North-Up
	u[N] = pony->imu_const.u*cos_lat;
	u[U] = pony->imu_const.u*sin_lat;
	// Coriolis acceleration
	Wu2[E] = W[E];
	Wu2[N] = W[N] + u[N]*2;
	Wu2[U] =        u[U]*2;
	pony_linal_cross3x1(u, V, Wu2);
	// azimuth rotation matrix
	C[0*hdim + E] =  cos_chi; C[0*hdim + N] = sin_chi;
	C[1*hdim + E] = -sin_chi; C[1*hdim + N] = cos_chi;
	// rotate Coriolis acceleration
	pony_linal_mmul(*dvcor, C, u, hdim, hdim, 1);
	(*dvcor)[U] = u[U];
	// proper acceleration
	pony_linal_mmul(*dvrel, C, f, hdim, hdim, 1);
	(*dvrel)[U] = f[U];
	// gravity acceleration
	pony_linal_mmul(*dvg, C, g, hdim, hdim, 1);
	(*dvg)[U] = g[U];
}

	// parsing
double pony_ins_motion_parse_double(const char* token, char* src, const size_t len, double* range, const double default_value) 
{

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
			// parse value
			val = atof(src + i);
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

double pony_ins_motion_parse_double_imu_or_settings(const char* token, const size_t imu_dev, double range[2], const double default_value)
{
	char *src;
	size_t len;

	src = pony->imu[imu_dev].cfg;
	len = pony->imu[imu_dev].cfglength;
	if (pony_locate_token(token, src, len, '=') == NULL) {
		src = pony->cfg_settings;
		len = pony->settings_length;
	}
	
	return pony_ins_motion_parse_double(token, src, len, range, default_value);

}

	// memory handling
void pony_ins_motion_free_null(void** ptr)
{
	if (ptr == NULL || *ptr == NULL)
		return;

	free(*ptr);
	*ptr = NULL;
}
