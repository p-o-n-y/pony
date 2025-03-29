// Feb-2025
/*	pony_ins_alignment

	pony plugins for ins initial alignment (initial attitude matrix determination):

	- pony_ins_alignment_static
		Conventional averaging of accelerometer and gyroscope outputs.
		Then constructing the attitude matrix out of those averages.
		Also calculates attitude angles (roll, pitch, yaw=true heading), and quaternion.
		Sets velocity vector to zero, as it is assumed for this type of initial alignment.
		Performs the alignment for all initialized IMU devices.
		Recommended for navigation/tactical-grade systems on a highly stable static base,
		e.g. turntable or stabilized plate.

	- pony_ins_alignment_rotating
		Approximation of ground reaction force rotating along with the Earth in an inertial reference frame.
		Then estimating northern direction via ground reaction force displacement.
		Also calculates attitude angles (roll, pitch, yaw=true heading), and quaternion.
		Sets velocity vector to zero and its validity flag up, as this is assumed for this type of initial alignment.
		Performs the alignment for all initialized IMU devices.
		Recommended for navigation-grade systems on a rotating base,
		allowing vibrations with zero average acceleration,
		e.g. an airplane standing still on the ground with engine(s) running.

	- pony_ins_alignment_rotating_rpy
		The same as pony_ins_alignment_rotating, but attitude matrix is calculated using attitude angles
		(roll, pitch and yaw=true heading). Does not allow the first instrumental axis to point upwards.
		Also sets velocity vector to zero.
*/


#include <stdlib.h>
#include <math.h>

#include "../pony.h"


// pony bus version check
#define PONY_INS_ALIGNMENT_BUS_VERSION_REQUIRED 18
#if PONY_BUS_VERSION < PONY_INS_ALIGNMENT_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif


// service functions
	// parsing
double pony_ins_alignment_parse_double_imu_or_settings(const char* token, const size_t imu_dev, const double default_value);
	// memory handling
void pony_ins_alignment_free_null(void** ptr);


/* pony_ins_alignment_static - pony plugin

	Conventional averaging of accelerometer and gyroscope outputs.
	Then constructing the attitude matrix out of those averages.
	Also calculates attitude angles (roll, pitch, yaw=true heading), and quaternion.
	Performs the alignment for all initialized IMU devices.
	Recommended for navigation-grade systems on a highly stable static base,
	e.g. turntable or stabilized plate. Sets velocity vector to zero,
	as it is assumed for this type of initial alignment.

	description:
		    [  <w> x <f>  | <f> x (<w> x <f>) |  <f>  ]
		L = [ ----------- | ----------------- | ----- ],
		    [ |<w> x <f>| | |<f>| |<w> x <f>| | |<f>| ]
		where <w> and <f> are the angular rate and specific force vectors, respectively,
		as measured by gyroscopes and accelerometers, averaged over an initial alignment period
		specified in the configuration string, u x v is vector cross product

	uses:
		pony->imu->t
		pony->imu->w
		pony->imu->w_valid
		pony->imu->f
		pony->imu->f_valid

	changes:
		pony->imu->sol.L
		pony->imu->sol.L_valid
		pony->imu->sol.q
		pony->imu->sol.q_valid
		pony->imu->sol.rpy
		pony->imu->sol.rpy_valid
		pony->imu->sol.v
		pony->imu->sol.v_valid

	cfg parameters:
		alignment
		or
		{imu: alignment} - imu initial alignment end time, sec
			type :   floating point
			range:   0+ to +inf (positive decimal)
			default: 300
			example: {imu: alignment = 900}
			note:    if specified for the particular IMU device, the value overrides one from the common configuration settings (outside of groups)
*/
void pony_ins_alignment_static(void)
{

	const char   t0_token[] = "alignment"; // alignment end time parameter name in configuration
	const double t0_default = 300;         // default alignment end time

	static size_t ndev;    // number of IMU devices
	static double
		(*w0)[3] = NULL,    // average angular rate measured by gyroscopes <w>              for each device
		(*f0)[3] = NULL,    // average specific force vector measured by accelerometers <f> for each device
		 *t0     = NULL,    // alignment end time                                            for each device
		  v [3]  = {0,0,0}, // cross product <w> x <f>
		  vv[3]  = {0,0,0}, // double cross product <f> x (<w> x <f>)
		  d [3]  = {0,0,0}; // vector magnitudes
	static unsigned long
		 *n      = NULL;    // measurement counter for each device                          for each device

	double *L;              // pointer to attitude matrix in solution
	double  n1_n;           // (n - 1)/n
	size_t  i, j, dev;      // index variables


	// validate IMU subsystem
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// allocate memory
		ndev = pony->imu_count;
		w0 = (double       (*)[3])calloc(ndev, sizeof(double[3]    ));
		f0 = (double       (*)[3])calloc(ndev, sizeof(double[3]    ));
		t0 = (double        *    )calloc(ndev, sizeof(double       ));
		n  = (unsigned long *    )calloc(ndev, sizeof(unsigned long));
		if (w0 == NULL || f0 == NULL || t0 == NULL || n == NULL) {
			pony->mode = -1;
			return;
		}
		// init for all devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// init values
			n[dev] = 0; // drop the counter on init
			t0[dev] = pony_ins_alignment_parse_double_imu_or_settings(t0_token, dev, -1);
			if (t0[dev] <= 0)
				t0[dev] = t0_default;
		}

	}

	else if (pony->mode < 0) {	// terminate

		// free allocated memory
		pony_ins_alignment_free_null((void**)(&w0));
		pony_ins_alignment_free_null((void**)(&f0));
		pony_ins_alignment_free_null((void**)(&t0));
		pony_ins_alignment_free_null((void**)(&n ));

	}

	else						// main cycle
	{
		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// check if alignment end time exceeded
			if (pony->imu[dev].t > t0[dev])
				return;
			// drop validity flags
			pony->imu[dev].sol.L_valid   = 0;
			pony->imu[dev].sol.q_valid   = 0;
			pony->imu[dev].sol.rpy_valid = 0;
			// renew averages, cross products and their lengths
			if (pony->imu[dev].w_valid && pony->imu[dev].f_valid) {
				n[dev]++;
				n1_n = (n[dev] - 1.0)/n[dev];
				for (i = 0; i < 3; i++) {
					w0[dev][i] = w0[dev][i]*n1_n + pony->imu[dev].w[i]/n[dev];
					f0[dev][i] = f0[dev][i]*n1_n + pony->imu[dev].f[i]/n[dev];
				}

				pony_linal_cross3x1(v , w0[dev], f0[dev]); d[0] = pony_linal_vnorm(v      , 3);
				pony_linal_cross3x1(vv, f0[dev], v      ); d[1] = pony_linal_vnorm(vv     , 3);
				                                           d[2] = pony_linal_vnorm(f0[dev], 3);
			}
			// check for singularity
			for (i = 0; i < 3; i++)
				if (d[i] <= 0) // attitude undefined
					continue;
			// renew attitude matrix
			L = pony->imu[dev].sol.L;
			for (i = 0, j = 0; j < 3; i += 3, j++) {
				L[i + 0] = v      [j]/d[0];
				L[i + 1] = vv     [j]/d[1];
				L[i + 2] = f0[dev][j]/d[2];
			}
			pony->imu[dev].sol.L_valid   = 1;
			// renew angles
			pony_linal_mat2rpy (pony->imu[dev].sol.rpy, L);
			pony->imu[dev].sol.rpy_valid = 1;
			// renew quaternion
			pony_linal_mat2quat(pony->imu[dev].sol.q  , L);
			pony->imu[dev].sol.q_valid   = 1;
			// set velocity equal to zero, as the static alignment implies
			for (i = 0; i < 3; i++)
				pony->imu[dev].sol.v[i] = 0;
			pony->imu[dev].sol.v_valid = 1;
		}

	}

}

/* pony_ins_alignment_rotating - pony plugin

	Approximation of ground reaction force rotating along with the Earth in an inertial reference frame.
	Then estimating northern direction via ground reaction force displacement.
	Also calculates attitude angles (roll, pitch, yaw=true heading), and quaternion.
	Sets velocity vector to zero and its validity flag up, as this is assumed for this type of initial alignment.
	Performs the alignment for all initialized IMU devices.
	Recommended for navigation-grade systems on a rotating base,
	allowing vibrations with zero average acceleration,
	e.g. an airplane standing still on the ground with engine(s) running.

	description:
		algorithm may be found in a separate document [A. Kozlov]

		          takes current latitude from IMU solution,    if valid,
		otherwise takes current latitude from hybrid solution, if valid,
		otherwise provides approximate azimuth which has error growing with time and validity flag down

	uses:
		pony->imu->t
		pony->imu->w
		pony->imu->w_valid
		pony->imu->f
		pony->imu->f_valid
		pony->imu->sol.llh
		pony->imu->sol.llh_valid
		pony->     sol.llh
		pony->     sol.llh_valid

	changes:
		pony->imu->sol.L
		pony->imu->sol.L_valid
		pony->imu->sol.q
		pony->imu->sol.q_valid
		pony->imu->sol.rpy
		pony->imu->sol.rpy_valid
		pony->imu->sol.v
		pony->imu->sol.v_valid

	cfg parameters:
		alignment
		or
		{imu: alignment} - imu initial alignment end time, sec
			type :   floating point
			range:   0+ to +inf (positive decimal)
			default: 300
			example: {imu: alignment = 300}
			note:    if specified for the particular IMU device, the value overrides one from the common configuration settings (outside of groups)
*/
void pony_ins_alignment_rotating(void)
{

	enum dim {
		E, N, U,
		m   = 3,         // approximation coefficient count
		mm  = m*(m+1)/2, // upper-triangular matrix elements
	};

	const char
		t1_token[] = "alignment"; // alignment end time parameter name in configuration
	const double
		t1_default = 300,         // default alignment end time
		v_std      = 1,           // velocity integral standard deviation
		k_sigma    = 3,           // measurement residual criterion coefficient
		S0         = 6e6;         // starting covariances

	static size_t ndev;           // number of IMU devices
	static double
		 *t0           = NULL,    // alignment start time                                                       for each device
		 *t1           = NULL,    // alignment end time                                                         for each device
		 *t_           = NULL,    // previous time                                                              for each device
		(* vz0)[3]     = NULL,    // velocity integral                                                          for each device
		(*Azz0)[9]     = NULL,    // transition matrix from inertial to instrumental reference frame            for each device
		(*x   )[3][m ] = NULL,    // approximation coefficients                                                 for each device
		(*S   )[3][mm] = NULL,    // upper-triangular part of covariance's Cholesky factorization of covariance for each device
		  K       [m ] = {0,0,0}, // approximation Kalman gain
		  h       [m ] = {0,0,0}, // approximation model coefficients
		  hd      [m ] = {1,0,0}, // approximation model derivative coefficients for base epoch
		 u, _u2;                  // Earth's rotation rate, and reciprocal square

	double
		fz0   [3],  // accelerometers output and their approximation
		fza   [3],  // approximation at current time in instrumental frame
		fxf   [3],  // cross-product orthogonal vector
		fxfxf [3],  // double cross product
		a     [3],  // Euler vector of rotation from inertial to instrumental reference frame
		Lt    [6],  // estimated attitude at t, two first columns
		C[9], D[9], // intermediate transition matrices
		  t,  dt,   // time since the alignment has started and time increment
		 ut, sut_u, // Earth rotation angle and its normalized sine
		sut,  cut,  // sine and cosine of the Earth's rotation angle
		schi, cchi, // azimuth rotation angle's trig functions
		lat, slat,  // latitude and its sine
		n;          // vector norm
	char valid;     // validity flag
	size_t
		dev,        // current IMU device index
		i, j;       // common indexing variables


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// allocate memory
		ndev = pony->imu_count;
		t0   = (double  *        )calloc(ndev, sizeof(double       ));
		t1   = (double  *        )calloc(ndev, sizeof(double       ));
		t_   = (double  *        )calloc(ndev, sizeof(double       ));
		 vz0 = (double (*)[3]    )calloc(ndev, sizeof(double[3]    ));
		Azz0 = (double (*)[9]    )calloc(ndev, sizeof(double[9]    ));
		x    = (double (*)[3][m ])calloc(ndev, sizeof(double[3][m ]));
		S    = (double (*)[3][mm])calloc(ndev, sizeof(double[3][mm]));
		if (   t0   == NULL || t1  == NULL || t_ == NULL
			|| x    == NULL || S   == NULL
			|| Azz0 == NULL) {
			pony->mode = -1;
			return;
		}
		// init for all devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// reset time variables
			t0[dev] = -1;
			t1[dev] = pony_ins_alignment_parse_double_imu_or_settings(t1_token, dev, -1);
			if (t1[dev] <= 0)
				t1[dev] = t1_default;
			t_[dev] = -1;
		}
		// constants
		u   = pony->imu_const.u;
		_u2 = 1/(u*u);

	}

	else if (pony->mode < 0) {	// terminate

		// free allocated memory
		pony_ins_alignment_free_null((void**)(&t0  ));
		pony_ins_alignment_free_null((void**)(&t1  ));
		pony_ins_alignment_free_null((void**)(&t_  ));
		pony_ins_alignment_free_null((void**)(&x   ));
		pony_ins_alignment_free_null((void**)(&S   ));
		pony_ins_alignment_free_null((void**)(&Azz0));

	}

	else						// main cycle
	{
		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// check if alignment has ended
			if (pony->imu[dev].t > t1[dev])
				continue;
			// drop validity flags
			pony->imu[dev].sol.L_valid		= 0;
			pony->imu[dev].sol.q_valid		= 0;
			pony->imu[dev].sol.rpy_valid	= 0;
			// check for required data present
			if (   !pony->imu[dev].w_valid   // check if angular rate measurements are available
				|| !pony->imu[dev].f_valid)  // check if accelerometer measurements are available
				continue;
			// initialization procedures
			if (t_[dev] < 0) {
				// time init
				t0[dev] = pony->imu[dev].t;
				t_[dev] = t0[dev];
				// identity attitude matrix
				for (i = 0; i < 9; i++)
					Azz0[dev][i] = (i%4 == 0) ? 1 : 0;
				// starting covariance
				for (j = 0; j < m; j++)
					K[j] = S0;
				for (i = 0; i < 3; i++) {
					pony_linal_diag2u(S[dev][i], K, m);
					// starting estimates
					for (j = 0; j < m; j++)
						x[dev][i][j] = 0;
					// velocity integral
					vz0[dev][i] = 0;
				}
				continue;
			}
			// set assumed validity (may be dropped later)
			valid = 1;
			// time increments
			dt = pony->imu[dev].t - t_[dev];
			if (dt <= 0)
				continue; // invalid increment, abort
			t_[dev] = pony->imu[dev].t;
			t       = pony->imu[dev].t - t0[dev];
			// advance instrumental frame to time step midpoint using Rodrigues' rotation formula:
			//    D = Azz0(t+dt/2) = (E + [w x]*sin(|w*dt/2|)/|w| + [w x]^2*(1-cos(|w*dt/2|)/|w*dt|^2)*Azz0(t)
			for (i = 0; i < 3; i++)
				a[i] = pony->imu[dev].w[i]*dt/2; // a = w*dt/2
			pony_linal_eul2mat(C,a);             // C = E + [a x]*sin(|a|)/|a| + [a x]^2*(1-cos(|a|)/|a|^2
			pony_linal_mmul(D, C,Azz0[dev], 3,3,3);
			// transform accelerometers into inertial frame at time step midpoint fz0 = Azz0(t+dt/2)^T*fz
			pony_linal_mmul1T(fz0, D,pony->imu[dev].f, 3,3,1);
			// update instrumental frame to the end of time step using Rodrigues' rotation formula:
			//    Azz0(t+dt) = (E + [w x]*sin(|w*dt/2|)/|w| + [w x]^2*(1-cos(|w*dt/2|)/|w*dt|^2)*Azz0(t+dt/2)
			pony_linal_mmul(Azz0[dev], C,D, 3,3,3);
			// latitude
			lat = 
				(pony->imu[dev].sol.llh_valid ? pony->imu[dev].sol.llh[N] : // inertial coordinates
				(pony->         sol.llh_valid ? pony->         sol.llh[N] : // hybrid coordinates
					0));                                                    // neither - take zero latitude yielding approximate azimuth
			slat = sin(lat);
			// Earth rotation
			 ut   = u*t;
			sut   = sin(ut);
			cut   = cos(ut);
			sut_u = sut/u;
			// approximation model coefficients
			h[0] =  t             ;
			h[1] = (1 - cut  )*_u2;
			h[2] = (t - sut_u)*_u2;
			// approximation model derivative coefficients for base epoch
			// after ~6 hours, when ut exceeds pi/2, geometry starts to degrade, i.e. cross products below start decreasing in magnitude,
			// so base epoch is no longer at t = t0, but at t2 such that u(t-t2) = pi/2, which simplifies calculations and provides roughly the best geometry
			if (ut <= pony->imu_const.pi_2) {
				hd[1] = 0; // h0[0] is always one
				hd[2] = 0;
				schi = slat*(1-cut);
				cchi =         sut ;
			}
			else {
				hd[1] =   -cut/u   ; // h0[0] is always one
				hd[2] = (1-sut)*_u2;
				schi = slat;
				cchi = 1   ;
			}
			n = sqrt(schi*schi + cchi*cchi); // never zero since 0 < dt <= ut <= pi/2 so that 0 < sin(ut) <= cchi <= 1
			schi /= n, cchi /= n;
			// approximation
			for (i = 0; i < 3; i++) {
				// update velocity integral
				vz0[dev][i] += fz0[i]*dt;
				// estimate approximation coefficient
				if (pony_linal_check_measurement_residual(x[dev][i],S[dev][i], vz0[dev][i],h,v_std, k_sigma, m)) // check measurement residual against 3-sigma with current estimate
					pony_linal_kalman_update(x[dev][i],S[dev][i],K, vz0[dev][i],h,v_std, m);                     // update estimates using v = h*x + dv, v_std = sqrt(E[dv^2])
				// average ground reaction specific force approximation at base epoch   in inertial frame
				fxf[i] = x[dev][i][0] + x[dev][i][1]*hd[1] + x[dev][i][2]*hd[2]; // velocity integral approximation derivative taken at base epoch
				// average ground reaction specific force approximation at current time in inertial frame
				fz0[i] = x[dev][i][0] + x[dev][i][1]*sut_u + x[dev][i][2]*h [1]; // velocity integral approximation derivative taken at t
			}
			// transition to current instrumental frame (fza = Azz0*fz0)
			pony_linal_mmul(fza, Azz0[dev],fz0, 3,3,1); // average ground reaction at current time
			pony_linal_mmul(fz0, Azz0[dev],fxf, 3,3,1); // average ground reaction at base epoch
			// vertical ort
			n = pony_linal_vnorm(fza, 3);              // n = |fza|
			if (n > 0)
				for (i = 0; i < 3; i++)
					pony->imu[dev].sol.L[i*3 + U] = fza[i]/n; // normalize and calculate attitude 3rd column
			else
				valid = 0;
			// cross-product to obtain orthogonal vector
			// for ut << 1 (small Earth's rotation angles, i.e. short alignment time) points approximately northward
			pony_linal_cross3x1(fxf, fz0,fza);
			n = pony_linal_vnorm(fxf, 3);
			if (n > 0)
				for (i = 0; i < 3; i++)
					Lt[i*2 + 1] = fxf  [i]/n; // normalize and calulate 2nd column of intermediate attitude matrix (subject to azimuth rotation)
			else
				valid = 0;
			// third ort as a cross-product
			pony_linal_cross3x1(fxfxf, fxf,fza);
			n = pony_linal_vnorm(fxfxf, 3);
			if (n > 0)
				for (i = 0; i < 3; i++)
					Lt[i*2 + 0] = fxfxf[i]/n; // normalize and calulate 1st column of intermediate attitude matrix (subject to azimuth rotation)
			else
				valid = 0;							
			// rotate in azimuth so that fza and fz0 match
			for (i = 0; i < 3; i++) {
				pony->imu[dev].sol.L[i*3+E] =  Lt[i*2+0]*cchi + Lt[i*2+1]*schi;
				pony->imu[dev].sol.L[i*3+N] = -Lt[i*2+0]*schi + Lt[i*2+1]*cchi;
			}
			pony->imu[dev].sol.  L_valid = valid;
			// renew angles: roll, pitch and yaw=true heading
			pony_linal_mat2rpy (pony->imu[dev].sol.rpy, pony->imu[dev].sol.L);
			pony->imu[dev].sol.rpy_valid = valid;
			// renew quaternion
			pony_linal_mat2quat(pony->imu[dev].sol.q  , pony->imu[dev].sol.L);
			pony->imu[dev].sol.  q_valid = valid;
			// set velocity equal to zero, as the rotating base implies
			for (i = 0; i < 3; i++)
				pony->imu[dev].sol.v[i] = 0;
			pony->imu[dev].sol.v_valid = 1;
		}

	}

}

/* pony_ins_alignment_rotating_rpy - pony plugin, legacy version, not worked on anymore

	The same as pony_ins_alignment_rotating, but attitude matrix is calculated using attitude angles
	(roll, pitch and yaw=true heading). Does not allow the first instrumental axis to point upwards.
	Sets velocity vector to zero, as it is assumed for this type of initial alignment.
	Performs the alignment for all initialized IMU devices.

	description:
		algorithm may be found in a separate document [A.A. Golovan]

		takes latitude from imu solution, if valid
		else takes latitude from hybrid solution
		otherwise quits

	uses:
		pony->imu->t
		pony->imu->w
		pony->imu->w_valid
		pony->imu->f
		pony->imu->f_valid
		pony->imu->sol.llh
		pony->imu->sol.llh_valid
		pony->     sol.llh
		pony->     sol.llh_valid

	changes:
		pony->imu->sol.L
		pony->imu->sol.L_valid
		pony->imu->sol.q
		pony->imu->sol.q_valid
		pony->imu->sol.rpy
		pony->imu->sol.rpy_valid
		pony->imu->sol.v
		pony->imu->sol.v_valid

	cfg parameters:
		alignment
		or
		{imu: alignment} - imu initial alignment end time, sec
			type :   floating point
			range:   0+ to +inf (positive decimal)
			default: 300
			example: {imu: alignment = 300}
			note:    if specified for the particular IMU device, the value overrides one from the common configuration settings (outside of groups)
*/
void pony_ins_alignment_rotating_rpy(void)
{

	enum dim {
		m   = 3,         // approximation coefficient count
		mm  = m*(m+1)/2, // upper-triangular matrix elements
		lat = 1          // latitude index in coordinates array
	};

	const char
		t1_token[] = "alignment"; // alignment end time parameter name in configuration
	const double
		t1_default = 300,         // default alignment end time
		v_std      = 1,           // velocity integral standard deviation
		k_sigma    = 3,           // measurement residual criterion coefficient
		S0         = 6e6;         // starting covariances

	static size_t ndev;                         // number of IMU devices
	static double
		 *t0           = NULL,                  // alignment start time                                                       for each device
		 *t1           = NULL,                  // alignment end time                                                         for each device
		 *t_           = NULL,                  // previous time                                                              for each device
		 *slt          = NULL,                  //   sine of latitude                                                         for each device
		 *clt          = NULL,                  // cosine of latitude                                                         for each device
		(* vz0)[3]     = NULL,                  // velocity integral                                                          for each device
		(*Azz0)[9]     = NULL,                  // transition matrix from inertial to instrumental reference frame            for each device
		(*x   )[3][m ] = NULL,                  // approximation coefficients                                                 for each device
		(*S   )[3][mm] = NULL,                  // upper-triangular part of covariance's Cholesky factorization of covariance for each device
		  K       [m ] = {0,0,0},               // approximation Kalman gain
		  h       [m ] = {0,0,0},               // approximation model matrix
		 fz0   [3]     = {0,0,0},               // accelerometers output and their approximation in inertial frame
		 fz0a0 [3]     = {0,0,0},               // approximation at t0
		 fza   [3]     = {0,0,0},               // approximation at current time in instrumental frame
		 rpy0  [3]     = {0,0,0},               // attitude angles at start
		 a     [3]     = {0,0,0},               // Euler vector of rotation from inertial to instrumental reference frame
		 C     [9]     = {1,0,0, 0,1,0, 0,0,1}, // intermediate transition matrix
		 D     [9]     = {1,0,0, 0,1,0, 0,0,1}, // intermediate transition matrix
		 L0    [9]     = {1,0,0, 0,1,0, 0,0,1}, // estimated attitude at t0
		 Lt    [9]     = {1,0,0, 0,1,0, 0,0,1}, // estimated attitude at t
		 u, u2;                                 // Earth's rotation rate, and squared, shortcuts

	double
		  t,        // time since the alignment has started
		 dt,        // time increment
		 ut,        // Earth rotation angle
		sut, cut,   // sine and cosine of Earth rotation angle
		schi, cchi, // azimuth rotation angle
		n2;         // vector norm squared
	char valid;     // validity flag
	size_t
		dev,        // current IMU device index
		i, j;       // common indexing variables


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

			// allocate memory
		ndev = pony->imu_count;
		t0   = (double  *        )calloc(ndev, sizeof(double       ));
		t1   = (double  *        )calloc(ndev, sizeof(double       ));
		t_   = (double  *        )calloc(ndev, sizeof(double       ));
		slt  = (double  *        )calloc(ndev, sizeof(double       ));
		clt  = (double  *        )calloc(ndev, sizeof(double       ));
		 vz0 = (double (*)[3]    )calloc(ndev, sizeof(double[3]    ));
		Azz0 = (double (*)[9]    )calloc(ndev, sizeof(double[9]    ));
		x    = (double (*)[3][m ])calloc(ndev, sizeof(double[3][m ]));
		S    = (double (*)[3][mm])calloc(ndev, sizeof(double[3][mm]));
		if (   t0   == NULL || t1  == NULL || t_ == NULL
			|| slt  == NULL || clt == NULL
			|| x    == NULL || S   == NULL
			|| Azz0 == NULL) {
			pony->mode = -1;
			return;
		}
		// init for all devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// reset time variables
			t0[dev] = -1;
			t1[dev] = pony_ins_alignment_parse_double_imu_or_settings(t1_token, dev, -1);
			if (t1[dev] <= 0)
				t1[dev] = t1_default;
			t_[dev] = -1;
		}
		// constants
		u  = pony->imu_const.u;
		u2 = u*u;

	}

	else if (pony->mode < 0) {	// terminate

		// free allocated memory
		pony_ins_alignment_free_null((void**)(&t0  ));
		pony_ins_alignment_free_null((void**)(&t1  ));
		pony_ins_alignment_free_null((void**)(&t_  ));
		pony_ins_alignment_free_null((void**)(&slt ));
		pony_ins_alignment_free_null((void**)(&clt ));
		pony_ins_alignment_free_null((void**)(&x   ));
		pony_ins_alignment_free_null((void**)(&S   ));
		pony_ins_alignment_free_null((void**)(&Azz0));

	}

	else						// main cycle
	{
		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// check if alignment has ended
			if (pony->imu[dev].t > t1[dev])
				continue;
			// drop validity flags
			pony->imu[dev].sol.L_valid		= 0;
			pony->imu[dev].sol.q_valid		= 0;
			pony->imu[dev].sol.rpy_valid	= 0;
			// check for required data present
			if (    !pony->imu[dev].      w_valid   // check if angular rate measurements are available
				||  !pony->imu[dev].      f_valid   // check if accelerometer measurements are available
				|| (!pony->imu[dev].sol.llh_valid   // check if imu coordinates are available
				&&  !pony->         sol.llh_valid)) // or if hybrid solution coordinates are available
				continue;
			// initializtion procedures
			if (t_[dev] < 0) {
				// time init
				t0[dev] = pony->imu[dev].t;
				t_[dev] = t0[dev];
				// identity attitude matrix
				for (i = 0; i < 9; i++)
					Azz0[dev][i] = (i%4 == 0) ? 1 : 0;
				// starting covariance
				for (j = 0; j < m; j++)
					K[j] = S0;
				for (i = 0; i < 3; i++) {
					pony_linal_diag2u(S[dev][i], K, m);
					// starting estimates
					for (j = 0; j < m; j++)
						x[dev][i][j] = 0;
					// velocity integral
					vz0[dev][i] = 0;
				}
				// latitude
				if (pony->imu[dev].sol.llh_valid) { // if imu coordinates are available
					slt[dev] = sin(pony->imu[dev].sol.llh[lat]);
					clt[dev] = cos(pony->imu[dev].sol.llh[lat]);
				}
				else if (pony->sol.llh_valid) { // if hybrid solution coordinates are available
					slt[dev] = sin(pony->     sol.llh[lat]);
					clt[dev] = cos(pony->     sol.llh[lat]);
				}
				continue;
			}
			// set assumed validity (may be dropped later)
			valid = 1;
			// time increments
			dt = pony->imu[dev].t - t_[dev];
			if (dt <= 0)
				continue; // invalid increment, abort
			t_[dev] = pony->imu[dev].t;
			t       = pony->imu[dev].t - t0[dev];
			// Earth rotation angle
			 ut = u*t;
			sut = sin(ut);
			cut = cos(ut);
			// update instrumental frame to time midpoint using Rodrigues' rotation formula:
			//    D = Azz0(t+dt/2) = (E + [w x]*sin(|w*dt/2|)/|w| + [w x]^2*(1-cos(|w*dt/2|)/|w*dt|^2)*Azz0(t)
			for (i = 0; i < 3; i++)
				a[i] = pony->imu[dev].w[i]*dt/2; // a = w*dt/2
			pony_linal_eul2mat(C,a);             // C = E + [a x]*sin(|a|)/|a| + [a x]^2*(1-cos(|a|)/|a|^2
			pony_linal_mmul(D, C,Azz0[dev], 3,3,3);
			// transforming accelerometers into inertial frame at time midpoint fz0 = Azz0(t+dt/2)^T*fz
			pony_linal_mmul1T(fz0, D,pony->imu[dev].f, 3,3,1);
			// update instrumental frame to time midpoint using Rodrigues' rotation formula:
			//    Azz0(t+dt) = (E + [w x]*sin(|w*dt/2|)/|w| + [w x]^2*(1-cos(|w*dt/2|)/|w*dt|^2)*Azz0(t+dt/2)
			pony_linal_mmul(Azz0[dev], C,D, 3,3,3);
			// approximation model coefficients
			h[0] =      sut/u    ;
			h[1] = (1 - cut  )/u2;
			h[2] = (t - sut/u)/u2;
			// approximation
			for (i = 0; i < 3; i++) {
				// velocity integral update
				vz0[dev][i] += fz0[i]*dt;
				// approximation coefficient estimates
				if (pony_linal_check_measurement_residual(x[dev][i],S[dev][i], vz0[dev][i],h,v_std, k_sigma, m)) // check measurement residual within 3-sigma with current estimate
					pony_linal_kalman_update(x[dev][i],S[dev][i],K, vz0[dev][i],h,v_std, m);                     // update estimates using v = h*x + dv, v_std = sqrt(E[dv^2])
				// approximation at t0 in inertial frame
				fz0a0[i] = x[dev][i][0];
				// approximation at t  in inertial frame
				fz0  [i] = x[dev][i][0]*cut + x[dev][i][1]*h[0] + x[dev][i][2]*h[1];
			}
			// transition to instrumental frame (fza = Azz0*fz0) and attitude at time t
			pony_linal_mmul(fza, Azz0[dev],fz0, 3,3,1);
			// roll angle via fza components
			if (fza[1] != 0 || fza[2] != 0)
				pony->imu[dev].sol.rpy[0] = -atan2(fza[2], fza[1]);
			// pitch angle via fza components
			n2 = pony_linal_dot(fza,fza, 3);
			if (n2 > 0)
				pony->imu[dev].sol.rpy[1] =  atan2(fza[0], sqrt(fza[1]*fza[1] + fza[2]*fza[2]));
			else
				valid = 0;
			// heading, main branch
			// initial roll and pitch, zero heading
			if (fz0a0[1] !=0 || fz0a0[2] != 0)
				rpy0[0] = -atan2(fz0a0[2], fz0a0[1]);
			n2 = pony_linal_dot(fz0a0,fz0a0, 3);
			if (n2 > 0)
				rpy0[1] = atan2(fz0a0[0], sqrt(fz0a0[1]*fz0a0[1] + fz0a0[2]*fz0a0[2]));
			else
				valid = 0;
			rpy0[2] = 0;
			// "matrix C" `\_("o)_/`
			pony->imu[dev].sol.rpy[2] = 0;
			pony_linal_rpy2mat(Lt, pony->imu[dev].sol.rpy); // Axz^T(t)
				// how to produce matrix A_z_x_t0: via angles at either t0, or t `\_("o)_/`
			pony_linal_rpy2mat(L0, rpy0); // Azx(t0)
			pony_linal_mmul1T(D, Lt,Azz0[dev], 3,3,3);
			pony_linal_mmul  (C, D ,L0       , 3,3,3);
			// "matrix D" `\_("o)_/`	 // only the elements used later
			D[2] =     -sut *clt[dev];
			D[5] = (1 - cut)*slt[dev]*clt[dev];
			// azimuth rotation angle
			schi = -C[2]*D[5] + C[5]*D[2];
			cchi =  C[2]*D[2] + C[5]*D[5];
			// heading(t) `\_("o)_/`
			if (schi != 0 || cchi != 0)
				pony->imu[dev].sol.rpy[2] = atan2(schi, cchi);
			else
				valid = 0;
			pony->imu[dev].sol.rpy_valid = valid;
			// attitude matrix
			pony_linal_rpy2mat (pony->imu[dev].sol.L, pony->imu[dev].sol.rpy);
			pony->imu[dev].sol.L_valid   = valid;
			// renew quaternion
			pony_linal_mat2quat(pony->imu[dev].sol.q, pony->imu[dev].sol.L  );
			pony->imu[dev].sol.q_valid   = valid;
			// set velocity equal to zero, as the rotating base implies
			for (i = 0; i < 3; i++)
				pony->imu[dev].sol.v[i] = 0;
			pony->imu[dev].sol.v_valid = 1;
		}

	}

}


// service functions
	// parsing
double pony_ins_alignment_parse_double_imu_or_settings(const char* token, const size_t imu_dev, const double default_value)
{
	char *cfg_ptr;

	// look for parameter either in specific IMU device configuration
	cfg_ptr = pony_locate_token(token, pony->imu[imu_dev].cfg, pony->imu[imu_dev].cfglength, '=');
	if (cfg_ptr == NULL)
		// or in common settings
		cfg_ptr = pony_locate_token(token, pony->cfg_settings, pony->settings_length, '=');
	// try parsing
	return (cfg_ptr == NULL ? default_value : atof(cfg_ptr));

}

	// memory handling
void pony_ins_alignment_free_null(void** ptr)
{
	if (ptr == NULL || *ptr == NULL)
		return;

	free(*ptr);
	*ptr = NULL;
}
