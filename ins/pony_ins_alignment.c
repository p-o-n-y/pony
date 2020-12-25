// Dec-2020
/*	pony_ins_alignment 
	
	pony plugins for ins initial alignment (initial attitude matrix determination):
	
	- pony_ins_alignment_static
		Conventional averaging of accelerometer and gyroscope outputs.
		Then constructing the attitude matrix out of those averages.
		Also calculates attitude angles (roll, pitch, yaw=true heading), and quaternion.
		Sets velocity vector to zero, as it is assumed for this type of initial alignment.
		Recommended for navigation/tactical-grade systems on a highly stable static base,
		e.g. turntable or stabilized plate.

	- pony_ins_alignment_rotating
		Approximation of gravity vector rotating along with the Earth in an inertial reference frame.
		Then estimating northern direction via gravity vector displacement.
		Also calculates attitude angles (roll, pitch, yaw=true heading), and quaternion.
		Sets velocity vector to zero.
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
#define PONY_INS_ALIGNMENT_BUS_VERSION_REQUIRED 8
#if PONY_BUS_VERSION < PONY_INS_ALIGNMENT_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif

/* pony_ins_alignment_static - pony plugin
	
	Conventional averaging of accelerometer and gyroscope outputs.
	Then constructing the attitude matrix out of those averages.
	Also calculates attitude angles (roll, pitch, yaw=true heading).
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
		{imu: alignment} - imu initial alignment duration, sec
			type :   floating point
			range:   0+ to +inf
			default: 300
			example: {imu: alignment = 900}
*/
void pony_ins_alignment_static(void) {

	const char   t0_token[] = "alignment"; // alignment duration parameter name in configuration
	const double t0_default = 300;         // default alignment duration

	static double 
		w [3]	= {0,0,0},  // average angular rate measured by gyroscopes <w>
		f [3]	= {0,0,0},  // average specific force vector measured by accelerometers <f>
		v [3]	= {0,0,0},  // cross product <w> x <f>
		vv[3]	= {0,0,0},  // double cross product <f> x (<w> x <f>)
		d [3]	= {0,0,0},  // vector magnitudes
		*L	    = NULL,     // pointer to attitude matrix in solution
		t0      = -1;       // alignment duration
	static int n = 0;       // measurement counter

	char   *cfg_ptr;        // pointer to a substring in configuration
	double  n1_n;           // (n - 1)/n
	size_t  i, j;			// common index variables


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// init values
		n = 0;                // drop the counter on init
		L = pony->imu->sol.L; // set pointer to imu solution matrix
		// parse alignment duration from configuration string
		cfg_ptr = pony_locate_token(t0_token, pony->imu->cfg, pony->imu->cfglength, '=');
		if (cfg_ptr != NULL)
			t0 = atof(cfg_ptr);		
		// if not found or invalid, set to default
		if (cfg_ptr == NULL || t0 <= 0)
			t0 = t0_default;

	}
	else if (pony->mode < 0) {	// terminate
		// do nothing
	}
	else						// main cycle
	{
		// check if alignment duration exceeded 
		if (pony->imu->t > t0)
			return;
		// drop validity flags
		pony->imu->sol.L_valid		= 0;
		pony->imu->sol.q_valid		= 0;
		pony->imu->sol.rpy_valid	= 0;
		// renew averages, cross products and their lengths
		if (pony->imu->w_valid && pony->imu->f_valid) {
			n++;
			n1_n = (n - 1.0)/n;
			for (i = 0; i < 3; i++) {
				w[i] = w[i]*n1_n + pony->imu->w[i]/n;
				f[i] = f[i]*n1_n + pony->imu->f[i]/n;
			}

			pony_linal_cross3x1(v , w, f);	d[0] = pony_linal_vnorm(v , 3);
			pony_linal_cross3x1(vv, f, v);	d[1] = pony_linal_vnorm(vv, 3);
											d[2] = pony_linal_vnorm(f , 3);
		}
		// check for singularity
		for (i = 0; i < 3; i++)
			if (d[i] <= 0) // attitude undefined
				return;
		// renew attitude matrix
		for (i = 0, j = 0; j < 3; i += 3, j++) {
			L[i + 0] = v [j]/d[0];
			L[i + 1] = vv[j]/d[1];
			L[i + 2] = f [j]/d[2];
		}
		pony->imu->sol.L_valid   = 1;
		// renew quaternion
		pony_linal_mat2quat(pony->imu->sol.q  , pony->imu->sol.L);
		pony->imu->sol.q_valid   = 1;
		// renew angles
		pony_linal_mat2rpy (pony->imu->sol.rpy, pony->imu->sol.L);
		pony->imu->sol.rpy_valid = 1;
		// set velocity equal to zero, as it is assumed to do so in static alignment
		for (i = 0; i < 3; i++)
			pony->imu->sol.v[i] = 0;
		pony->imu->sol.v_valid = 1;

	}

}

/* pony_ins_alignment_rotating - pony plugin
	
	Approximation of gravity vector rotating along with the Earth in an inertial reference frame.
	Then estimating northern direction via gravity vector displacement.
	Also calculates attitude angles (roll, pitch, yaw=true heading). Sets velocity vector to zero.
	Recommended for navigation-grade systems on a rotating base, 
	allowing vibrations with zero average acceleration,
	e.g. an airplane standing still on the ground with engine(s) running.

	description:
		algorithm may be found in a separate document [A. Kozlov]

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
		{imu: alignment} - imu initial alignment duration, sec
			type :   floating point
			range:   0+ to +inf
			default: 900
			example: {imu: alignment = 300}
*/
void pony_ins_alignment_rotating(void) {

	const char   
		t1_token[] = "alignment"; // alignment duration parameter name in configuration
	const double 
		t1_default = 900,         // default alignment duration
		v_std      = 1,           // velocity integral standard deviation
		S0         = 9e9,         // starting covariances
		q2_std     = 9;           // Earth rotation axis ort variance, rad^2/Hz
	const size_t 
		m      = 3;	              // approximation coefficient count

	static double 
		*L		= NULL, // pointer to attitude matrix in solution
		t1      = -1,   // alignment duration
		t_prev  = -1,   // previous time
		t0      = -1,   // alignment start time
		t_shift =  0,   // time since the alignment started
		slt     =  0,   //   sine of latitude
		clt     =  0,   // cosine of latitude
		fz0  [3],       // accelerometers output and their approximation in inertial frame
		vz0  [3],       // velocity integral
		fz0a0[3],       // approximation at t0
		fza  [3],       // approximation at current time in instrumental frame
		a    [3],       // Euler vector of rotation from inertial to instrumental reference frame
		C    [9],       // intermediate transition matrix
		Azz0 [9],       // transition matrix from inertial to instrumental reference frame
		x    [3][3],    // approximation coefficients
		 Sx  [3][6],    // upper-triangular part of Colesky factorization of approximation coefficients covariance
		 Kx  [3],       // approximation Kalman gain
		 hx  [3],       // approximation model matrix
		  y  [3],       // Earth rotation axis ort estimate
		 Sy  [6],       // upper-triangular part of Colesky factorization of Earth rotation axis ort covariance
		 hy  [3],       // Earth rotation axis ort model matrix
		 Ky  [3],       // Earth rotation axis ort Kalman gain
		 wf  [3],       // first column of attitude matrix
		fwf  [3];       // second column of attitude matrix
					    
	double 			    
		   dt,          // time increment
		   ut,          // Earth rotation angle
		  sut, cut,     // sine and cosine of Earth rotation angle
		  n,            // vector norm
		  q2;           // Earth rotation axis ort variance, rad^2
	char *cfg_ptr;      // pointer to a substring in configuration
	size_t 			    
		  i, j, k, k0;  // common indexing variables


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// reset time variables
		t_prev  = -1;
		t0      = -1;
		t_shift =  0;
		// parse alignment duration from configuration string
		cfg_ptr = pony_locate_token(t1_token, pony->imu->cfg, pony->imu->cfglength, '=');
		if (cfg_ptr != NULL)
			t1 = atof(cfg_ptr);		
		// if not found or invalid, set to default
		if (cfg_ptr == NULL || t1 <= 0)
			t1 = t1_default;
		// set matrix pointer to imu->sol
		L = pony->imu->sol.L;

	}

	else if (pony->mode < 0) {	// terminate
		// do nothing
	}

	else						// main cycle
	{
		// check if alignment duration exceeded 
		if (pony->imu->t > t1)
			return;
		// drop validity flags
		pony->imu->sol.L_valid		= 0;
		pony->imu->sol.q_valid		= 0;
		pony->imu->sol.rpy_valid	= 0;
		// check for crucial data present
		if (   !pony->imu->      w_valid     // check if angular rate measurements are available
			|| !pony->imu->      f_valid     // check if accelerometer measurements are available
			|| (   !pony->imu->sol.llh_valid // check if imu coordinates are available
			    && !pony->     sol.llh_valid // check if hybrid solution coordinates are available
			   )
			)
			return;
		// starting procedures
		if (t_prev < 0) {
			// time init
			t_prev = pony->imu->t;
			t0     = pony->imu->t;
			// identity attitude matrix
			for (i = 0; i < 9; i++)
				Azz0[i] = (i%4 == 0) ? 1 : 0;
			// component-wise init
			for (i = 0; i < 3; i++) {
				// velocity integral
				vz0[i] = 0;
				// starting estimates and covariances for approximation
				for (j = 0, k = 0; j < m; j++) {
					 x[i][j] = 0;
					Sx[i][k] = S0;
					k0 = k + m - j;
					for (k++; k < k0; k++) Sx[i][k] = 0;
				}
			}
			// starting estimates and covariances for Earth rotation axis ort
			for (j = 0, k = 0; j < 3; j++) {
				 y[j] = 0;
				Sy[k] = S0;
				k0 = k + 3 - j;
				for (k++; k < k0; k++) Sy[k] = 0;
			}
			// latitude
			if (pony->imu->sol.llh_valid) { // if imu coordinates are available
				slt    = sin(pony->imu->sol.llh[1]);
				clt    = cos(pony->imu->sol.llh[1]);
			}
			else if (pony->sol.llh_valid) { // if hybrid solution coordinates are available
				slt    = sin(pony->     sol.llh[1]);
				clt    = cos(pony->     sol.llh[1]);
			}
			return;
		}
		// time increments
		dt      = pony->imu->t - t_prev; 
		if (dt <= 0) return; // invalid increment, aborting
		t_prev  = pony->imu->t;
		t_shift = pony->imu->t - t0;
		q2 = q2_std/dt;
		// Earth rotation angle
		 ut = pony->imu_const.u*t_shift;
		sut = sin(ut);
		cut = cos(ut);
		// transforming into inertial frame fz0 = A^T*fz
		pony_linal_mmul1T(fz0, Azz0, pony->imu->f, 3, 3, 1);
		// approximation model coefficients
		hx[0] = (1 - cut)/(pony->imu_const.u*pony->imu_const.u);  
		hx[1] =   sut    / pony->imu_const.u;  
		hx[2] = t_shift;
		// approximation
		for (i = 0; i < 3; i++) {
			// velocity interal update
			vz0[i] += fz0[i]*dt;
			// approiximation coefficient estimates
			if (pony_linal_check_measurement_residual(x[i],Sx[i], vz0[i],hx,v_std, 3.0, m)) // check measurement residual within 3-sigma with current estimate
				pony_linal_kalman_update(x[i],Sx[i],Kx, vz0[i],hx,v_std, m);                // update estimates using v = h*x + dv, v_std = sqrt(E[dv^2])
			// approximation at t0 in inertial frame
			fz0a0[i] = x[i][1] + x[i][2];
			// approximation at t  in inertial frame
			fz0[i] = x[i][0]*sut/pony->imu_const.u + x[i][1]*cut + x[i][2];
		}
		// transition to instrumental frame (fza = Azz0*fz0) and normalization
		pony_linal_mmul(fza , Azz0, fz0 , 3, 3, 1);
		n = pony_linal_vnorm(fza,3);                    // n = |fza|
		if (n > 0) for (i = 0; i < 3; i++) fza[i] /= n; // normalization
		pony_linal_mmul(fz0, Azz0, fz0a0, 3, 3, 1);
		n = pony_linal_vnorm(fz0,3);                    // n = |fz0|
		if (n > 0) for (i = 0; i < 3; i++) fz0[i] /= n; // normalization
		// estimating Earth rotation axis ort
		C[0] =        - sut;
		C[1] = slt*(1 - cut);
		C[2] = slt*slt + clt*clt*cut;
		for (i = 0; i < 3; i++) {
			// H = (w x f)*C_13 + [f x (w x f)]*C_23
			hy[(i+0)%3] = (fza[(i+1)%3]*fza[(i+1)%3] + fza[(i+2)%3]*fza[(i+2)%3])*C[1];
			hy[(i+1)%3] =  fza[(i+2)%3]*C[0]         - fza[(i+0)%3]*fza[(i+1)%3] *C[1];
			hy[(i+2)%3] = -fza[(i+1)%3]*C[0]         - fza[(i+0)%3]*fza[(i+2)%3] *C[1];
			pony_linal_kalman_update(y,Sy,Ky, (fz0[i]-fza[i]*C[2])*sin(ut/2),hy,1, 3); // update estimate using z = [fza0 - fza*C_33], z = H*y + r, M[r^2] = 1
		}
		// Kalman prediction step, identity transition, diagonal system noise covariance
		pony_linal_kalman_predict_I_qI(Sy,q2,3); // P = S*S^T, P_ii = P_ii + q^2, S = chol(P)
		// renew attitude matrix: L = [ (w x f)/|w x f| , (f x (w x f))/(|f||w x f|) , f/|f| ]
		pony_linal_cross3x1( wf, y,fza); //  wf =      w x f
		n = pony_linal_vnorm(wf,3);      //   n =     |w x f|
		if (n > 0) {
			for (i = 0; i < 3; i++) L[i*3+0] =  wf[i]/n; // L1 =      (w x f) /    |w x f|
			pony_linal_cross3x1(fwf,fza,wf); // fwf = f x (w x f)
			for (i = 0; i < 3; i++) L[i*3+1] = fwf[i]/n; // L2 = (f x (w x f))/(|f||w x f|)
			for (i = 0; i < 3; i++) L[i*3+2] = fza[i];   // L3 =           f  / |f|
			pony->imu->sol.L_valid = 1;
			// renew quaternion
			pony_linal_mat2quat(pony->imu->sol.q ,L);
			pony->imu->sol.q_valid   = 1;
			// renew angles: roll, pitch and yaw=true heading
			pony_linal_mat2rpy(pony->imu->sol.rpy,L);
			pony->imu->sol.rpy_valid = 1;
		}
		// instrumental frame update: Azz0(t+dt) = (E + [w x]*sin(|w*dt|)/|w| + [w x]^2*(1-cos(|w*dt|)/|w*dt|^2)*Azz0(t) - Rodrigues' rotation formula
		for (i = 0; i < 3; i++)
			a[i] = pony->imu->w[i]*dt; // a = w*dt
		pony_linal_eul2mat(C,a);       // C = E + [a x]*sin(|a|)/|a| + [a x]^2*(1-cos(|a|)/|a|^2
		for (i = 0; i < 3; i++) {      // matrix multiplication overwriting Azz0
			for (j = 0; j < 3; j++)
				a[j] = Azz0[j*3+i];    // i-th column of Azz0 previous value
			for (j = 0; j < 3; j++)
				for (k = 0, Azz0[j*3+i] = 0; k < 3; k++)
					Azz0[j*3+i] += C[j*3+k]*a[k]; // replace column with matrix product: Azz0(t+dt) = C*Azz0(t)
		}
		// set velocity equal to zero, with no better information at initial alignment phase
		for (i = 0; i < 3; i++)
			pony->imu->sol.v[i] = 0;
		pony->imu->sol.v_valid = 1;
	}

}

/* pony_ins_alignment_rotating_rpy - pony plugin
	
	The same as pony_ins_alignment_rotating, but attitude matrix is calculated using attitude angles
	(roll, pitch and yaw=true heading). Does not allow the first instrumental axis to point upwards.
	Sets velocity vector to zero.

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
		{imu: alignment} - imu initial alignment duration, sec
			type :   floating point
			range:   0+ to +inf
			default: 900
			example: {imu: alignment = 300}
*/
void pony_ins_alignment_rotating_rpy(void) {

	const char   
		t1_token[] = "alignment"; // alignment duration parameter name in configuration
	const double 
		t1_default = 900,         // default alignment duration
		v_std      = 1,           // velocity integral standard deviation
		S0         = 9e9;         // starting covariances
	const size_t 
		m      = 3;	              // approximation coefficient count

	static double 
		t1      = -1,  // alignment duration
		t_prev  = -1,  // previous time
		t0      = -1,  // alignment start time
		t_shift =  0,  // time since the alignment started
		slt     =  0,  //   sine of latitude
		clt     =  0,  // cosine of latitude
		fz0  [3],      // accelerometers output and their approximation in inertial frame
		vz0  [3],      // velocity integral
		rpy0 [3],      // initial roll, pitch and yaw=true heading
		fz0a0[3],      // approximation at t0
		fza  [3],      // approximation at current time in instrumental frame
		a    [3],      // Euler vector of rotation from inertial to instrumental reference frame
		C    [9],      // intermediate transition matrix
		D    [9],      // intermediate transition matrix
		L0   [9],      // initial attitude matrix
		Azz0 [9],      // transition matrix from inertial to instrumental reference frame
		x    [3][3],   // approximation coefficients
		 Sx  [3][6],   // upper-triangular part of Colesky factorization of approximation coefficients covariance
		 Kx  [3],      // approximation Kalman gain
		 hx  [3];      // approximation model matrix

	double 
		   dt,         // time increment
		   ut,         // Earth rotation angle
		  sut, cut;    // sine and cosine of Earth rotation angle
	char *cfg_ptr;     // pointer to a substring in configuration
	size_t 
		  i, j, k, k0; // common indexing variables


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// reset time variables
		t_prev  = -1;
		t0      = -1;
		t_shift =  0; 
		// parse alignment duration from configuration string
		cfg_ptr = pony_locate_token(t1_token, pony->imu->cfg, pony->imu->cfglength, '=');
		if (cfg_ptr != NULL)
			t1 = atof(cfg_ptr);		
		// if not found or invalid, set to default
		if (cfg_ptr == NULL || t1 <= 0)
			t1 = t1_default;

	}

	else if (pony->mode < 0) {	// terminate
		// do nothing
	}

	else						// main cycle
	{
		// check if alignment duration exceeded 
		if (pony->imu->t > t1)
			return;
		// drop validity flags
		pony->imu->sol.L_valid		= 0;
		pony->imu->sol.q_valid		= 0;
		pony->imu->sol.rpy_valid	= 0;
		// check for crucial data present
		if (   !pony->imu->      w_valid     // check if angular rate measurements are available
			|| !pony->imu->      f_valid     // check if accelerometer measurements are available
			|| (   !pony->imu->sol.llh_valid // check if imu coordinates are available
			    && !pony->     sol.llh_valid // check if hybrid solution coordinates are available
			   )
			)
			return;
		// starting procedures
		if (t_prev < 0) {
			// time init
			t_prev = pony->imu->t;
			t0     = pony->imu->t;
			// identity attitude matrix
			for (i = 0; i < 9; i++)
				Azz0[i] = (i%4 == 0) ? 1 : 0;
			// component-wise init
			for (i = 0; i < 3; i++) {
				// velocity integral
				vz0[i] = 0;
				// starting estimates and covariances for approximation
				for (j = 0, k = 0; j < m; j++) {
					 x[i][j] = 0;
					Sx[i][k] = S0;
					k0 = k + m - j;
					for (k++; k < k0; k++) Sx[i][k] = 0;
				}
			}
			// latitude
			if (pony->imu->sol.llh_valid) { // if imu coordinates are available
				slt    = sin(pony->imu->sol.llh[1]);
				clt    = cos(pony->imu->sol.llh[1]);
			}
			else if (pony->sol.llh_valid) { // if hybrid solution coordinates are available
				slt    = sin(pony->     sol.llh[1]);
				clt    = cos(pony->     sol.llh[1]);
			}
			return;
		}
		// time increments
		dt      = pony->imu->t - t_prev; 
		if (dt <= 0) return; // invalid increment, aborting
		t_prev  = pony->imu->t;
		t_shift = pony->imu->t - t0;
		// Earth rotation angle
		 ut = pony->imu_const.u*t_shift;
		sut = sin(ut);
		cut = cos(ut);
		// transforming into inertial frame fz0 = A^T*fz
		pony_linal_mmul1T(fz0, Azz0, pony->imu->f, 3, 3, 1);
		// approximation model coefficients
		hx[0] = (1 - cut)/(pony->imu_const.u*pony->imu_const.u);  
		hx[1] =   sut    / pony->imu_const.u;  
		hx[2] = t_shift;
		// approximation
		for (i = 0; i < 3; i++) {
			// velocity interal update
			vz0[i] += fz0[i]*dt;
			// approiximation coefficient estimates
			if (pony_linal_check_measurement_residual(x[i],Sx[i], vz0[i],hx,v_std, 3.0, m)) // check measurement residual within 3-sigma with current estimate
				pony_linal_kalman_update(x[i],Sx[i],Kx, vz0[i],hx,v_std, m);                // update estimates using v = h*x + dv, v_std = sqrt(E[dv^2])
			// approximation at t0 in inertial frame
			fz0a0[i] = x[i][1] + x[i][2];
			// approximation at t  in inertial frame
			fz0[i] = x[i][0]*sut/pony->imu_const.u + x[i][1]*cut + x[i][2];
		}
		// transition to instrumental frame (fza = Azz0*fz0) and normalization
		pony_linal_mmul(fza , Azz0, fz0 , 3, 3, 1);
		// roll angle via fza components
		pony->imu->sol.rpy[0] = -atan2(fza[2], fza[1]);
		// pitch angle via fza components
		pony->imu->sol.rpy[1] =  atan2(fza[0], sqrt(fza[1]*fza[1] + fza[2]*fza[2]));
		// heading, main branch
		// initial roll and pitch, zero heading
		rpy0[0] = -atan2(fz0a0[2], fz0a0[1]);
		rpy0[1] =  atan2(fz0a0[0], sqrt(fz0a0[1]*fz0a0[1] + fz0a0[2]*fz0a0[2]));
		rpy0[2] =  0;
		// "matrix C" `\_("o)_/`
		pony->imu->sol.rpy[2] = 0;
		pony_linal_rpy2mat(pony->imu->sol.L, pony->imu->sol.rpy); // Axz^T(t)
			// how to produce matrix A_z_x_t0: via angles at either t0, or t `\_("o)_/`
		pony_linal_rpy2mat(L0, rpy0); // Azx(t0)
		pony_linal_mmul1T(D, pony->imu->sol.L, Azz0, 3,3,3);
		pony_linal_mmul  (C, D, L0, 3,3,3);
		// "matrix D" `\_("o)_/`	 // only the elements being used	
		D[2] = -sut*clt;
		D[5] = (1 - cut)*slt*clt;
		// heading(t) `\_("o)_/`
		pony->imu->sol.rpy[2] = atan2(-C[2]*D[5] + C[5]*D[2], C[2]*D[2] + C[5]*D[5]);
		pony->imu->sol.rpy_valid = 1;
		// attitude matrix
		pony_linal_rpy2mat(pony->imu->sol.L, pony->imu->sol.rpy);
		pony->imu->sol.L_valid = 1;
		// renew quaternion
		pony_linal_mat2quat(pony->imu->sol.q  , pony->imu->sol.L);
		pony->imu->sol.q_valid   = 1;		
		// instrumental frame update: Azz0(t+dt) = (E + [w x]*sin(|w*dt|)/|w| + [w x]^2*(1-cos(|w*dt|)/|?|^2)*Azz0(t) - Rodrigues' rotation formula
		for (i = 0; i < 3; i++)
			a[i] = pony->imu->w[i]*dt; // a = w*dt
		pony_linal_eul2mat(C,a);       // � = E + [a x]*sin(|a|)/|a| + [a x]^2*(1-cos(|a|)/|a|^2
		for (i = 0; i < 3; i++) {      // matrix multiplication overwriting Azz0
			for (j = 0; j < 3; j++)
				a[j] = Azz0[j*3+i];    // i-th column of Azz0 previous value
			for (j = 0; j < 3; j++)
				for (k = 0, Azz0[j*3+i] = 0; k < 3; k++)
					Azz0[j*3+i] += C[j*3+k]*a[k]; // replace column with matrix product: Azz0(t+dt) = C*Azz0(t)
		}
		// set velocity equal to zero, with no better information at initial alignment phase
		for (i = 0; i < 3; i++)
			pony->imu->sol.v[i] = 0;
		pony->imu->sol.v_valid = 1;
	}

}
