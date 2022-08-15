// Aug-2022
/*	pony_ins_attitude
	
	pony plugins for ins angular rate integration:
	
	- pony_ins_attitude_rodrigues
		Combines angular rate components or their integrals into Euler rotation vector 
		for both instrumental frame and navigation frame (according to rate validity). 
		Then applies Rodrigues' rotation formula to each frame. Then derives the transition matrix between them. 
		Quaternion and angles are also updated.
		Performs attitude integration step for each initialized IMU device with valid current attitude matrix.
		Recommended for navigation/tactical grade systems.

	- (planned) pony_ins_attitude_madgwick
		Planned for future development
*/


#include <stdlib.h>
#include <math.h>

#include "../pony.h"


// pony bus version check
#define PONY_INS_ATTITUDE_BUS_VERSION_REQUIRED 18
#if PONY_BUS_VERSION < PONY_INS_ATTITUDE_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif


// internal routines
	// attitude algorithms
void pony_ins_attitude_proper_rotation(double* L, const double a[3]);
void pony_ins_attitude_base_rotation  (double* L, const double c[3]);
	// initialization
void pony_ins_attitude_init(pony_imu* imu);
	// memory handling
void pony_ins_attitude_free_null(void** ptr);


/* pony_ins_attitude_rodrigues - pony plugin
	
	Combines angular rate components or their integrals into Euler rotation vector 
	for both instrumental frame and navigation frame (according to rate validity). 
	Then applies Rodrigues' rotation formula to each frame. Then derives the transition matrix between them. 
	Quaternion and angles are also updated.
	Performs attitude integration step for each initialized IMU device with valid current attitude matrix.
	Recommended for navigation/tactical grade systems.

	description:
		    
		L(t+dt) = A L(t) C^T,
		    
		where
		A = E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2
		C = E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2,
		a = w*dt, c = (W + u)*dt
		        [  0   v3 -v2 ]
		[v x] = [ -v3  0   v1 ] for v = a, v = c
	            [  v2 -v1  0  ]
				
		pony core function pony_linal_eul2mat safely uses Taylor expansions for |a|,|c| < 2^-8
		
	uses:
		pony->imu[].cfg
		pony->imu[].t
		pony->imu[].sol.L
		pony->imu[].sol.L_valid
		pony->imu[].w
		pony->imu[].w_valid
		pony->imu[].W
		pony->imu[].W_valid
		pony->imu[].sol.llh
		pony->imu[].sol.llh_valid

	changes:
		pony->imu[].sol.L
		pony->imu[].sol.L_valid
		pony->imu[].sol.q
		pony->imu[].sol.q_valid
		pony->imu[].sol.rpy
		pony->imu[].sol.rpy_valid

	cfg parameters:
		none
*/
void pony_ins_attitude_rodrigues(void) 
{
	enum {E, N, U};

	const char 
		  W_valid = 0x4,
		rpy_valid = 0x7; // roll, pitch, true heading

	static size_t ndev;             // number of IMU devices
	static double 
		*t0   = NULL,               // previous times, for each IMU device
		*lon0 = NULL;               // previous longitudes to keep navigation frame aligned to East and North directions, for each IMU device
	static char *lon0_valid = NULL; // previous longitude validity flags, for each IMU device

	double 
		   dt,	           // time step
		   dlon,           // longitude increment since previous step
		   a[3];           // Euler rotation vector
	size_t i, dev;         // index variables


	// validate IMU subsystem
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// allocate memory
		ndev = pony->imu_count;
		t0         = (double*)calloc(ndev, sizeof(double));
		lon0       = (double*)calloc(ndev, sizeof(double));
		lon0_valid = (char  *)calloc(ndev, sizeof(char  ));
		if (t0 == NULL || lon0 == NULL || lon0_valid == NULL) {
			pony->mode = -1;
			return;
		}
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL)
				continue;
			// init attitude solution
			pony_ins_attitude_init(&(pony->imu[dev]));
			// reset previous time and longitude
			t0        [dev] = -1;
			lon0      [dev] =  0;
			lon0_valid[dev] =  0;
		}

	}

	else if (pony->mode < 0) {	// termination

		// free allocated memory
		pony_ins_attitude_free_null((void**)(&t0        ));
		pony_ins_attitude_free_null((void**)(&lon0      ));
		pony_ins_attitude_free_null((void**)(&lon0_valid));

	}

	else						// regular processing
	{
		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL || !pony->imu[dev].sol.L_valid)
				continue;
			// time variables
			if (t0[dev] < 0) { // first touch
				t0[dev] = pony->imu[dev].t;
				if (pony->imu[dev].sol.llh_valid) {
					lon0[dev]       = pony->imu[dev].sol.llh[E]; // track longitude to keep navigation frame aligned to East and North
					lon0_valid[dev] = 1;
				}
				continue;
			}
			dt      = pony->imu[dev].t - t0[dev];
			t0[dev] = pony->imu[dev].t;
			// proper rotation
			if (pony->imu[dev].w_valid) {
				for (i = 0; i < 3; i++)
					a[i] = pony->imu[dev].w[i]*dt;
				pony_ins_attitude_proper_rotation(pony->imu[dev].sol.L, a);
			}
			// navigation (base) frame rotation
			for (i = 0; i < 3; i++)
				a[i] = (pony->imu[dev].W_valid & (0x1<<i)) ? pony->imu[dev].W[i] : 0; // check each validity flag separately, when W_valid is a bitfield
			if (pony->imu[dev].sol.llh_valid) {
				// ENU navigation frame
				if (lon0_valid) {
					dlon      = pony->imu[dev].sol.llh[E] - lon0[dev];
					lon0[dev] = pony->imu[dev].sol.llh[E];
					while (dlon > +pony->imu_const.pi) dlon -= pony->imu_const.pi2;
					while (dlon < -pony->imu_const.pi) dlon += pony->imu_const.pi2;
				}
				else
					dlon = 0;
				dlon *= sin(pony->imu[dev].sol.llh[N]);
				pony->imu[dev].W[U] = dlon/dt;          // rotate navigation frame in azimuth to keep it aligned to East and North
				pony->imu[dev].W_valid |= W_valid;      // add up component validity
				a[U] = pony->imu[dev].W[U];
				// Earth rotation
				a[N] += pony->imu_const.u*cos(pony->imu[dev].sol.llh[N]);
				a[U] += pony->imu_const.u*sin(pony->imu[dev].sol.llh[N]);
			}
			for (i = 0; i < 3; i++)
				a[i] *= dt;
			pony_ins_attitude_base_rotation(pony->imu[dev].sol.L, a);
			// renew quaternion
			pony_linal_mat2quat(pony->imu[dev].sol.q , pony->imu[dev].sol.L);
			pony->imu[dev].sol.q_valid   = 1;
			// renew angles
			pony_linal_mat2rpy(pony->imu[dev].sol.rpy, pony->imu[dev].sol.L);
			pony->imu[dev].sol.rpy_valid = rpy_valid;
		}

	}

}


// internal routines
	// attitude algorithms
void pony_ins_attitude_proper_rotation(double* L, const double a[3])
{
	double 
		A [9], // intermediate matrix
		Li[3]; // matrix column
	size_t  i;

	// L = (E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2)*L
	pony_linal_eul2mat(A,a);  // A = E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2
	for (i = 0; i < 3; i++) { //      L = A*L
		Li[0] = L[0+i], Li[1] = L[3+i], Li[2] = L[6+i];
		L[0+i] = A[0]*Li[0] + A[1]*Li[1] + A[2]*Li[2];
		L[3+i] = A[3]*Li[0] + A[4]*Li[1] + A[5]*Li[2];
		L[6+i] = A[6]*Li[0] + A[7]*Li[1] + A[8]*Li[2];
	}
}

void pony_ins_attitude_base_rotation(double* L, const double c[3])
{
	double 
		C [9], // intermediate matrix
		Li[3]; // matrix row
	size_t  i;

	// L = L*(E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2)^T
	pony_linal_eul2mat(C,c);     // C = E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2
	for (i = 0; i < 9; i += 3) { // L = L*C^T
		Li[0] = L[i+0], Li[1] = L[i+1], Li[2] = L[i+2];
		L[i+0] = Li[0]*C[0] + Li[1]*C[1] + Li[2]*C[2];
		L[i+1] = Li[0]*C[3] + Li[1]*C[4] + Li[2]*C[5];
		L[i+2] = Li[0]*C[6] + Li[1]*C[7] + Li[2]*C[8];
	}
}

	// initialization
void pony_ins_attitude_init(pony_imu* imu)
{
	size_t i;

	// identity quaternion
	for (i = 1, imu->sol.q[0] = 1; i < 4; i++)
		imu->sol.q[i] = 0;
	imu->sol.  q_valid = 0;
	// identity attitude matrix
	for (i = 0; i < 9; i++)
		imu->sol.L[i] = ((i%4) == 0) ? 1 : 0; // for 3x3 matrix, each 4-th element is diagonal
	imu->sol.  L_valid = 0;
	// attitude angles for identity matrix
	imu->sol.rpy[0] = -pony->imu_const.pi_2;	// roll             -90 deg
	imu->sol.rpy[1] =  0;						// pitch              0 deg
	imu->sol.rpy[2] = +pony->imu_const.pi_2;	// yaw=true heading +90 deg
	imu->sol.rpy_valid = 0;
}

	// memory handling
void pony_ins_attitude_free_null(void** ptr)
{
	if (ptr == NULL || *ptr == NULL)
		return;

	free(*ptr);
	*ptr = NULL;
}
