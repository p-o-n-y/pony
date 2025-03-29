// Feb-2025
/*	pony_ins_attitude

	pony plugins for ins angular rate integration:

	- pony_ins_attitude_rodrigues
		Updates attitude matrix using Euler rotation vectors and Rodrigues rotation formula.
		Applies approximated Runge-Kutta 4-th order integration to Bortz's kinematic equation
		for proper rotation, and single-step Euler integration for navigation (base) ENU frame.
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
void   pony_ins_attitude_integration_rk4(double *L, double *w0, double *w1, double dt);
void   pony_ins_attitude_proper_rotation(double* L, const double a[3]);
void   pony_ins_attitude_base_rotation  (double* L, const double c[3]);
void   pony_ins_attitude_odefun(double *F, double *w, double *p);
	// initialization
void   pony_ins_attitude_init(pony_imu* imu);
	// math
double pony_ins_attitude_round(double x) { return floor(x + 0.5); }
	// memory handling
void   pony_ins_attitude_free_null(void** ptr);


/* pony_ins_attitude_rodrigues - pony plugin

	Updates attitude matrix using Euler rotation vectors and Rodrigues rotation formula.
	Applies approximated Runge-Kutta 4-th order integration to Bortz's kinematic equation
	for proper rotation, and single-step Euler integration for navigation (base) ENU frame.
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
		 Wu_valid = 0x4, // for upward component
		rpy_valid = 0x7; // roll, pitch, true heading

	static size_t ndev;             // number of IMU devices
	static double
		(*w0)[3] = NULL,            // previous angular rate measurements
		 *t0     = NULL,            // previous times, for each IMU device
		 *lon0   = NULL;            // previous longitudes to keep navigation frame aligned to East and North directions, for each IMU device
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
		w0         = (double(*)[3])calloc(ndev, sizeof(double[3]));
		t0         = (double *    )calloc(ndev, sizeof(double   ));
		lon0       = (double *    )calloc(ndev, sizeof(double   ));
		lon0_valid = (char   *    )calloc(ndev, sizeof(char     ));
		if (w0 == NULL || t0 == NULL || lon0 == NULL || lon0_valid == NULL) {
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
			for (i = 0; i < 3; i++)
				w0[dev][i] = 0;
			t0        [dev] = -1;
			lon0      [dev] =  0;
			lon0_valid[dev] =  0;
		}

	}

	else if (pony->mode < 0) {	// termination

		// free allocated memory
		pony_ins_attitude_free_null((void**)(&t0        ));
		pony_ins_attitude_free_null((void**)(&w0        ));
		pony_ins_attitude_free_null((void**)(&lon0      ));
		pony_ins_attitude_free_null((void**)(&lon0_valid));

	}

	else						// regular processing
	{
		// go through all IMU devices
		for (dev = 0; dev < ndev; dev++) {
			// skip uninitialized subsystems
			if (pony->imu[dev].cfg == NULL || !pony->imu[dev].sol.L_valid || !pony->imu[dev].w_valid)
				continue;
			// time variables
			if (t0[dev] < 0) { // first touch
				t0[dev] = pony->imu[dev].t;
				for (i = 0; i < 3; i++)
					w0[dev][i] = pony->imu[dev].w[i];
				if (pony->imu[dev].sol.llh_valid) {
					lon0[dev]       = pony->imu[dev].sol.llh[E]; // track longitude to keep navigation frame aligned to East and North
					lon0_valid[dev] = 1;
				}
				continue;
			}
			dt      = pony->imu[dev].t - t0[dev];
			t0[dev] = pony->imu[dev].t;
			// proper rotation
			pony_ins_attitude_integration_rk4(pony->imu[dev].sol.L, w0[dev], pony->imu[dev].w, dt);
			for (i = 0; i < 3; i++) // store previous angular rate measurements
				w0[dev][i] = pony->imu[dev].w[i];
			// navigation (base) frame rotation
			for (i = 0; i < 3; i++)
				a[i] = ((pony->imu[dev].W_valid & (0x1<<i)) ? pony->imu[dev].W[i] : 0)*dt; // check each validity flag separately, when W_valid is a bitfield
			if (pony->imu[dev].sol.llh_valid) {
				// ENU navigation frame
				if (lon0_valid[dev]) {
					dlon      = pony->imu[dev].sol.llh[E] - lon0[dev];
					lon0[dev] = pony->imu[dev].sol.llh[E];
					dlon -= pony_ins_attitude_round(dlon/pony->imu_const.pi2)*pony->imu_const.pi2;
				}
				else
					dlon = 0;
				dlon *= sin(pony->imu[dev].sol.llh[N]);
				a[U] = dlon;
				// Earth rotation
				a[N] += pony->imu_const.u*cos(pony->imu[dev].sol.llh[N])*dt;
				a[U] += pony->imu_const.u*sin(pony->imu[dev].sol.llh[N])*dt;
				// ENU frame azimuth rate
				if (dt != 0) {
					pony->imu[dev].W[U] = dlon/dt;       // rotate navigation frame in azimuth to keep it aligned to East and North
					pony->imu[dev].W_valid |=  Wu_valid; // set   component validity
				}
				else
					pony->imu[dev].W_valid &= ~Wu_valid; // clear component validity
			}
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
void pony_ins_attitude_integration_rk4(double *L, double *w0, double *w1, double dt)
{
	static double
		a[3], w2[3],
		k1[3], k2[3], k3[3], k4[3];

	size_t i;


	// a is the Euler rotation vector as the solution to the Bortz's kinematic equation at [0,dt]:
	//   da/dt = w + 1/2[a x w] + [1/|a|^2]*[1 - |a|/2*sin|a|/(1-cos|a|)]*[a x [a x w]], a(0) = 0
	// using Runge-Kutta 4-th order method with the following approximations for an integrating angular rate sensor:
	//   w(0)    ~ 1/2[  w1' + w0'], w0' = w'(t_n-1), w1' = w'(t_n), for the instantaneous angular rate at left node
	//   w(dt/2) ~ w1', for the instantaneous angular rate at midpoint as a measured average value
	//   w(dt)   ~ 1/2[3*w1' - w0'], for the instantaneous angular rate at right node as an extrapolation of average values
	// [1/|a|^2]*[1 - |a|/2*sin|a|/(1-cos|a|)] ~ 1/12 + |a|^2/720, given |a| << 1
	// see A.Kozlov, F.Kapralov - Angular Misalignment Calibration for Dual-Antenna GNSS/IMU Navigation Sensor
	// in MDPI Sensors 2023, 23, 77. DOI: 10.3390/s23010077
		// RK4 steps #1 & #2
	for (i = 0; i < 3; i++) {
		k1[i] = (w0[i] + w1[i])/2;   // w(0) = pony_ins_attitude_odefun(k1, w(t_n-1), 0)
		a [i] = dt/2*k1[i];
	}
	pony_ins_attitude_odefun(k2, w1, a);
		// RK4 step #3
	for (i = 0; i < 3; i++)
		a [i] = dt/2*k2[i];
	pony_ins_attitude_odefun(k3, w1, a);
		// RK4 step #4
	for (i = 0; i < 3; i++) {
		w2[i] = (3*w1[i] - w0[i])/2;
		a [i] = dt*k3[i];
	}
	pony_ins_attitude_odefun(k4, w2, a);
		// RK4 steps combination
	for (i = 0; i < 3; i++)
		a[i] = dt/6*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
	// attitude transformation
	pony_ins_attitude_proper_rotation(L, a);
}

void pony_ins_attitude_proper_rotation(double* L, const double a[3])
{
	double
		A [9], // intermediate matrix
		Li[3]; // matrix column
	size_t  i;

	// attitude matrix transformation using Rodrigues rotation formula
	// L = (E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2)*L
	pony_linal_eul2mat(A,a);  // A = E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2
	for (i = 0; i < 3; i++) { // L = A*L
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

void pony_ins_attitude_odefun(double *F, double *w, double *p)
{
	static double pxw[3], pxpxw[3];
	double k;
	size_t i;

	// below is an approximation of the Bortz's equation right-hand side:
	// see Bortz,J.E. - A New Mathematical Formulation for Strapdown Inertial Navigation
	// in IEEE Transactions on Aerospace and Electronic Systems, vol.AES-7, No.1, 1971, pp. 61-66
	// DOI: 10.1109/TAES.1971.310252
	// with w for omega, and p for phi
	k = (60 + pony_linal_dot(p, p, 3))/720;  //     k = 1/12 + |p|^2/720
	pony_linal_cross3x1(pxw, p, w);          //   pxw = p x w
	pony_linal_cross3x1(pxpxw, pxw, w);      // pxpxw = p x [p x w]

	for (i = 0; i < 3; i++)
		F[i] = w[i] + pxw[i]/2 + k*pxpxw[i]; //     F = w + 1/2 [p x w] + (1/12 + |p|^2/720)(p x [p x w])
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
