// Nov-2020
// Oct-2020
/*	pony_ins_attitude
	
	pony plugins for ins angular rate integration:
	
	- pony_ins_attitude_rodrigues
		Combines angular rate components or their integrals into Euler rotation vector 
		for both instrumental frame and navigation frame. Then applies Rodrigues' rotation formula 
		to each frame. Then derives the transition matrix between them. Quaternion and angles are also updated.
		Recommended for navigation/tactical grade systems.

	- (planned) pony_ins_attitude_madgwick
		Planned for future development
*/

#include <math.h>

#include "../pony.h"

// pony bus version check
#define PONY_INS_ATTITUDE_BUS_VERSION_REQUIRED 8
#if PONY_BUS_VERSION < PONY_INS_ATTITUDE_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif

/* pony_ins_attitude_rodrigues - pony plugin
	
	Combines angular rate components or their integrals into Euler rotation vector 
	for both instrumental frame and navigation frame. Then applies Rodrigues' rotation formula 
	to each frame. Then derives the transition matrix between them. Quaternion and angles are also updated.
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
		pony->imu->t
		pony->imu->sol.L
		pony->imu->sol.L_valid
		pony->imu->w
		pony->imu->w_valid
		pony->imu->W
		pony->imu->W_valid
		pony->imu->sol.llh
		pony->imu->sol.llh_valid

	changes:
		pony->imu->sol.L
		pony->imu->sol.L_valid
		pony->imu->sol.q
		pony->imu->sol.q_valid
		pony->imu->sol.rpy
		pony->imu->sol.rpy_valid

	cfg parameters:
		none
*/
void pony_ins_attitude_rodrigues(void) {

	static double t0 = -1; // previous time
	static double 		   
		 C[9],             // intermediate matrix
		*L;                // pointer to attitude matrix in solution

	double 
		   dt,	           // time step
		   a[3];           // Euler rotation vector
	size_t i;              // common index variable


	// check if imu data has been initialized
	if (pony->imu == NULL)
		return;

	if (pony->mode == 0) {		// init

		// drop validity flags
		pony->imu->sol.  q_valid = 0;
		pony->imu->sol.  L_valid = 0;
		pony->imu->sol.rpy_valid = 0;
		// set matrix pointer to imu->sol
		L = pony->imu->sol.L;	
		// identity quaternion
		for (i = 1, pony->imu->sol.q[0] = 1; i < 4; i++)
			pony->imu->sol.q[i] = 0;
		pony->imu->sol.  q_valid = 1;
		// identity attitude matrix
		for (i = 0; i < 9; i++)
			L[i] = ((i%4) == 0) ? 1 : 0; // for 3x3 matrix, each 4-th element is diagonal
		pony->imu->sol.  L_valid = 1;
		// attitude angles for identity matrix
		pony->imu->sol.rpy[0] = -pony->imu_const.pi/2;	// roll             -90 deg
		pony->imu->sol.rpy[1] =  0;						// pitch              0 deg
		pony->imu->sol.rpy[2] = +pony->imu_const.pi/2;	// yaw=true heading +90 deg
		pony->imu->sol.rpy_valid = 1;
		// reset previous time
		t0 = -1;

	}

	else if (pony->mode < 0) {	// termination
		// do nothing
	}
	else						// main cycle
	{
		// check for crucial data initialized
		if (!pony->imu->sol.L_valid || !pony->imu->w_valid)
			return;
		// time variables
		if (t0 < 0) { // first touch
			t0 = pony->imu->t;
			return;
		}
		dt = pony->imu->t - t0;
		t0 = pony->imu->t;
		// a = w*dt
		for (i = 0; i < 3; i++)
			a[i] = pony->imu->w[i]*dt;
		// L = (E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2)*L
		pony_linal_eul2mat(C,a);  // C <- A = E + [a x]*sin(a)/|a| + [a x]^2*(1-cos(a))/|a|^2
		for (i = 0; i < 3; i++) { // L = A*L
			a[0] = L[0+i], a[1] = L[3+i], a[2] = L[6+i];
			L[0+i] = C[0]*a[0] + C[1]*a[1] + C[2]*a[2];
			L[3+i] = C[3]*a[0] + C[4]*a[1] + C[5]*a[2];
			L[6+i] = C[6]*a[0] + C[7]*a[1] + C[8]*a[2];
		}
		// a <- c = (W + u)*dt
		for (i = 0; i < 3; i++)
			a[i] = pony->imu->W_valid ? pony->imu->W[i] : 0;
		if (pony->imu->sol.llh_valid) {
			a[1] += pony->imu_const.u*cos(pony->imu->sol.llh[1]);
			a[2] += pony->imu_const.u*sin(pony->imu->sol.llh[1]);
		}
		for (i = 0; i < 3; i++)
			a[i] *= dt;
		// L = L*(E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2)^T
		pony_linal_eul2mat(C,a);     // C = E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2
		for (i = 0; i < 9; i += 3) { // L = L*C^T
			a[0] = L[i+0], a[1] = L[i+1], a[2] = L[i+2];
			L[i+0] = a[0]*C[0] + a[1]*C[1] + a[2]*C[2];
			L[i+1] = a[0]*C[3] + a[1]*C[4] + a[2]*C[5];
			L[i+2] = a[0]*C[6] + a[1]*C[7] + a[2]*C[8];
		}
		// renew quaternion
		pony_linal_mat2quat(pony->imu->sol.q ,L);
		pony->imu->sol.q_valid   = 1;
		// renew angles
		pony_linal_mat2rpy(pony->imu->sol.rpy,L);
		pony->imu->sol.rpy_valid = 1;

	}

}