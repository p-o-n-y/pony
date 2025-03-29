// Jan-2025
/*	pony_gnss_sat

	pony plugins for GNSS satellite-related calculations:

	- pony_gnss_sat_pos_vel_clock_gps
		Calculates position, velocity and clock correction for all GPS             satellites with valid ephemeris.
	- pony_gnss_sat_pos_vel_clock_glo
		Calculates position, velocity and clock correction for all GLONASS         satellites with valid ephemeris.
	- pony_gnss_sat_pos_vel_clock_gal
		Calculates position, velocity and clock correction for all Galileo         satellites with valid ephemeris.
	- pony_gnss_sat_pos_vel_clock_bds
		Calculates position, velocity and clock correction for all BeiDou MEO/IGSO satellites with valid ephemeris.
*/

#include <stdlib.h>
#include <math.h>

#include "../pony.h"

// pony bus version check
#define PONY_GNSS_SAT_BUS_VERSION_REQUIRED 7
#if PONY_BUS_VERSION < PONY_GNSS_SAT_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif

// internal routines
	// single-receiver satellite calculations for GPS
void   pony_gnss_sat_pos_vel_clock_gps_single_receiver(pony_gnss *gnss, double *allIODE, double *allA, double *alln, double *allE, double **allaf);
	// single-receiver satellite calculations for GLONASS
void   pony_gnss_sat_pos_vel_clock_glo_single_receiver(pony_gnss *gnss,  double *t, double **x, double **v, double **a, int *tb,  double *rk4_y, double **rk4_k);
void   pony_gnss_sat_glo_motion_rk4_step(double *x, double *v, double *a,  double dt,  double *rk4_y, double **rk4_k);
void   pony_gnss_sat_glo_motion_ode_fun(double *dy,  double *y, double *a);
	// single-receiver satellite calculations for Galileo
void   pony_gnss_sat_pos_vel_clock_gal_single_receiver(pony_gnss *gnss, double *allIODnav, double *allA, double *alln, double *allE, double **allaf);
	// single-receiver satellite calculations for BeiDou
void   pony_gnss_sat_pos_vel_clock_bds_single_receiver(pony_gnss *gnss, double *allSOW, double *allA, double *alln, double *allE, double **alla);
	// service routines
char   pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gps  (double  ***ptr);                             // allocate memory for [gnss_count x sat_count                ] doubles
char   pony_gnss_sat_alloc_pppointer_gnss_count_sat_count_gps (double ****ptr, const size_t internal_size); // allocate memory for [gnss_count x sat_count x internal_size] doubles
char   pony_gnss_sat_alloc_dppointer_gnss_count_sat_count_glo (double  ***ptr);                             // allocate memory for [gnss_count x sat_count                ] doubles
char   pony_gnss_sat_alloc_ippointer_gnss_count_sat_count_glo (int     ***ptr);                             // allocate memory for [gnss_count x sat_count                ] integers
char   pony_gnss_sat_alloc_dpppointer_gnss_count_sat_count_glo(double ****ptr, const size_t internal_size); // allocate memory for [gnss_count x sat_count x internal_size] doubles
char   pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gal  (double  ***ptr);                             // allocate memory for [gnss_count x sat_count                ] doubles
char   pony_gnss_sat_alloc_pppointer_gnss_count_sat_count_gal (double ****ptr, const size_t internal_size); // allocate memory for [gnss_count x sat_count x internal_size] doubles
char   pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_bds  (double  ***ptr);                             // allocate memory for [gnss_count x sat_count                ] doubles
char   pony_gnss_sat_alloc_pppointer_gnss_count_sat_count_bds (double ****ptr, const size_t internal_size); // allocate memory for [gnss_count x sat_count x internal_size] doubles
void   pony_gnss_sat_free_pppointer_gnss_count_sat_count_gps  (void   ****ptr);                             // free memory and assign NULL to pointer to [gnss_count x sat_count] double arrays
void   pony_gnss_sat_free_pppointer_gnss_count_sat_count_glo  (void   ****ptr);                             // free memory and assign NULL to pointer to [gnss_count x sat_count] double arrays
void   pony_gnss_sat_free_pppointer_gnss_count_sat_count_gal  (void   ****ptr);                             // free memory and assign NULL to pointer to [gnss_count x sat_count] double arrays
void   pony_gnss_sat_free_pppointer_gnss_count_sat_count_bds  (void   ****ptr);                             // free memory and assign NULL to pointer to [gnss_count x sat_count] double arrays
void   pony_gnss_sat_free_ppointer_gnss_count                 (void    ***ptr);                             // free memory and assign NULL to pointer to  gnss_count              double arrays
double pony_gnss_sat_epoch_diff_within_day(pony_time_epoch *t1, pony_time_epoch *t2);                       // difference between epochs within 24 hours
int    pony_gnss_sat_round(double x);                                                                       // round to the nearest integer














// plugin definitions

/* pony_gnss_sat_pos_vel_clock_gps - pony plugin

	Calculates position, velocity and clock correction for all GPS satellites with valid ephemeris.

	description:
		- ephemeris have to match RINEX format and order;
		- calculations comply with Sections 20.3.3.3.3 and 20.3.3.4.3 of IS-GPS-200J (22 May 2018);
		- if no single code measurement available, the calculated position refers to
		  receiver reference epoch (pony->gnss[].epoch) in ECEF at the same time instant;
		- if at least one code measurement available, the calculated position refers to
		  estimated time of signal emission (pony->gnss[].gps.sat[].t_em) in ECEF at estimated time of signal reception;
        - if receiver position-and-clock solution is available (pony->gnss[].sol.x_valid, pony->gnss[].sol.dt_valid),
		  ECEF frame is adjusted by receiver clock error, and satellite elevation (pony->gnss[].gps.sat[s].sinEl) becomes available;
		- all time intervals only less than 24 hours;
		- satellite position fit intervals only less than 6 hours.
	uses:
		pony->gnss_count
		pony->gnss[].epoch
		pony->gnss[].sol. x
		pony->gnss[].sol. x_valid
		pony->gnss[].sol.dt
		pony->gnss[].sol.dt_valid
		pony->gnss[].gps->max_sat_count
		pony->gnss[].gps->    obs_count
		pony->gnss[].gps->    obs_types
		pony->gnss[].gps->sat[].eph
		pony->gnss[].gps->sat[].eph_valid
		pony->gnss[].gps->sat[].obs_valid
		pony->gnss[].gps->sat[].  x_valid
	changes:
		pony->gnss[].gps->sat[]. t_em
		pony->gnss[].gps->sat[]. t_em_valid
		pony->gnss[].gps->sat[].    x
		pony->gnss[].gps->sat[].    x_valid
		pony->gnss[].gps->sat[].    v
		pony->gnss[].gps->sat[].    v_valid
		pony->gnss[].gps->sat[].Deltatsv
		pony->gnss[].gps->sat[].sinEl
		pony->gnss[].gps->sat[].sinEl_valid
		pony->gnss[].gps->sat[].  eph_valid
	cfg parameters:
		none
*/
void pony_gnss_sat_pos_vel_clock_gps(void)
{
	static double **allIODE = NULL; // issues of data, ephemeris, for each receiver and each satellite to check if they have changed
	// elements pre-calculated and constant between ephemeride issues to speed up epoch processing
	static double **allA = NULL, **alln = NULL, **allE = NULL, ***allaf = NULL;

	size_t r, s;


	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return;

	// init
	if (pony->mode == 0) {

		// allocate memory
		if (   !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gps (&allIODE)
			|| !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gps (&allA)
			|| !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gps (&alln)
			|| !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gps (&allE)
			|| !pony_gnss_sat_alloc_pppointer_gnss_count_sat_count_gps(&allaf,3)
			) {
			pony->mode = -1;
			return;
		}
		// initialize non-zeros
		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].gps == NULL) // do nothing if no gps data in the receiver available
				continue;
			for (s = 0; s < pony->gnss[r].gps->max_sat_count; s++) {
				allIODE	[r][s] = -1;
				allE	[r][s] = -20;
			}
		}

	}

	// terminate
	if (pony->mode < 0) {

		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&allIODE) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&allA   ) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&alln   ) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&allE   ) );
		pony_gnss_sat_free_pppointer_gnss_count_sat_count_gps( (void ****)(&allaf  ) );

		return;

	}

	// regular processing
	else {

		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].gps == NULL) // do nothing if no gps structure initialized in this receiver
				continue;
			// process single receiver
			pony_gnss_sat_pos_vel_clock_gps_single_receiver(&(pony->gnss[r]), allIODE[r], allA[r], alln[r], allE[r], allaf[r]);
		}

	}

}

/* pony_gnss_sat_pos_vel_clock_glo - pony plugin

	Calculates position, velocity and clock correction for all GLONASS satellites with valid ephemeris.

	description:
		- ephemeris have to match RINEX format and order;
		- calculations comply with Section A3.1.2 of ICD GLONASS Edition 5.1 2008;
		- if no single code measurement available, the calculated position refers to
		  receiver reference epoch (pony->gnss[].epoch) in ECEF at the same time instant;
		- if at least one code measurement available, the calculated position refers to
		  estimated time of signal emission (pony->gnss[].glo.sat[].t_em) in ECEF at estimated time of signal reception;
        - if receiver position-and-clock solution is available (pony->gnss[].sol.x_valid, pony->gnss[].sol.dt_valid),
		  ECEF frame is adjusted by receiver clock error, and satellite elevation (pony->gnss[].glo.sat[s].sinEl) becomes available;
		- all time intervals only less than 24 hours;
		- satellite position fit intervals only less than 6 hours.
	uses:
		pony->gnss_count
		pony->gnss[].epoch
		pony->gnss[].sol. x
		pony->gnss[].sol. x_valid
		pony->gnss[].sol.dt
		pony->gnss[].sol.dt_valid
		pony->gnss[].glo->max_sat_count
		pony->gnss[].glo->    obs_count
		pony->gnss[].glo->    obs_types
		pony->gnss[].glo->clock_corr
		pony->gnss[].glo->clock_corr_to
		pony->gnss[].glo->clock_corr_valid
		pony->gnss[].glo->sat[].eph
		pony->gnss[].glo->sat[].eph_valid
		pony->gnss[].glo->sat[].obs_valid
		pony->gnss[].glo->sat[].  x_valid
	changes:
		pony->gnss[].glo->sat[]. t_em
		pony->gnss[].glo->sat[]. t_em_valid
		pony->gnss[].glo->sat[].    x
		pony->gnss[].glo->sat[].    x_valid
		pony->gnss[].glo->sat[].    v
		pony->gnss[].glo->sat[].    v_valid
		pony->gnss[].glo->sat[].Deltatsv
		pony->gnss[].glo->sat[].sinEl
		pony->gnss[].glo->sat[].sinEl_valid
		pony->gnss[].glo->sat[].  eph_valid
	cfg parameters:
		none
*/
void pony_gnss_sat_pos_vel_clock_glo(void)
{
	const int rk4_state = 6, rk4_coeff = 4;
	static double
		**t ,			// at times t (sec) within a day [0 86400], for each receiver and each satellite:
		***x,			// last known positions X, Y, Z (m),
		***v,			// last known velocity components Vx, Vy, Vz (m/s),
		***a;			// last known extra acceleration components (m/s^2);
	static int **tb;	// toc of the ephemeris used (sec) within a day [0 86400]
	static double *rk4_y, **rk4_k; // memory array to be used in Runge-Kutta integration

	size_t r, s;


	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return;

	// init
	if (pony->mode == 0) {

		// allocate memory
		rk4_y = (double * )calloc( rk4_state, sizeof(double  ) );
		rk4_k = (double **)calloc( rk4_coeff, sizeof(double *) );
		if ( rk4_y == NULL || rk4_k == NULL
			|| !pony_gnss_sat_alloc_dppointer_gnss_count_sat_count_glo (&t   )
			|| !pony_gnss_sat_alloc_dpppointer_gnss_count_sat_count_glo(&x ,3)
			|| !pony_gnss_sat_alloc_dpppointer_gnss_count_sat_count_glo(&v ,3)
			|| !pony_gnss_sat_alloc_dpppointer_gnss_count_sat_count_glo(&a, 3)
			|| !pony_gnss_sat_alloc_ippointer_gnss_count_sat_count_glo (&tb  ) ) {
			pony->mode = -1;
			return;
		}
		for (r = 0; r < rk4_coeff; r++) {
			rk4_k[r] = (double *)calloc( rk4_state, sizeof(double) );
			if (rk4_k[r] == NULL) {
				pony->mode = -1;
				return;
			}
		}
		// init tb with -1 - undefined seconds of the day
		for (r = 0; r < pony->gnss_count; r++)
			if (pony->gnss[r].glo != NULL)
				for (s = 0; s < pony->gnss[r].glo->max_sat_count; s++)
					tb[r][s] = -1;

	}

	// terminate
	if (pony->mode < 0) {

		pony_gnss_sat_free_ppointer_gnss_count               ( (void *** )(&t ) );
		pony_gnss_sat_free_pppointer_gnss_count_sat_count_glo( (void ****)(&x ) );
		pony_gnss_sat_free_pppointer_gnss_count_sat_count_glo( (void ****)(&v ) );
		pony_gnss_sat_free_pppointer_gnss_count_sat_count_glo( (void ****)(&a ) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void *** )(&tb) );

		return;

	}

	// regular processing
	else {

		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].glo == NULL) // do nothing if no glonass data initialized in this receiver
				continue;
			// process single receiver
			pony_gnss_sat_pos_vel_clock_glo_single_receiver(&(pony->gnss[r]),   t[r], x[r], v[r], a[r], tb[r],  rk4_y, rk4_k);
		}

	}

}

/* pony_gnss_sat_pos_vel_clock_gal - pony plugin

	Calculates position, velocity and clock correction for all Galileo satellites with valid ephemeris.

	description:
		- ephemeris have to match RINEX format and order;
		- calculations comply with Sections 5.1.1 and 5.1.4 of Galileo OS SIS ICD Issue 1.2 (November 2015);
		- if no single code measurement available, the calculated position refers to
		  receiver reference epoch (pony->gnss[].epoch) in ECEF at the same time instant;
		- if at least one code measurement available, the calculated position refers to
		  estimated time of signal emission (pony->gnss[].glo.sat[].t_em) in ECEF at estimated time of signal reception;
        - if receiver position-and-clock solution is available (pony->gnss[].sol.x_valid, pony->gnss[].sol.dt_valid),
		  ECEF frame is adjusted by receiver clock error, and satellite elevation (pony->gnss[].glo.sat[s].sinEl) becomes available;
		- all time intervals only less than 24 hours;
		- satellite position fit intervals only less than 6 hours.
	uses:
		pony->gnss_count
		pony->gnss[].epoch
		pony->gnss[].sol. x
		pony->gnss[].sol. x_valid
		pony->gnss[].sol.dt
		pony->gnss[].sol.dt_valid
		pony->gnss[].gal->max_sat_count
		pony->gnss[].gal->    obs_count
		pony->gnss[].gal->    obs_types
		pony->gnss[].gal->sat[].eph
		pony->gnss[].gal->sat[].eph_valid
		pony->gnss[].gal->sat[].obs_valid
		pony->gnss[].gal->sat[].  x_valid
	changes:
		pony->gnss[].gal->sat[]. t_em
		pony->gnss[].gal->sat[]. t_em_valid
		pony->gnss[].gal->sat[].    x
		pony->gnss[].gal->sat[].    x_valid
		pony->gnss[].gal->sat[].    v
		pony->gnss[].gal->sat[].    v_valid
		pony->gnss[].gal->sat[].Deltatsv
		pony->gnss[].gal->sat[].sinEl
		pony->gnss[].gal->sat[].sinEl_valid
		pony->gnss[].gal->sat[].  eph_valid
	cfg parameters:
		none
*/
void pony_gnss_sat_pos_vel_clock_gal(void)
{
	static double **allIODnav = NULL; // issues of data, ephemeris, for each receiver and each satellite to check if they have changed
	// elements pre-calculated and constant between ephemeride issues to speed up epoch processing
	static double **allA = NULL, **alln = NULL, **allE = NULL, ***allaf = NULL;

	size_t r, s;


	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return;

	// init
	if (pony->mode == 0) {

		// allocate memory
		if (   !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gal (&allIODnav)
			|| !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gal (&allA     )
			|| !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gal (&alln     )
			|| !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gal (&allE     )
			|| !pony_gnss_sat_alloc_pppointer_gnss_count_sat_count_gal(&allaf, 3 )
			) {
			pony->mode = -1;
			return;
		}
		// initialize non-zeros
		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].gal == NULL) // do nothing if no galileo data in the receiver available
				continue;
			for (s = 0; s < pony->gnss[r].gal->max_sat_count; s++) {
				allIODnav	[r][s] = -1;
				allE		[r][s] = -20;
			}
		}

	}

	// terminate
	if (pony->mode < 0) {

		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&allIODnav) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&allA     ) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&alln     ) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&allE     ) );
		pony_gnss_sat_free_pppointer_gnss_count_sat_count_gal( (void ****)(&allaf    ) );

		return;

	}

	// regular processing
	else {

		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].gal == NULL) // do nothing if no galileo structure initialized in this receiver
				continue;
			// process single receiver
			pony_gnss_sat_pos_vel_clock_gal_single_receiver(&(pony->gnss[r]), allIODnav[r], allA[r], alln[r], allE[r], allaf[r]);
		}

	}

}

/* pony_gnss_sat_pos_vel_clock_bds - pony plugin

	Calculates position, velocity and clock correction for all BeiDou MEO/IGSO satellites with valid ephemeris.

	description:
		- ephemeris have to match RINEX format and order;
		- calculations comply with Sections 5.2.4.10 and 5.2.4.12 of BeiDou SIS ICD OSS Version 2.1 (November 2016);
		- if no single code measurement available, the calculated position refers to
		  receiver reference epoch (pony->gnss[].epoch) in ECEF at the same time instant;
		- if at least one code measurement available, the calculated position refers to
		  estimated time of signal emission (pony->gnss[].glo.sat[].t_em) in ECEF at estimated time of signal reception;
        - if receiver position-and-clock solution is available (pony->gnss[].sol.x_valid, pony->gnss[].sol.dt_valid),
		  ECEF frame is adjusted by receiver clock error, and satellite elevation (pony->gnss[].glo.sat[s].sinEl) becomes available;
		- all time intervals only less than 24 hours;
		- satellite position fit intervals only less than 6 hours.
	uses:
		pony->gnss_count
		pony->gnss[].epoch
		pony->gnss[].sol. x
		pony->gnss[].sol. x_valid
		pony->gnss[].sol.dt
		pony->gnss[].sol.dt_valid
		pony->gnss[].bds->max_sat_count
		pony->gnss[].bds->    obs_count
		pony->gnss[].bds->    obs_types
		pony->gnss[].bds->sat[].eph
		pony->gnss[].bds->sat[].eph_valid
		pony->gnss[].bds->sat[].obs_valid
		pony->gnss[].bds->sat[].  x_valid
	changes:
		pony->gnss[].bds->sat[]. t_em
		pony->gnss[].bds->sat[]. t_em_valid
		pony->gnss[].bds->sat[].    x
		pony->gnss[].bds->sat[].    x_valid
		pony->gnss[].bds->sat[].    v
		pony->gnss[].bds->sat[].    v_valid
		pony->gnss[].bds->sat[].Deltatsv
		pony->gnss[].bds->sat[].sinEl
		pony->gnss[].bds->sat[].sinEl_valid
		pony->gnss[].bds->sat[].  eph_valid
	cfg parameters:
		none
*/
void pony_gnss_sat_pos_vel_clock_bds(void)
{
	static double **allSOW = NULL; // seconds of the week of the NAV message, for each receiver and each satellite to check if they have changed
	// elements pre-calculated and constant between ephemeride issues to speed up epoch processing
	static double **allA = NULL, **alln = NULL, **allE = NULL, ***alla = NULL;

	size_t r, s;


	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return;

	// init
	if (pony->mode == 0) {

		// allocate memory
		if (   !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_bds (&allSOW)
			|| !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_bds (&allA)
			|| !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_bds (&alln)
			|| !pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_bds (&allE)
			|| !pony_gnss_sat_alloc_pppointer_gnss_count_sat_count_bds(&alla,3)
			) {
			pony->mode = -1;
			return;
		}
		// initialize non-zeros
		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].bds == NULL) // do nothing if no beidou data in the receiver available
				continue;
			for (s = 0; s < pony->gnss[r].bds->max_sat_count; s++) {
				allSOW	[r][s] = -1;
				allE	[r][s] = -20;
			}
		}

	}

	// terminate
	if (pony->mode < 0) {

		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&allSOW) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&allA  ) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&alln  ) );
		pony_gnss_sat_free_ppointer_gnss_count               ( (void  ***)(&allE  ) );
		pony_gnss_sat_free_pppointer_gnss_count_sat_count_bds( (void ****)(&alla  ) );

		return;

	}

	// regular processing
	else {

		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].bds == NULL) // do nothing if no beidou structure initialized in this receiver
				continue;
			// process single receiver
			pony_gnss_sat_pos_vel_clock_bds_single_receiver(&(pony->gnss[r]), allSOW[r], allA[r], alln[r], allE[r], alla[r]);
		}

	}

}








// internal routines
	// single-receiver satellite calculations for GPS
void pony_gnss_sat_pos_vel_clock_gps_single_receiver(pony_gnss *gnss, double *allIODE, double *allA, double *alln, double *allE, double **allaf)
{
	const double
		radius_squared_min   = 650e12,           //  m^2,    approx. (25.5e6)^2
		radius_squared_max   = 756e12,           //  m^2,    approx. (27.5e6)^2
		 speed_squared_min   =  13e6,            // (m/s)^2, approx. ( 3.6e3)^2
		 speed_squared_max   =  18e6,            // (m/s)^2, approx. ( 4.1e3)^2
		max_fit_interval     = 6*3600,           // maximum fit interval, seconds
		anomaly_precision    = 1.0/0xe000000000; // threshold to stop Kepler's equation iterations, radians, roughly 1e-12
	const int max_iterations = 32;				 // threshold to stop Kepler's equation iterations
	const char   code_id     = 'C';

	// ephemeris as in Table 20-I of IS-GPS-200J (22 May 2018) p. 95
	// (!) toc fit intervals only less than 24 hours supported yet
	// (!) toc expressed as pony_time_epoch: Year, Month, Day, hours, minutes, seconds
	int CL2, week, L2Pflag, SVa, SVh, IODC;
	pony_time_epoch toc;
	double TGD, af2, af1, af0;

	// subframe 1 parameters as in Section 20.3.3.3.3 of IS-GPS-200J (22 May 2018) p. 95
	double t_toc, Deltatr; // Deltatsv

	// ephemeris as in Table 20-III of IS-GPS-200J (22 May 2018) p. 103,
	// (!) Deltan, Omdot, idot - in rad/s, according to RINEX
	// (!) M0, Om0, i0, om - in radians, according to RINEX
	// (!) toe is seconds of GPS week, a fractional part to go with week number, according to RINEX
	double IODE, Crs, Deltan, M0, Cuc, e, Cus, sqrtA, toe, Cic, Om0, Cis, i0, Crc, om, Omdot, idot;

	// elements of coordinate systems as in Table 20-IV of IS-GPS-200J (22 May 2018) p. 104, except for pre-calculated ones
	double // A,
		n0, // t,
		tk, // n,
		Mk, Ek, nuk, Phik, duk, drk, dik, uk, rk, ik, xk_, yk_, Omk, xk, yk, zk;

	// their derivatives for velocity calculation
	double
		nuk_dot, duk_dot, drk_dot, dik_dot;

	size_t i, s;
	double sqrt1e2, sinE, one_ecosE, cos2Phik, sin2Phik, cuk, suk, cOmk, sOmk, cik, sik, r, dx, ut, cut, sut, vr, vu, v0[3], v[3];
	pony_gnss_sat *sat; // current satellite


	// validate
	if (gnss == NULL || allIODE == NULL || allA == NULL || alln == NULL || allE == NULL || allaf == NULL)
		return;

	// requires gps and sat structure to be initialized
	if (gnss->gps == NULL || gnss->gps->sat == NULL)
		return;

	// run through all gps satellites
	for (s = 0; s < gnss->gps->max_sat_count; s++)
	{
		sat = &(gnss->gps->sat[s]);
		if (!sat->eph_valid || sat->eph == NULL) // do nothing if no valid ephemeris available
			continue;

		// ephemeris 0 -- 5  - toc (see RINEX documentation)
		toc.Y	= pony_gnss_sat_round(sat->eph[0]);
		toc.M	= pony_gnss_sat_round(sat->eph[1]);
		toc.D	= pony_gnss_sat_round(sat->eph[2]);
		toc.h	= pony_gnss_sat_round(sat->eph[3]);
		toc.m	= pony_gnss_sat_round(sat->eph[4]);
		toc.s	=                     sat->eph[5] ;

		// check fit interval
		// t - toc, time elapsed since the time of clock
		// (!) intervals only less than 24 hours are supported
		t_toc = pony_gnss_sat_epoch_diff_within_day( &(gnss->epoch), &(toc) );
		if (fabs(t_toc) >= max_fit_interval)
				continue;

		// all other ephemeris (see RINEX documentation & GPS interface specifications)
		af0		=                     sat->eph[ 6] ;
		af1		=                     sat->eph[ 7] ;
		af2		=                     sat->eph[ 8] ;
		IODE	=                     sat->eph[ 9] ;
		Crs		=                     sat->eph[10] ;
		Deltan	=                     sat->eph[11] ;
		M0		=                     sat->eph[12] ;
		Cuc		=                     sat->eph[13] ;
		e		=                     sat->eph[14] ;
		Cus		=                     sat->eph[15] ;
		sqrtA	=                     sat->eph[16] ;
		toe		=                     sat->eph[17] ;
		Cic		=                     sat->eph[18] ;
		Om0		=                     sat->eph[19] ;
		Cis		=                     sat->eph[20] ;
		i0		=                     sat->eph[21] ;
		Crc		=                     sat->eph[22] ;
		om		=                     sat->eph[23] ;
		Omdot	=                     sat->eph[24] ;
		idot	=                     sat->eph[25] ;
		CL2		= pony_gnss_sat_round(sat->eph[26]); // Codes on L2 channel, skip this field, not analyzed yet
		week	= pony_gnss_sat_round(sat->eph[27]);
		L2Pflag	= pony_gnss_sat_round(sat->eph[28]); // L2 P data flag, skip this field, not analyzed yet
		SVa		= pony_gnss_sat_round(sat->eph[29]); // SV accuracy, skip this field, not analyzed yet
		SVh	    = pony_gnss_sat_round(sat->eph[30]); // SV health
		TGD		=                     sat->eph[31] ;
		IODC	= pony_gnss_sat_round(sat->eph[32]);

		if (SVh) {
				// drop all flags and skip calculations if SV is not OK
				sat->eph_valid		= 0;
				sat->t_em_valid		= 0;
				sat->x_valid		= 0;
				sat->v_valid		= 0;
				sat->sinEl_valid	= 0;
				continue;
		}
		// recalculate stored quantities if new issue of data detected
		if ( pony_gnss_sat_round(IODE) != pony_gnss_sat_round(allIODE[s]) ) {
			allIODE[s] = IODE;
			// quantities that are constant between issues of ephemeris
			allA [s]	= sqrtA*sqrtA;
			n0			= sqrt(pony->gnss_const.gps.mu)/(sqrtA*sqrtA*sqrtA);
			alln [s]	= n0 + Deltan;
			allaf[s][0]	= af0;
			allaf[s][1]	= af1;
			allaf[s][2]	= af2;
		}

		// t_em - time of emission
		// (!) fit intervals only less than 12 hours are supported yet
			// toe - time of epoch treated as t_em base approximation
		sat->t_em = gnss->epoch.h*3600.0 + gnss->epoch.m*60.0 + gnss->epoch.s;
			// look for any valid code measurement to compensate for travel time
		ut = 0; // ECEF frame rotation angle
		for (i = 0; i < gnss->gps->obs_count; i++)
			if (gnss->gps->obs_types[i][0] == code_id && sat->obs_valid[i]) {
				ut = sat->obs[i]/pony->gnss_const.c; // base estimate for travel time
				if (sat->x_valid) // if position is already available, satellite clock correction applies
					ut += sat->Deltatsv;
				sat->t_em -= ut; // estimate for the time of emission in the system time frame
				sat->t_em_valid = 1;

				if (gnss->sol.dt_valid) // if receiver clock error estimated, receiver clock correction applies
					ut -= gnss->sol.dt;
				ut *= pony->gnss_const.gps.u; // ECEF frame rotation angle between emission and reception
				break;
			}


		// satellite position
		// (!) fit intervals only less than 12 hours are supported yet

			// tk - time from ephemeris reference epoch
		tk = sat->t_em - fmod(toe, pony->gnss_const.sec_in_d);
		if (tk < -pony->gnss_const.sec_in_d/2) // day rollover - forward
			tk += pony->gnss_const.sec_in_d;
		if (tk >  pony->gnss_const.sec_in_d/2) // day rollover - backward
			tk -= pony->gnss_const.sec_in_d;

			// Mk - mean anomaly at current time
		Mk = M0 + alln[s]*tk;

			// Kepler equation by iterations
		if (allE[s] < -10) // if no previous estimate available, take the mean motion
			allE[s] = Mk;
		i = 0;
		do {
			Ek = allE[s];
			allE[s] = Mk + e*sin(Ek);
			i++;
		} while (fabs(Ek - allE[s]) > anomaly_precision && i < max_iterations);

			// nuk - true anomaly
		sqrt1e2 = sqrt(1-e*e);
		sinE = sin(Ek);
		nuk = atan2( sqrt1e2*sinE, cos(Ek)-e );
			// Phik - argument of latitude at current time
		Phik = nuk + om;
			// second harmonic perturbations
		cos2Phik = cos(2*Phik); sin2Phik = sin(2*Phik);
		duk = Cus*sin2Phik + Cuc*cos2Phik; // duk - argument of latitude correction
		drk = Crs*sin2Phik + Crc*cos2Phik; // drk - radius correction
		dik = Cis*sin2Phik + Cic*cos2Phik; // dik - inclination correction
			// corrected argument of latitude
		uk = Phik + duk;
			// corrected radius
		one_ecosE = 1 - e*cos(Ek);
		rk = allA[s]*one_ecosE + drk;
			// corrected inclination
		ik = i0 + dik + idot*tk;
			// position in orbital plane
		cuk = cos(uk); suk = sin(uk);
		xk_ = rk*cuk;
		yk_ = rk*suk;
			// corrected longitude of ascending node
		Omk = Om0 + (Omdot - pony->gnss_const.gps.u)*tk - pony->gnss_const.gps.u*toe; // gnss->gps_const.u stands for Omedot

			// Earth-fixed coordinates
		cOmk = cos(Omk); sOmk = sin(Omk);
		cik = cos(ik); sik = sin(ik);
		xk = xk_*cOmk - yk_*cik*sOmk;
		yk = xk_*sOmk + yk_*cik*cOmk;
		zk = yk_*sik;

		// radius to be checked
		r = xk*xk + yk*yk + zk*zk;

		// rotate the coordinate frame to the time of reception if t_em was corrected
		cut = cos(ut);
		sut = sin(ut);
		sat->x[0] =  xk*cut + yk*sut;
		sat->x[1] = -xk*sut + yk*cut;
		sat->x[2] =  zk;
		if (radius_squared_min < r && r < radius_squared_max)
			sat->x_valid = 1;

		// SV clock correction as in 20.3.3.3.3.1 of IS-GPS-200J (22 May 2018), p. 96
		// (!) fit intervals only less than 12 hours are supported yet
			// t_toc = t - t0c is calculated above
			// relativistic correction
		Deltatr = pony->gnss_const.gps.F*e*sqrtA*sinE;
			// SV PRN code phase time offset
		sat->Deltatsv = allaf[s][0] + allaf[s][1]*t_toc + allaf[s][2]*t_toc*t_toc + Deltatr;

		// check if position is available to calculate elevation
		if (sat->x_valid && gnss->sol.x_valid) {
			for (i = 0, r = 0.0, sat->sinEl = 0; i < 3; i++) {
				dx = sat->x[i] - gnss->sol.x[i];
				sat->sinEl += gnss->sol.x[i]*dx;
				r += dx*dx;
			}
			r = sqrt(r);
			sat->sinEl /= r;

			for (i = 0, r = 0.0; i < 3; i++)
				r += gnss->sol.x[i]*gnss->sol.x[i];
			r = sqrt(r);
			sat->sinEl /= r;
			sat->sinEl_valid = 1;
		}
		else
			sat->sinEl_valid = 0;

		// velocity calculation
			// derivatives
		nuk_dot = alln[s]*sqrt1e2/(one_ecosE*one_ecosE);
		duk_dot = 2*(Cus*cos2Phik - Cuc*sin2Phik)*nuk_dot;
		drk_dot = 2*(Crs*cos2Phik - Crc*sin2Phik)*nuk_dot;
		dik_dot = 2*(Cis*cos2Phik - Cic*sin2Phik)*nuk_dot;
			// absolute velocity components in orbital plane
		vr = allA[s]*alln[s]*e*sinE/one_ecosE + drk_dot;
		vu = rk*(nuk_dot + duk_dot);
		v0[0] =	vr*cuk - vu*suk;
		v0[1] = vr*suk + vu*cuk;
		v0[2] = rk*sin(nuk)*(idot + dik_dot);
			// absolute velocity components in ECEF
		v[0] = v0[0]*cOmk -		v0[1]*cik*sOmk +	v0[2]*sik*sOmk;
		v[1] = v0[0]*sOmk +		v0[1]*cik*cOmk -	v0[2]*sik*cOmk;
		v[2] =					v0[1]*sik +			v0[2]*cik;
			// speed to be checked
		r = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
			// relative to ECEF velocity components
		sat->v[0] = v[0] + (pony->gnss_const.gps.u - Omdot)*sat->x[1];
		sat->v[1] = v[1] - (pony->gnss_const.gps.u - Omdot)*sat->x[0];
		sat->v[2] = v[2];

		if (speed_squared_min < r && r < speed_squared_max)
			sat->v_valid = 1;
		else
			sat->v_valid = 0;
	}

}


	// single-receiver satellite calculations for GLONASS
void pony_gnss_sat_pos_vel_clock_glo_single_receiver(pony_gnss *gnss,  double *t, double **x, double **v, double **a, int *tb,  double *rk4_y, double **rk4_k)
{
	const double
		radius_squared_min = 600e12, //  m^2,    (24.5e6)^2
		radius_squared_max = 702e12, //  m^2,    (26.5e6)^2
		 speed_squared_min =   3e6,  // (m/s)^2, ( 1.8e3)^2 - relative to the Earth (+/- 1.9e3 m/s in equatorial plane)
		 speed_squared_max =  37e6,  // (m/s)^2, ( 6.1e3)^2 - relative to the Earth (+/- 1.9e3 m/s in equatorial plane)
		max_dt             = 1,      // maximum integration step size, sec
		max_fit_interval   = 6*3600; // maximum fit interval, seconds
	const char
		code_id            = 'C',
		utc_time_id[]      = "UT";

	// ephemeris as in Table 4.5 of ICD GLONASS Edition 5.1 2008
	double gamma_n, tau_n;	// clock correction parameters
	int l_n;				// satellite health
	// almanac data as in Table 4.9 of ICD GLONASS Edition 5.1 2008
	double tau_c;

	// (!) toc fit intervals only less than 24 hours supported yet
	// (!) toc expressed as pony_time_epoch: Year, Month, Day, hours, minutes, seconds
	pony_time_epoch toc;
	pony_gnss_sat *sat;
	double tem_tb, tem_t, dt;
	double r, dx, vel, ut, cut, sut;
	int tb_new;
	size_t i, k, s;


	// validate
	if (gnss == NULL || t == NULL || x == NULL || v == NULL || a == NULL || tb == NULL || rk4_y == NULL || rk4_k == NULL)
		return;

	// requires glo and sat structure to be initialized
	if (gnss->glo == NULL || gnss->glo->sat == NULL)
		return;

	// run through all glonass satellites
	for (s = 0; s < gnss->glo->max_sat_count; s++) {
		sat = &(gnss->glo->sat[s]);
		if (!sat->eph_valid || sat->eph == NULL) // do nothing if no valid ephemeris available
			continue;

		// ephemeris 0 -- 5  - current toc (see RINEX documentation)
		toc.Y  = pony_gnss_sat_round(sat->eph[0]);
		toc.M  = pony_gnss_sat_round(sat->eph[1]);
		toc.D  = pony_gnss_sat_round(sat->eph[2]);
		toc.h  = pony_gnss_sat_round(sat->eph[3]);
		toc.m  = pony_gnss_sat_round(sat->eph[4]);
		toc.s  =                     sat->eph[5] ;
		tb_new = pony_gnss_sat_round(toc.h*3600.0 + toc.m*60 + toc.s);

		// check tb
		if (tb[s] < 0 || tb[s] != tb_new) { // new ephemeris have come

			l_n = pony_gnss_sat_round(sat->eph[12]);	// satellite health

			if (l_n) {
				// drop all flags and skip calculations if SV is not OK
				sat->eph_valid		= 0;
				sat->t_em_valid		= 0;
				sat->x_valid		= 0;
				sat->v_valid		= 0;
				sat->sinEl_valid	= 0;
				tb[s] = -1;
				continue;
			}

			tb[s] = tb_new;
			t [s] = tb_new;
			for (i = 0, k = 9; i < 3 && k < gnss->glo->max_eph_count-4; i++, k += 4) {
				x[s][i] = 1e3*sat->eph[k  ];	// coordinates
				v[s][i] = 1e3*sat->eph[k+1];	// velocity components
				a[s][i] = 1e3*sat->eph[k+2];	// extra acceleration components
			}
			gnss->glo->freq_slot[s]	= pony_gnss_sat_round(sat->eph[16]);	// frequency number
		}

		tau_n	= -sat->eph[6];	// clock bias
		gamma_n	=  sat->eph[7];	// clock drift rate

		// check fit interval
		// (!) intervals only less than 6 hours are supported
		if (fabs( pony_gnss_sat_epoch_diff_within_day( &(gnss->epoch), &(toc) ) ) >= max_fit_interval) {
				//sat->eph_valid = 0;
				continue;
		}

		// t_em - time of emission
		// (!) fit intervals only less than 12 hours are supported yet
			// toe - time of epoch treated as t_em base approximation
		sat->t_em = gnss->epoch.h*3600.0 + gnss->epoch.m*60.0 + gnss->epoch.s - ( (gnss->leap_sec_valid)? gnss->leap_sec : 0);
			// look for any valid code measurement to compensate for travel time
		ut = 0; // ECEF frame rotation angle
		for (i = 0; i < gnss->glo->obs_count; i++)
			if (gnss->glo->obs_types[i][0] == code_id && sat->obs_valid[i]) {
				ut = sat->obs[i]/pony->gnss_const.c; // base estimate for travel time
				if (sat->x_valid) // if position is already available, satellite clock correction applies
					ut += sat->Deltatsv;
				sat->t_em -= ut; // estimate for the time of emission int hte system time frame
				sat->t_em_valid = 1;

				if (gnss->sol.dt_valid) // if receiver clock error estimated, receiver clock correction applies
					ut -= gnss->sol.dt;
				ut *= pony->gnss_const.glo.u; // ECEF frame rotation angle between emission and reception
				break;
			}

			// satellite position & velocity
		// (!) fit intervals only less than 12 hours are supported yet
			// t_em - tb, time elapsed since the time of ephemeris
		tem_tb = sat->t_em - tb[s];
		if (tem_tb < -pony->gnss_const.sec_in_d/2) // day rollover - forward
			tem_tb += pony->gnss_const.sec_in_d;
		if (tem_tb >  pony->gnss_const.sec_in_d/2) // day rollover - backward
			tem_tb -= pony->gnss_const.sec_in_d;
			// tem - t, time elapsed since the last known position
		tem_t = sat->t_em - t[s];
		if (tem_t  < -pony->gnss_const.sec_in_d/2) // day rollover - forward
			tem_t  += pony->gnss_const.sec_in_d;
		if (tem_t  >  pony->gnss_const.sec_in_d/2) // day rollover - backward
			tem_t  -= pony->gnss_const.sec_in_d;
			// choose starting position
		if ( fabs(tem_tb) < fabs(tem_t) ) { // closer to time of ephemeris -- start from tb
			t[s] = tb[s];
			tem_t = tem_tb;
			for (i = 0, k = 9; i < 3 && k < gnss->glo->max_eph_count-4; i++, k += 4) {
				x[s][i] = 1e3*sat->eph[k  ];	// coordinates
				v[s][i] = 1e3*sat->eph[k+1];	// velocity components
				a[s][i] = 1e3*sat->eph[k+2];	// extra acceleration components
			}
		}

			// integrate from t, step 1 sec
		dt = tem_t;
		while ( fabs(dt) > max_dt) {
			dt = (dt > 0)? max_dt : -max_dt;
			pony_gnss_sat_glo_motion_rk4_step(x[s], v[s], a[s],  dt,  rk4_y, rk4_k);
			t[s] += dt;
			dt = sat->t_em - t[s];
			if (dt < -pony->gnss_const.sec_in_d/2) // day rollover - forward
				dt += pony->gnss_const.sec_in_d;
			if (dt >  pony->gnss_const.sec_in_d/2) // day rollover - backward
				dt -= pony->gnss_const.sec_in_d;
		}
		pony_gnss_sat_glo_motion_rk4_step(x[s], v[s], a[s],  dt,  rk4_y, rk4_k);
		t[s] += dt;

			// radius and speed to be checked
		for (i = 0, r = 0, vel = 0; i < 3; i++) {
			r   += x[s][i]*x[s][i];
			vel += v[s][i]*v[s][i];
		}

			// rotate the coordinate frame to the time of reception if t_em was corrected
		cut = cos(ut);
		sut = sin(ut);
		sat->x[0] =  x[s][0]*cut + x[s][1]*sut;
		sat->x[1] = -x[s][0]*sut + x[s][1]*cut;
		sat->x[2] =  x[s][2];
		if (radius_squared_min < r && r < radius_squared_max)
			sat->x_valid = 1;
		sat->v[0] =  v[s][0]*cut + v[s][1]*sut;
		sat->v[1] = -v[s][0]*sut + v[s][1]*cut;
		sat->v[2] =  v[s][2];
		if (speed_squared_min < vel && vel < speed_squared_max)
			sat->v_valid = 1;

		// SV clock correction as in Section 3.3.3 of ICD GLONASS Edition 5.1 2008, minus sign, tau_c if present in gnss->glo->clock_corr
		if (gnss->glo->clock_corr_valid && gnss->glo->clock_corr_to[0] == utc_time_id[0] && gnss->glo->clock_corr_to[1] == utc_time_id[1])
			tau_c = -gnss->glo->clock_corr[0];
		else
			tau_c = 0;

		sat->Deltatsv = -tau_n + gamma_n*tem_tb - tau_c;

		// check if position is available to calculate elevation
		if (sat->x_valid && gnss->sol.x_valid) {
			for (i = 0, r = 0.0, sat->sinEl = 0; i < 3; i++) {
				dx = sat->x[i] - gnss->sol.x[i];
				sat->sinEl += gnss->sol.x[i]*dx;
				r += dx*dx;
			}
			r = sqrt(r);
			sat->sinEl /= r;

			for (i = 0, r = 0.0; i < 3; i++)
				r += gnss->sol.x[i]*gnss->sol.x[i];
			r = sqrt(r);
			sat->sinEl /= r;
			sat->sinEl_valid = 1;
		}
		else
			sat->sinEl_valid = 0;
	}

}

void pony_gnss_sat_glo_motion_rk4_step(double *x, double *v, double *a,  double dt, double *y, double **k)
{
	double dt_2, dt_6;
	size_t i;

	// validate
	if (x == NULL || v == NULL || a == NULL || y == NULL || k == NULL)
		return;

	// time steps
	dt_2 = dt/2;
	dt_6 = dt/6;

	// k1
	for (i = 0; i < 3; i++) {
		y[i  ] = x[i];
		y[3+i] = v[i];
	}
	pony_gnss_sat_glo_motion_ode_fun(k[0],  y, a);

	// k2
	for (i = 0; i < 3; i++) {
		y[i  ] = x[i] + dt_2*k[0][  i];
		y[3+i] = v[i] + dt_2*k[0][3+i];
	}
	pony_gnss_sat_glo_motion_ode_fun(k[1],  y, a);

	// k3
	for (i = 0; i < 3; i++) {
		y[i  ] = x[i] + dt_2*k[1][  i];
		y[3+i] = v[i] + dt_2*k[1][3+i];
	}
	pony_gnss_sat_glo_motion_ode_fun(k[2],  y, a);

	// k4
	for (i = 0; i < 3; i++) {
		y[i  ] = x[i] + dt*k[2][  i];
		y[3+i] = v[i] + dt*k[2][3+i];
	}
	pony_gnss_sat_glo_motion_ode_fun(k[3],  y, a);

	// y += dt/6*(k1 + 2*k2 + 2*k3 + k4)
	for (i = 0; i < 3; i++) {
		x[i] += dt_6*(k[0][  i] + 2*k[1][  i] + 2*k[2][  i] + k[3][  i]);
		v[i] += dt_6*(k[0][3+i] + 2*k[1][3+i] + 2*k[2][3+i] + k[3][3+i]);
	}
}

	// equations of satellite motion as in ICD GLONASS Edition 5.1 2008
void pony_gnss_sat_glo_motion_ode_fun(double *dy,  double *y, double *a)
{
	double r, r3, r2, mu_r3, J02x3_2xa2_r2, z2_r2x5, u2;
	size_t i;

	// validate
	if (dy == NULL || y == NULL || a == NULL)
		return;
	// calculate
	for (i = 0, r = 0; i < 3; i++) {
		r += y[i]*y[i];
		dy[i] = y[3+i];																					// dx/dt = Vx, dy/dt = Vy, dz/dt = Vz
	}
	r = sqrt(r);																						// r = sqrt(x^2 + y^2 + z^2)

	r2 = r*r;																							// r^2
	r3 = r2*r;																							// r^3
	mu_r3 = pony->gnss_const.glo.mu/r3;																	// mu/r^3
	J02x3_2xa2_r2 = 1.5*pony->gnss_const.glo.J02*pony->gnss_const.glo.a*pony->gnss_const.glo.a/r2;		// 3/2*J02*mu*a^2/r^5
	z2_r2x5 = 5*y[2]*y[2]/r2;																			// 5*z^2/r^2
	u2 = pony->gnss_const.glo.u*pony->gnss_const.glo.u;													// u^2

	dy[3] = (-mu_r3*(1 + J02x3_2xa2_r2*(1-z2_r2x5)) + u2)*y[0] + 2*pony->gnss_const.glo.u*y[4]	+ a[0];	// -mu/r^3*x - 3/2*J02*mu*a^2/r^5*x*(1-5*z^2/r^2) + u^2*x + 2*u*Vy + ddx
	dy[4] = (-mu_r3*(1 + J02x3_2xa2_r2*(1-z2_r2x5)) + u2)*y[1] - 2*pony->gnss_const.glo.u*y[3]	+ a[1];	// -mu/r^3*y - 3/2*J02*mu*a^2/r^5*y*(1-5*z^2/r^2) + u^2*y - 2*u*Vx + ddy
	dy[5] = (-mu_r3*(1 + J02x3_2xa2_r2*(3-z2_r2x5))     )*y[2]									+ a[2];	// -mu/r^3*z - 3/2*J02*mu*a^2/r^5*z*(3-5*z^2/r^2)                  + ddz

}


	// single-receiver satellite calculations for Galileo
void pony_gnss_sat_pos_vel_clock_gal_single_receiver(pony_gnss *gnss, double *allIODnav, double *allA, double *alln, double *allE, double **allaf)
{
	const double
		radius_squared_min    = 812e12,           //  m^2,    approx. (28.5e6)^2
		radius_squared_max    = 930e12,           //  m^2,    approx. (30.5e6)^2
		 speed_squared_min    =  11e6,            // (m/s)^2, approx. ( 3.3e3)^2
		 speed_squared_max    =  15e6,            // (m/s)^2, approx. ( 3.9e3)^2
		max_fit_interval      = 6*3600,           // maximum fit interval, seconds
		anomaly_precision     = 1.0/0xe000000000; // threshold to stop Kepler's equation iterations, radians, roughly 1e-12
	const int  max_iterations = 32;               // threshold to stop Kepler's equation iterations
	const char code_id        = 'C';

	// ephemeris as in Table 60 of Galileo OS SIS ICD Issue 1.2 (November 2015) p. 46
	// (!) toc fit intervals only less than 24 hours supported yet
	// (!) toc expressed as pony_time_epoch: Year, Month, Day, hours, minutes, seconds
	unsigned int DS, week, SVh, tow;
	pony_time_epoch toc;
	double BGDE5aE1, BGDE5bE1, SISA, af2, af1, af0;

	// clock correction parameters as in Section 5.1.4 of Galileo OS SIS ICD Issue 1.2 (November 2015) p. 46
	double t_toc, Deltatr; // Deltatsv

	// ephemeris as in Table 57 of Galileo OS SIS ICD Issue 1.2 (November 2015) p. 43,
	// (!) Deltan, Omdot, idot - in rad/s, according to RINEX
	// (!) M0, Om0, i0, om - in radians, according to RINEX
	// (!) toe is seconds of Galileo week, a fractional part to go with week number, according to RINEX
	double IODnav, Crs, Deltan, M0, Cuc, e, Cus, sqrtA, toe, Cic, Om0, Cis, i0, Crc, om, Omdot, idot;

	// elements of coordinate systems as in Table 58 of Galileo OS SIS ICD Issue 1.2 (November 2015) p. 44, except for pre-calculated ones
	double // A,
		n0, // t,
		tk, // n,
		M, E, nu, Phi, du, dr, di, u, r, I, x_, y_, Om, x, y, z;

	// their derivatives for velocity calculation
	double
		nu_dot, du_dot, dr_dot, di_dot;

	size_t i, s;
	double sqrt1e2, sinE, one_ecosE, cos2Phi, sin2Phi, cu, su, cOm, sOm, ci, si, R, dx, ut, cut, sut, vr, vu, v0[3], v[3];
	pony_gnss_sat *sat; // current satellite


	// validate
	if (gnss == NULL || allIODnav == NULL || allA == NULL || alln == NULL || allE == NULL || allaf == NULL)
		return;

	// requires gal and sat structure to be initialized
	if (gnss->gal == NULL || gnss->gal->sat == NULL)
		return;

	// run through all galileo satellites
	for (s = 0; s < gnss->gal->max_sat_count; s++)
	{
		sat = &(gnss->gal->sat[s]);
		if (!sat->eph_valid || sat->eph == NULL) // do nothing if no valid ephemeris available
			continue;

		// ephemeris 0 -- 5  - toc (see RINEX documentation)
		toc.Y	= pony_gnss_sat_round(sat->eph[0]);
		toc.M	= pony_gnss_sat_round(sat->eph[1]);
		toc.D	= pony_gnss_sat_round(sat->eph[2]);
		toc.h	= pony_gnss_sat_round(sat->eph[3]);
		toc.m	= pony_gnss_sat_round(sat->eph[4]);
		toc.s	=                     sat->eph[5] ;

		// check fit interval
		// t - toc, time elapsed since the time of clock
		// (!) intervals only less than 24 hours are supported
		t_toc = pony_gnss_sat_epoch_diff_within_day( &(gnss->epoch), &(toc) );
		if ( fabs( t_toc ) >= max_fit_interval ) {
				//sat->eph_valid = 0;
				continue;
		}

		// all other ephemeris (see RINEX documentation & Galileo interface specifications)
		af0			=                     sat->eph[ 6] ;
		af1			=                     sat->eph[ 7] ;
		af2			=                     sat->eph[ 8] ;
		IODnav		=                     sat->eph[ 9] ;
		Crs			=                     sat->eph[10] ;
		Deltan		=                     sat->eph[11] ;
		M0			=                     sat->eph[12] ;
		Cuc			=                     sat->eph[13] ;
		e			=                     sat->eph[14] ;
		Cus			=                     sat->eph[15] ;
		sqrtA		=                     sat->eph[16] ;
		toe			=                     sat->eph[17] ;
		Cic			=                     sat->eph[18] ;
		Om0			=                     sat->eph[19] ;
		Cis			=                     sat->eph[20] ;
		i0			=                     sat->eph[21] ;
		Crc			=                     sat->eph[22] ;
		om			=                     sat->eph[23] ;
		Omdot		=                     sat->eph[24] ;
		idot		=                     sat->eph[25] ;
		DS			= pony_gnss_sat_round(sat->eph[26]); // Data sources, skip this field, not analyzed yet
		week		= pony_gnss_sat_round(sat->eph[27]);
		// spare parameter field here              28
		SISA		=                     sat->eph[29] ; // Signal in space accuracy, skip this field, not analyzed yet
		SVh			= pony_gnss_sat_round(sat->eph[30]); // SV health
		BGDE5aE1	=                     sat->eph[31] ;
		BGDE5bE1	=                     sat->eph[32] ;
		tow			= pony_gnss_sat_round(sat->eph[33]);

		if (SVh) {
				// drop all flags and skip calculations if SV is not OK
				sat->eph_valid		= 0;
				sat->t_em_valid		= 0;
				sat->x_valid		= 0;
				sat->v_valid		= 0;
				sat->sinEl_valid	= 0;
				continue;
		}
		// recalculate stored quantities if new issue of data detected
		if ( pony_gnss_sat_round(IODnav) != pony_gnss_sat_round(allIODnav[s]) ) {
			allIODnav[s] = IODnav;
			// quantities that are constant between issues of ephemeris
			allA [s]	= sqrtA*sqrtA;
			n0			= sqrt(pony->gnss_const.gal.mu)/(sqrtA*sqrtA*sqrtA);
			alln [s]	= n0 + Deltan;
			allaf[s][0]	= af0;
			allaf[s][1]	= af1;
			allaf[s][2]	= af2;
		}

		// t_em - time of emission
		// (!) fit intervals only less than 12 hours are supported yet
			// toe - time of epoch treated as t_em base approximation
		sat->t_em = gnss->epoch.h*3600.0 + gnss->epoch.m*60.0 + gnss->epoch.s;
			// look for any valid code measurement to compensate for travel time
		ut = 0; // ECEF frame rotation angle
		for (i = 0; i < gnss->gal->obs_count; i++)
			if (gnss->gal->obs_types[i][0] == code_id && sat->obs_valid[i]) {
				ut = sat->obs[i]/pony->gnss_const.c; // base estimate for travel time
				if (sat->x_valid) // if position is already available, satellite clock correction applies
					ut += sat->Deltatsv;
				sat->t_em -= ut; // estimate for the time of emission in the system time frame
				sat->t_em_valid = 1;

				if (gnss->sol.dt_valid) // if receiver clock error estimated, receiver clock correction applies
					ut -= gnss->sol.dt;
				ut *= pony->gnss_const.gal.u; // ECEF frame rotation angle between emission and reception
				break;
			}


		// satellite position
		// (!) fit intervals only less than 12 hours are supported yet

			// tk - time from ephemeris reference epoch
		tk = sat->t_em - fmod(toe, pony->gnss_const.sec_in_d);
		if (tk < -pony->gnss_const.sec_in_d/2) // day rollover - forward
			tk += pony->gnss_const.sec_in_d;
		if (tk >  pony->gnss_const.sec_in_d/2) // day rollover - backward
			tk -= pony->gnss_const.sec_in_d;

			// M - mean anomaly at current time
		M = M0 + alln[s]*tk;

			// Kepler equation by iterations
		if (allE[s] < -10) // if no previous estimate available, take the mean motion
			allE[s] = M;
		i = 0;
		do {
			E = allE[s];
			allE[s] = M + e*sin(E);
			i++;
		} while (fabs(E - allE[s]) > anomaly_precision && i < max_iterations);

			// nu - true anomaly
		sqrt1e2 = sqrt(1-e*e);
		sinE = sin(E);
		nu = atan2( sqrt1e2*sinE, cos(E)-e );
			// Phi - argument of latitude at current time
		Phi = nu + om;
			// second harmonic perturbations
		cos2Phi = cos(2*Phi); sin2Phi = sin(2*Phi);
		du = Cus*sin2Phi + Cuc*cos2Phi; // du - argument of latitude correction
		dr = Crs*sin2Phi + Crc*cos2Phi; // dr - radius correction
		di = Cis*sin2Phi + Cic*cos2Phi; // di - inclination correction
			// corrected argument of latitude
		u = Phi + du;
			// corrected radius
		one_ecosE = 1 - e*cos(E);
		r = allA[s]*one_ecosE + dr;
			// corrected inclination
		I = i0 + di + idot*tk;
			// position in orbital plane
		cu = cos(u); su = sin(u);
		x_ = r*cu;
		y_ = r*su;
			// corrected longitude of ascending node
		Om = Om0 + (Omdot - pony->gnss_const.gal.u)*tk - pony->gnss_const.gal.u*toe; // gnss->gal_const.u stands for Omedot

			// Earth-fixed coordinates
		cOm = cos(Om); sOm = sin(Om);
		ci = cos(I); si = sin(I);
		x = x_*cOm - y_*ci*sOm;
		y = x_*sOm + y_*ci*cOm;
		z = y_*si;

		// radius to be checked
		R = x*x + y*y + z*z;

		// rotate the coordinate frame to the time of reception if t_em was corrected
		cut = cos(ut);
		sut = sin(ut);
		sat->x[0] =  x*cut + y*sut;
		sat->x[1] = -x*sut + y*cut;
		sat->x[2] =  z;
		if (radius_squared_min < R && R < radius_squared_max)
			sat->x_valid = 1;

		// SV clock correction as in Section 5.1.4 of Galileo OS SIS ICD, Issue 1.2 (November 2015), p. 47
		// (!) fit intervals only less than 12 hours are supported yet
			// t_toc = t - t0c is calculated above
			// relativistic correction
		Deltatr = pony->gnss_const.gal.F*e*sqrtA*sinE;
			// SV PRN code phase time offset
		sat->Deltatsv = allaf[s][0] + allaf[s][1]*t_toc + allaf[s][2]*t_toc*t_toc + Deltatr;

		// check if position is available to calculate elevation
		if (sat->x_valid && gnss->sol.x_valid) {
			for (i = 0, R = 0.0, sat->sinEl = 0; i < 3; i++) {
				dx = sat->x[i] - gnss->sol.x[i];
				sat->sinEl += gnss->sol.x[i]*dx;
				R += dx*dx;
			}
			R = sqrt(R);
			sat->sinEl /= R;

			for (i = 0, R = 0.0; i < 3; i++)
				R += gnss->sol.x[i]*gnss->sol.x[i];
			R = sqrt(R);
			sat->sinEl /= R;
			sat->sinEl_valid = 1;
		}
		else
			sat->sinEl_valid = 0;

		// velocity calculation
			// derivatives
		nu_dot = alln[s]*sqrt1e2/(one_ecosE*one_ecosE);
		du_dot = 2*(Cus*cos2Phi - Cuc*sin2Phi)*nu_dot;
		dr_dot = 2*(Crs*cos2Phi - Crc*sin2Phi)*nu_dot;
		di_dot = 2*(Cis*cos2Phi - Cic*sin2Phi)*nu_dot;
			// absolute velocity components in orbital plane
		vr = allA[s]*alln[s]*e*sinE/one_ecosE + dr_dot;
		vu = r*(nu_dot + du_dot);
		v0[0] =	vr*cu - vu*su;
		v0[1] = vr*su + vu*cu;
		v0[2] = r*sin(nu)*(idot + di_dot);
			// absolute velocity components in ECEF
		v[0] = v0[0]*cOm -	v0[1]*ci*sOm +	v0[2]*si*sOm;
		v[1] = v0[0]*sOm +	v0[1]*ci*cOm -	v0[2]*si*cOm;
		v[2] =				v0[1]*si +		v0[2]*ci;
			// speed to be checked
		R = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
			// relative to ECEF velocity components
		sat->v[0] = v[0] + (pony->gnss_const.gal.u - Omdot)*sat->x[1];
		sat->v[1] = v[1] - (pony->gnss_const.gal.u - Omdot)*sat->x[0];
		sat->v[2] = v[2];

		if (speed_squared_min < R && R < speed_squared_max)
			sat->v_valid = 1;
		else
			sat->v_valid = 0;
	}

}


	// single-receiver satellite calculations for BeiDou
void pony_gnss_sat_pos_vel_clock_bds_single_receiver(pony_gnss *gnss, double *allSOW, double *allA, double *alln, double *allE, double **alla)
{
	enum orbit_type                    {    meo,       igso,   orbits};
		// nominal radius                28.0e6      42.0e6    m
		// nominal speed                  3.8e3       3.1e3    m/s

	const double
		radius_squared_min[orbits]   = { 729e12,    1681e12  },
		//                            ~(27.0e6)^2 ~(41.0e6)^2  m^2
		radius_squared_max[orbits]   = { 841e12,    1849e12  },
		//                            ~(29.0e6)^2 ~(43.0e6)^2  m^2
		 speed_squared_min[orbits]   = {  12e6 ,       8e6   },
		//                            ~( 3.5e3)^2 ~( 2.8e3)^2 (m/s)^2
		 speed_squared_max[orbits]   = {  17e6 ,      12e6   },
		//                            ~( 4.1e3)^2 ~( 3.4e3)^2 (m/s)^2
		       inclination_threshold = 0.75,             // threshold to distinct between MEO/IGSO and GEO orbits, radians, roughly 43 deg
		radius_square_root_threshold = 5632,             // threshold to distinct between MEO and IGSO/GEO orbits, sqrt(m), roughly sqrt(32e6)
		max_fit_interval             = 6*3600,           // maximum fit interval, seconds
		anomaly_precision            = 1.0/0xe000000000; // threshold to stop Kepler's equation iterations,        radians, roughly 1e-12
	const int  max_iterations        = 32;               // threshold to stop Kepler's equation iterations
	const char code_id               = 'C';

	// ephemeris as in Sections 5.2.4.3-5.2.4.6, 5.2.4.8-5.2.4.10 of BeiDou SIS ICD OSS Version 2.1 (November 2016) p. 28,
	// (!) toc fit intervals only less than 24 hours supported yet
	// (!) toc expressed as pony_time_epoch: Year, Month, Day, hours, minutes, seconds
	int SOW, week, URAI, SatH1, AODC;
	pony_time_epoch toc;
	double TGD1, TGD2, a2, a1, a0;

	// clock correction parameters as in Section 5.2.4.10 of BeiDou SIS ICD OSS Version 2.1 (November 2016) p. 30
	double t_toc, Deltatr; // Deltatsv

	// ephemeris as in Sections 5.2.4.11-5.2.4.12 of BeiDou SIS ICD OSS Version 2.1 (November 2016) p. 31,
	// (!) Deltan, Omdot, idot - in rad/s, according to RINEX
	// (!) M0, Om0, i0, om - in radians, according to RINEX
	// (!) toe is seconds of Beidou week, a fractional part to go with week number, according to RINEX
	double AODE, Crs, Deltan, M0, Cuc, e, Cus, sqrtA, toe, Cic, Om0, Cis, i0, Crc, om, Omdot, idot;

	// elements of coordinate systems as in Table 5-11 of BeiDou SIS ICD OSS Version 2.1 (November 2016) p. 34, except for pre-calculated ones
	double // A,
		n0, // t,
		tk, // n,
		Mk, Ek, nuk, Phik, duk, drk, dik, uk, rk, ik, xk_, yk_, Omk, xk, yk, zk;

	// their derivatives for velocity calculation
	double
		nuk_dot, duk_dot, drk_dot, dik_dot;

	size_t i, s, orbit;
	double sqrt1e2, sinE, one_ecosE, cos2Phi, sin2Phi, cu, su, cOm, sOm, ci, si, R, dx, ut, cut, sut, vr, vu, v0[3], v[3];
	pony_gnss_sat *sat; // current satellite


	// validate
	if (gnss == NULL || allSOW == NULL || allA == NULL || alln == NULL || allE == NULL || alla == NULL)
		return;

	// requires bds and sat structure to be initialized
	if (gnss->bds == NULL || gnss->bds->sat == NULL)
		return;

	// run through all beidou satellites
	for (s = 0; s < gnss->bds->max_sat_count; s++)
	{
		sat = &(gnss->bds->sat[s]);
		if (!sat->eph_valid || sat->eph == NULL) // do nothing if no valid ephemeris available
			continue;

		// ephemeris 0 -- 5  - toc (see RINEX documentation)
		toc.Y	= pony_gnss_sat_round(sat->eph[0]);
		toc.M	= pony_gnss_sat_round(sat->eph[1]);
		toc.D	= pony_gnss_sat_round(sat->eph[2]);
		toc.h	= pony_gnss_sat_round(sat->eph[3]);
		toc.m	= pony_gnss_sat_round(sat->eph[4]);
		toc.s	=                     sat->eph[5] ;

		// check fit interval
		// t - toc, time elapsed since the time of clock
		// (!) intervals only less than 24 hours are supported
		t_toc = pony_gnss_sat_epoch_diff_within_day( &(gnss->epoch), &(toc) );
		if ( fabs( t_toc ) >= max_fit_interval ) {
				//sat->eph_valid = 0;
				continue;
		}

		// all other ephemeris (see RINEX documentation & BeiDou interface specifications)
		a0			=                     sat->eph[ 6] ;
		a1			=                     sat->eph[ 7] ;
		a2			=                     sat->eph[ 8] ;
		AODE		=                     sat->eph[ 9] ; // Age of Data, Ephemeris, not analyzed yet
		Crs			=                     sat->eph[10] ;
		Deltan		=                     sat->eph[11] ;
		M0			=                     sat->eph[12] ;
		Cuc			=                     sat->eph[13] ;
		e			=                     sat->eph[14] ;
		Cus			=                     sat->eph[15] ;
		sqrtA		=                     sat->eph[16] ;
		toe			=                     sat->eph[17] ;
		Cic			=                     sat->eph[18] ;
		Om0			=                     sat->eph[19] ;
		Cis			=                     sat->eph[20] ;
		i0			=                     sat->eph[21] ;
		Crc			=                     sat->eph[22] ;
		om			=                     sat->eph[23] ;
		Omdot		=                     sat->eph[24] ;
		idot		=                     sat->eph[25] ;
		// spare parameter field here		       26
		week		= pony_gnss_sat_round(sat->eph[27]);
		// spare parameter field here		       28
		URAI		= pony_gnss_sat_round(sat->eph[29]); // User Range Accuracy Index, not analyzed yet
		SatH1		= pony_gnss_sat_round(sat->eph[30]); // SV health
		TGD1		=                     sat->eph[31] ;
		TGD2		=                     sat->eph[32] ;
		SOW			= pony_gnss_sat_round(sat->eph[33]);
		AODC		= pony_gnss_sat_round(sat->eph[34]); // Age of Data, Clock, not analyzed yet

		// select orbit type
		orbit = orbits; // set invalid
		if (i0 > inclination_threshold && sqrtA < radius_square_root_threshold)
			orbit = meo;
		if (i0 > inclination_threshold && sqrtA > radius_square_root_threshold)
			orbit = igso;

		// check SV health and orbit type
		if (SatH1 || orbit >= orbits) {
				// drop all flags and skip calculations if SV is not OK, or the orbit is undefined
				sat->eph_valid		= 0;
				sat->t_em_valid		= 0;
				sat->x_valid		= 0;
				sat->v_valid		= 0;
				sat->sinEl_valid	= 0;
				continue;
		}
		// recalculate stored quantities if new issue of data detected
		if ( SOW != pony_gnss_sat_round(allSOW[s]) ) {
			allSOW[s] = (double)SOW;
			// quantities that are constant between issues of ephemeris
			allA[s]		= sqrtA*sqrtA;
			n0			= sqrt(pony->gnss_const.bds.mu)/(sqrtA*sqrtA*sqrtA);
			alln[s]		= n0 + Deltan;
			alla[s][0]	= a0;
			alla[s][1]	= a1;
			alla[s][2]	= a2;
		}

		// t_em - time of emission
		// (!) fit intervals only less than 12 hours are supported yet
			// toe - time of epoch treated as t_em base approximation
		sat->t_em = gnss->epoch.h*3600.0 + gnss->epoch.m*60.0 + gnss->epoch.s - pony->gnss_const.bds.leap_sec;
			// look for any valid code measurement to compensate for travel time
		ut = 0; // ECEF frame rotation angle
		for (i = 0; i < gnss->bds->obs_count; i++)
			if (gnss->bds->obs_types[i][0] == code_id && sat->obs_valid[i]) {
				ut = sat->obs[i]/pony->gnss_const.c; // base estimate for travel time
				if (sat->x_valid) // if position is already available, satellite clock correction applies
					ut += sat->Deltatsv;
				sat->t_em -= ut; // estimate for the time of emission in the system time frame
				sat->t_em_valid = 1;

				if (gnss->sol.dt_valid) // if receiver clock error estimated, receiver clock correction applies
					ut -= gnss->sol.dt;
				ut *= pony->gnss_const.bds.u; // ECEF frame rotation angle between emission and reception
				break;
			}


		// satellite position
		// (!) fit intervals only less than 12 hours are supported yet

			// tk - time from ephemeris reference epoch
		tk = sat->t_em - fmod(toe, pony->gnss_const.sec_in_d);
		if (tk < -pony->gnss_const.sec_in_d/2) // day rollover - forward
			tk += pony->gnss_const.sec_in_d;
		if (tk >  pony->gnss_const.sec_in_d/2) // day rollover - backward
			tk -= pony->gnss_const.sec_in_d;

			// Mk - mean anomaly at current time
		Mk = M0 + alln[s]*tk;

			// Kepler equation by iterations
		if (allE[s] < -10) // if no previous estimate available, take the mean motion
			allE[s] = Mk;
		i = 0;
		do {
			Ek = allE[s];
			allE[s] = Mk + e*sin(Ek);
			i++;
		} while (fabs(Ek - allE[s]) > anomaly_precision && i < max_iterations);

			// nuk - true anomaly
		sqrt1e2 = sqrt(1-e*e);
		sinE = sin(Ek);
		nuk = atan2( sqrt1e2*sinE, cos(Ek)-e );
			// Phik - argument of latitude at current time
		Phik = nuk + om;
			// second harmonic perturbations
		cos2Phi = cos(2*Phik); sin2Phi = sin(2*Phik);
		duk = Cus*sin2Phi + Cuc*cos2Phi; // duk - argument of latitude correction
		drk = Crs*sin2Phi + Crc*cos2Phi; // drk - radius correction
		dik = Cis*sin2Phi + Cic*cos2Phi; // dik - inclination correction
			// corrected argument of latitude
		uk = Phik + duk;
			// corrected radius
		one_ecosE = 1 - e*cos(Ek);
		rk = allA[s]*one_ecosE + drk;
			// corrected inclination
		ik = i0 + dik + idot*tk;
			// position in orbital plane
		cu = cos(uk); su = sin(uk);
		xk_ = rk*cu;
		yk_ = rk*su;
			// corrected longitude of ascending node
		Omk = Om0 + (Omdot - pony->gnss_const.bds.u)*tk - pony->gnss_const.bds.u*toe; // gnss->bds_const.u stands for Omedot

			// Earth-fixed coordinates
		cOm = cos(Omk); sOm = sin(Omk);
		ci = cos(ik); si = sin(ik);
		xk = xk_*cOm - yk_*ci*sOm;
		yk = xk_*sOm + yk_*ci*cOm;
		zk = yk_*si;

		// radius to be checked
		R = xk*xk + yk*yk + zk*zk;

		// rotate the coordinate frame to the time of reception if t_em was corrected
		cut = cos(ut);
		sut = sin(ut);
		sat->x[0] =  xk*cut + yk*sut;
		sat->x[1] = -xk*sut + yk*cut;
		sat->x[2] =  zk;
		if (radius_squared_min[orbit] < R && R < radius_squared_max[orbit])
			sat->x_valid = 1;

		// SV clock correction as in Section 5.2.4.10 of BeiDou SIS ICD OSS Version 2.1 (November 2016) p. 29
		// (!) fit intervals only less than 12 hours are supported yet
			// t_toc = t - t0c is calculated above
			// relativistic correction
		Deltatr = pony->gnss_const.bds.F*e*sqrtA*sinE;
			// SV PRN code phase time offset
		sat->Deltatsv = alla[s][0] + alla[s][1]*t_toc + alla[s][2]*t_toc*t_toc + Deltatr;

		// check if position is available to calculate elevation
		if (sat->x_valid && gnss->sol.x_valid) {
			for (i = 0, R = 0.0, sat->sinEl = 0; i < 3; i++) {
				dx = sat->x[i] - gnss->sol.x[i];
				sat->sinEl += gnss->sol.x[i]*dx;
				R += dx*dx;
			}
			R = sqrt(R);
			sat->sinEl /= R;

			for (i = 0, R = 0.0; i < 3; i++)
				R += gnss->sol.x[i]*gnss->sol.x[i];
			R = sqrt(R);
			sat->sinEl /= R;
			sat->sinEl_valid = 1;
		}
		else
			sat->sinEl_valid = 0;

		// velocity calculation
			// derivatives
		nuk_dot = alln[s]*sqrt1e2/(one_ecosE*one_ecosE);
		duk_dot = 2*(Cus*cos2Phi - Cuc*sin2Phi)*nuk_dot;
		drk_dot = 2*(Crs*cos2Phi - Crc*sin2Phi)*nuk_dot;
		dik_dot = 2*(Cis*cos2Phi - Cic*sin2Phi)*nuk_dot;
			// absolute velocity components in orbital plane
		vr = allA[s]*alln[s]*e*sinE/one_ecosE + drk_dot;
		vu = rk*(nuk_dot + duk_dot);
		v0[0] =	vr*cu - vu*su;
		v0[1] = vr*su + vu*cu;
		v0[2] = rk*sin(nuk)*(idot + dik_dot);
			// absolute velocity components in ECEF
		v[0] = v0[0]*cOm -	v0[1]*ci*sOm +	v0[2]*si*sOm;
		v[1] = v0[0]*sOm +	v0[1]*ci*cOm -	v0[2]*si*cOm;
		v[2] =				v0[1]*si +		v0[2]*ci;
			// speed to be checked
		R = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
			// relative to ECEF velocity components
		sat->v[0] = v[0] + (pony->gnss_const.bds.u - Omdot)*sat->x[1];
		sat->v[1] = v[1] - (pony->gnss_const.bds.u - Omdot)*sat->x[0];
		sat->v[2] = v[2];

		if (speed_squared_min[orbit] < R && R < speed_squared_max[orbit])
			sat->v_valid = 1;
		else
			sat->v_valid = 0;
	}

}




	// service routines
		// free memory
void pony_gnss_sat_free_ppointer_gnss_count(void ***ptr)
{
	size_t r;

	// validate
	if (ptr == NULL || *ptr == NULL)
		return;
	// free array contents
	for (r = 0; r < pony->gnss_count; r++)
		if ( (*ptr)[r] != NULL ) {
			free( (*ptr)[r] );
			(*ptr)[r] = NULL;
		}
	// free array
	free(*ptr);
	*ptr = NULL;
}

void pony_gnss_sat_free_pppointer_gnss_count_sat_count_gps(void ****ptr)
{
	size_t r, s;

	// validate
	if (ptr == NULL || *ptr == NULL)
		return;
	// free array contents
	if (pony->gnss != NULL)
		for (r = 0; r < pony->gnss_count; r++) {
			// requires gnss gps structure initialized
			if (pony->gnss[r].gps == NULL)
				continue;
			for (s = 0; s < pony->gnss[r].gps->max_sat_count; s++)
				if ( (*ptr)[r][s] != NULL ) {
					free( (*ptr)[r][s] );
					(*ptr)[r][s] = NULL;
				}
			free( (*ptr)[r] );
			(*ptr)[r] = NULL;
		}
	// free array
	free(*ptr);
	*ptr = NULL;
}

void pony_gnss_sat_free_pppointer_gnss_count_sat_count_glo(void ****ptr)
{
	size_t r, s;

	// validate
	if (ptr == NULL || *ptr == NULL)
		return;
	// free array contents
	if (pony->gnss != NULL)
		for (r = 0; r < pony->gnss_count; r++) {
			// requires gnss glo structure initialized
			if (pony->gnss[r].glo == NULL)
				continue;
			for (s = 0; s < pony->gnss[r].glo->max_sat_count; s++)
				if ( (*ptr)[r][s] != NULL ) {
					free( (*ptr)[r][s] );
					(*ptr)[r][s] = NULL;
				}
			free( (*ptr)[r] );
			(*ptr)[r] = NULL;
		}
	// free array
	free(*ptr);
	*ptr = NULL;
}

void pony_gnss_sat_free_pppointer_gnss_count_sat_count_gal(void ****ptr)
{
	size_t r, s;

	// validate
	if (ptr == NULL || *ptr == NULL)
		return;
	// free array contents
	if (pony->gnss != NULL)
		for (r = 0; r < pony->gnss_count; r++) {
			// requires gnss gal structure initialized
			if (pony->gnss[r].gal == NULL)
				continue;
			for (s = 0; s < pony->gnss[r].gal->max_sat_count; s++)
				if ( (*ptr)[r][s] != NULL ) {
					free( (*ptr)[r][s] );
					(*ptr)[r][s] = NULL;
				}
			free( (*ptr)[r] );
			(*ptr)[r] = NULL;
		}
	// free array
	free(*ptr);
	*ptr = NULL;
}

void pony_gnss_sat_free_pppointer_gnss_count_sat_count_bds(void ****ptr)
{
	size_t r, s;

	// validate
	if (ptr == NULL || *ptr == NULL)
		return;
	// free array contents
	if (pony->gnss != NULL)
		for (r = 0; r < pony->gnss_count; r++) {
			// requires gnss bds structure initialized
			if (pony->gnss[r].bds == NULL)
				continue;
			for (s = 0; s < pony->gnss[r].bds->max_sat_count; s++)
				if ( (*ptr)[r][s] != NULL ) {
					free( (*ptr)[r][s] );
					(*ptr)[r][s] = NULL;
				}
			free( (*ptr)[r] );
			(*ptr)[r] = NULL;
		}
	// free array
	free(*ptr);
	*ptr = NULL;
}



		// allocate memory
char pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gps(double ***ptr)
{
	size_t r;

	// validate
	if (ptr == NULL)
		return 0;
	// check pointer
	if (*ptr != NULL)
		return 1; // already allocated
	// requires gnss structure initialized
	if (pony->gnss == NULL) {
		*ptr = NULL;
		return 0;
	}
	// allocate array
	*ptr = (double **)calloc( pony->gnss_count, sizeof(double *) );
	if (*ptr == NULL)
		return 0;
	// allocate array contents
	for (r = 0; r < pony->gnss_count; r++) {
		// requires gps structure initialized
		if (pony->gnss[r].gps == NULL)
			continue;
		(*ptr)[r] = (double *)calloc( pony->gnss[r].gps->max_sat_count, sizeof(double) );
		if ( (*ptr)[r] == NULL )
			return 0;
	}

	return 1;
}

char pony_gnss_sat_alloc_pppointer_gnss_count_sat_count_gps(double ****ptr, const size_t internal_size)
{
	size_t i, r;

	// validate
	if (ptr == NULL)
		return 0;
	// check pointer
	if (*ptr != NULL)
		return 1; // already allocated
	// allocate array
	*ptr = (double ***)calloc( pony->gnss_count, sizeof(double **) );
	if (*ptr == NULL)
		return 0;
	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return 0;
	// allocate array contents
	for (r = 0; r < pony->gnss_count; r++) {
		// requires gps structure initialized
		if (pony->gnss[r].gps == NULL)
			continue;
		// try to allocate
		(*ptr)[r] = NULL;
		(*ptr)[r] = (double **)calloc( pony->gnss[r].gps->max_sat_count, sizeof(double *) );
		if ( (*ptr)[r] == NULL )
			return 0;

		for (i = 0; i < pony->gnss[r].gps->max_sat_count; i++) {
			(*ptr)[r][i] = NULL;
			(*ptr)[r][i] = (double *)calloc( internal_size, sizeof(double) );
			if ( (*ptr)[r][i] == NULL )
				return 0;
		}
	}

	return 1;
}

char pony_gnss_sat_alloc_dppointer_gnss_count_sat_count_glo(double ***ptr)
{
	size_t r;

	// validate
	if (ptr == NULL)
		return 0;
	// check pointer
	if (*ptr != NULL)
		return 1; // already allocated
	// requires gnss structure initialized
	if (pony->gnss == NULL) {
		*ptr = NULL;
		return 0;
	}
	// allocate array
	*ptr = (double **)calloc( pony->gnss_count, sizeof(double *) );
	if (*ptr == NULL)
		return 0;
	// allocate array contents
	for (r = 0; r < pony->gnss_count; r++) {
		// requires glo structure initialized
		if (pony->gnss[r].glo == NULL)
			continue;
		(*ptr)[r] = (double *)calloc( pony->gnss[r].glo->max_sat_count, sizeof(double) );
		if ( (*ptr)[r] == NULL )
			return 0;
	}

	return 1;
}

char pony_gnss_sat_alloc_ippointer_gnss_count_sat_count_glo(int ***ptr)
{
	size_t r;

	// validate
	if (ptr == NULL)
		return 0;
	// check pointer
	if (*ptr != NULL)
		return 1; // already allocated
	// requires gnss structure initialized
	if (pony->gnss == NULL) {
		*ptr = NULL;
		return 0;
	}
	// allocate array
	*ptr = (int **)calloc( pony->gnss_count, sizeof(int *) );
	if (*ptr == NULL)
		return 0;
	// allocate array contents
	for (r = 0; r < pony->gnss_count; r++) {
		// requires glo structure initialized
		if (pony->gnss[r].glo == NULL)
			continue;
		(*ptr)[r] = (int *)calloc( pony->gnss[r].glo->max_sat_count, sizeof(int) );
		if ( (*ptr)[r] == NULL )
			return 0;
	}

	return 1;
}

char pony_gnss_sat_alloc_dpppointer_gnss_count_sat_count_glo(double ****ptr, const size_t internal_size)
{
	size_t i, r;

	// validate
	if (ptr == NULL)
		return 0;
	// check pointer
	if (*ptr != NULL)
		return 1; // already allocated
	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return 0;
	// allocate array
	*ptr = (double ***)calloc( pony->gnss_count, sizeof(double **) );
	if (*ptr == NULL)
		return 0;
	// allocate array contents
	for (r = 0; r < pony->gnss_count; r++) {
		// requires glo structure initialized
		if (pony->gnss[r].glo == NULL)
			continue;
		// try to allocate
		(*ptr)[r] = NULL;
		(*ptr)[r] = (double **)calloc( pony->gnss[r].glo->max_sat_count, sizeof(double *) );
		if ( (*ptr)[r] == NULL )
			return 0;

		for (i = 0; i < pony->gnss[r].glo->max_sat_count; i++) {
			(*ptr)[r][i] = NULL;
			(*ptr)[r][i] = (double *)calloc( internal_size, sizeof(double) );
			if ( (*ptr)[r][i] == NULL )
				return 0;
		}
	}

	return 1;
}

char pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_gal(double ***ptr)
{
	size_t r;

	// validate
	if (ptr == NULL)
		return 0;
	// check pointer
	if (*ptr != NULL)
		return 1; // already allocated
	// requires gnss structure initialized
	if (pony->gnss == NULL) {
		*ptr = NULL;
		return 0;
	}
	// allocate array
	*ptr = (double **)calloc( pony->gnss_count, sizeof(double *) );
	if (*ptr == NULL)
		return 0;
	// allocate array contents
	for (r = 0; r < pony->gnss_count; r++) {
		// requires gal structure initialized
		if (pony->gnss[r].gal == NULL)
			continue;
		(*ptr)[r] = (double *)calloc( pony->gnss[r].gal->max_sat_count, sizeof(double) );
		if ( (*ptr)[r] == NULL )
			return 0;
	}

	return 1;
}

char pony_gnss_sat_alloc_pppointer_gnss_count_sat_count_gal(double ****ptr, const size_t internal_size)
{
	size_t i, r;

	// validate
	if (ptr == NULL)
		return 0;
	// check pointer
	if (*ptr != NULL)
		return 1; // already allocated
	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return 0;
	// allocate array
	*ptr = (double ***)calloc( pony->gnss_count, sizeof(double **) );
	if (*ptr == NULL)
		return 0;
	// allocate array contents
	for (r = 0; r < pony->gnss_count; r++) {
		// requires gal structure initialized
		if (pony->gnss[r].gal == NULL)
			continue;
		// try to allocate
		(*ptr)[r] = NULL;
		(*ptr)[r] = (double **)calloc( pony->gnss[r].gal->max_sat_count, sizeof(double *) );
		if ( (*ptr)[r] == NULL )
			return 0;

		for (i = 0; i < pony->gnss[r].gal->max_sat_count; i++) {
			(*ptr)[r][i] = NULL;
			(*ptr)[r][i] = (double *)calloc( internal_size, sizeof(double) );
			if ( (*ptr)[r][i] == NULL )
				return 0;
		}
	}

	return 1;
}

char pony_gnss_sat_alloc_ppointer_gnss_count_sat_count_bds(double ***ptr)
{
	size_t r;

	// validate
	if (ptr == NULL)
		return 0;
	// check pointer
	if (*ptr != NULL)
		return 1; // already allocated
	// requires gnss structure initialized
	if (pony->gnss == NULL) {
		*ptr = NULL;
		return 0;
	}
	// allocate array
	*ptr = (double **)calloc( pony->gnss_count, sizeof(double *) );
	if (*ptr == NULL)
		return 0;
	// allocate array contents
	for (r = 0; r < pony->gnss_count; r++) {
		// requires bds structure initialized
		if (pony->gnss[r].bds == NULL)
			continue;
		(*ptr)[r] = (double *)calloc( pony->gnss[r].bds->max_sat_count, sizeof(double) );
		if ( (*ptr)[r] == NULL )
			return 0;
	}

	return 1;
}

char pony_gnss_sat_alloc_pppointer_gnss_count_sat_count_bds(double ****ptr, const size_t internal_size)
{
	size_t i, r;

	// validate
	if (ptr == NULL)
		return 0;
	// check pointer
	if (*ptr != NULL)
		return 1; // already allocated
	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return 0;
	// allocate array
	*ptr = (double ***)calloc( pony->gnss_count, sizeof(double **) );
	if (*ptr == NULL)
		return 0;
	// allocate array contents
	for (r = 0; r < pony->gnss_count; r++) {
		// requires bds structure initialized
		if (pony->gnss[r].bds == NULL)
			continue;
		// try to allocate
		(*ptr)[r] = NULL;
		(*ptr)[r] = (double **)calloc( pony->gnss[r].bds->max_sat_count, sizeof(double *) );
		if ( (*ptr)[r] == NULL )
			return 0;

		for (i = 0; i < pony->gnss[r].bds->max_sat_count; i++) {
			(*ptr)[r][i] = NULL;
			(*ptr)[r][i] = (double *)calloc( internal_size, sizeof(double) );
			if ( (*ptr)[r][i] == NULL )
				return 0;
		}
	}

	return 1;
}




		// difference between two epochs not further than 1 day apart
double pony_gnss_sat_epoch_diff_within_day(pony_time_epoch *t1, pony_time_epoch *t2)
{
	double t1s, t2s;

	// validate
	if (t1 == NULL || t2 == NULL)
		return pony->gnss_const.sec_in_w; // invalid input
	// calculate seconds
	t1s = t1->h*3600.0 + t1->m*60 + t1->s;
	t2s = t2->h*3600.0 + t2->m*60 + t2->s;
	// calculate difference
	if (t1->D == t2->D)
		return t1s - t2s;
	else if (t1s < t2s) // t1 is assumed in the next day after t2
		return t1s - t2s + pony->gnss_const.sec_in_d;
	else				// t1 is assumed in the previous day before t2
		return t2s - t1s + pony->gnss_const.sec_in_d;
}

		// round to the nearest integer
int pony_gnss_sat_round(double x)
{
	return ( (int)( x >= 0.0 ? (x + 0.5) : (x - 0.5) ) );
}
