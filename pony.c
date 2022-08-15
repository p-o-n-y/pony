// Aug-2022
// PONY core source code

#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "pony.h"

// core functions to be used in host application
	// basic
char pony_add_plugin(void(*newplugin)(void));                                // add plugin to the plugin execution list,                             input: pointer to plugin function,                       output: OK/not OK (1/0)
char pony_init      (char*                 );                                // initialize the bus, except for core,                                 input: configuration string (see description),           output: OK/not OK (1/0)
char pony_step      (void                  );                                // step through the plugin execution list,                                                                                       output: OK/not OK (1/0)
char pony_terminate (void                  );                                // terminate operation,                                                                                                          output: OK/not OK (1/0)
	
	// advanced scheduling
char pony_remove_plugin    (void(*plugin   )(void)                        ); // remove all instances of the plugin from the plugin execution list,   input: pointer to plugin function,                       output: OK/not OK (1/0)
char pony_replace_plugin   (void(*oldplugin)(void), void(*newplugin)(void)); // replace all instances of the plugin by another one,                  input: pointers to old and new plugin functions,         output: OK/not OK (1/0)
char pony_schedule_plugin  (void(*newplugin)(void), int cycle, int shift  ); // add scheduled plugin to the plugin execution list,                   input: pointer to plugin function, cycle, shift,         output: OK/not OK (1/0)
char pony_reschedule_plugin(void(*plugin   )(void), int cycle, int shift  ); // reschedule all instances of the plugin in the plugin execution list, input: pointer to plugin function, new cycle, new shift, output: OK/not OK (1/0)
char pony_suspend_plugin   (void(*plugin   )(void)                        ); // suspend all instances of the plugin in the plugin execution list,    input: pointer to plugin function,                       output: OK/not OK (1/0)
char pony_resume_plugin    (void(*plugin   )(void)                        ); // resume all instances of the plugin in the plugin execution list,     input: pointer to plugin function,                       output: OK/not OK (1/0)





// bus instance
static pony_struct pony_bus = {
	PONY_BUS_VERSION,           // ver
	pony_add_plugin,            // add_plugin
	pony_init,                  // init
	pony_step,                  // step
	pony_terminate,             // terminate
	pony_remove_plugin,         // remove_plugin
	pony_replace_plugin,        // replace_plugin
	pony_schedule_plugin,       // schedule_plugin
	pony_reschedule_plugin,     // reschedule_plugin
	pony_suspend_plugin,        // suspend plugin
	pony_resume_plugin,         // resume plugin
	{ NULL, 0, 0, UINT_MAX, 0 } // core.plugins, core.plugin_count, core.exit_plugin_id, core.host_termination
};

pony_struct* pony = &pony_bus;





// service subroutines
	/*
		allocate memory with size validation and NULL-assignment
		input:
			void**       ptr            --- pointer to a pointer to the desired memory block
			const int    specified size --- specified array size to be validated
			size_t*      actual_size    --- pointer to store actual (validated and allocated) array size, provided non-NULL pointer
			const size_t element_size   --- size of each array element, e.g. sizeof(double)
			const size_t default_size   --- default array size if validation fails
			const size_t max_size       --- maximum allowed array size
		return value:
			1 if successful
			0 otherwise (ptr==NULL or memory allocation failed)
	 */
char pony_alloc_null(void** ptr, const int specified_size, size_t* actual_size, const size_t element_size, const size_t default_size, const size_t max_size)
{
	size_t size;
	char   res;

	// defaults
	size = 0;
	res  = 0;
	if (ptr != NULL) {
		// validate size
		if      (specified_size >  max_size)
			size = max_size;
		else if (specified_size >= 0       )
			size = specified_size;
		else
			size = default_size;
		// allocate memory
		if (size > 0) // if nonzero size requested
			*ptr = calloc(size, element_size);
		else           // if no elements required
			*ptr = NULL;
		// verify
		if (size == 0 || *ptr != NULL)
			res = 1;
	}
	// store size
	if (actual_size != NULL)
		*actual_size = size;

	return res;
}

	/*
		free memory with pointer NULL-check and NULL-assignment
		input:
			void** ptr --- pointer to a pointer to the desired memory block
	*/
void pony_free_null(void** ptr) 
{
	if (*ptr == NULL)
		return;
	free(*ptr);
	*ptr = NULL;
}

	/* 
		locate parameter group within a configuration string
		input:
			char*  groupname --- pointer to a group identifier (see documentation) or empty string to locate a substring that is outside of any group
			char*  cfgstr    --- pointer to a configuration string to parse
			size_t cfglen    --- number of characters in cfgstr to parse
		output:
			char** groupptr --- pointer to a pointer to the starting character of the group contents within a configuration string
			int*   grouplen --- pointer to a number of characters in the group contents
		return value:
			1 if the requested group found and grouplen > 0
			0 otherwise
		example:
			groupname = "gnss:"
			cfgstr    = "{gnss: {gps: eph_in="gpsa.nav", obs_in="gpsa.obs"}}, out="sol.txt""
			cfglen    = 66
	*/
char pony_locatecfggroup(const char* groupname, char* cfgstr, const size_t cfglen, char** groupptr, size_t* grouplen)
{
	size_t i, j;
	int group_layer;
	char group_found = 0;

	*groupptr = NULL;
	*grouplen = 0;

	if (cfgstr == NULL)
		return 0;

	// locate configuration substring that is outside of any group
	if (groupname[0] == '\0') {
		for (i = 0; i < cfglen && cfgstr[i]; i++) {
			// skip all non-printable characters, blank spaces and commas between groups
			for (; i < cfglen && cfgstr[i] && (cfgstr[i] <= ' ' || cfgstr[i] == ','); i++); 
			// if no group started at this point
			if (cfgstr[i] != '{')
				break;
			// if a group started
			else {
				group_layer = 1;
				while (i < cfglen && group_layer > 0 && cfgstr[i]) {
					i++;
					if (cfgstr[i] == '{')
						group_layer++;
					if (cfgstr[i] == '}')
						group_layer--;
				}
			}
		}

		// skip all non-printable characters, blank spaces and commas between groups
		for (; i < cfglen && cfgstr[i] && (cfgstr[i] <= ' ' || cfgstr[i] == ','); i++);
		// start from this point
		*groupptr = cfgstr + i;

		// determine the length, counting until the end of the string or when a group started or ended
		for (; (*groupptr)[*grouplen] && (*groupptr)[*grouplen] != '{' && (*groupptr)[*grouplen] != '}'; (*grouplen)++);
	}

	// locate configuration substring inside a requested group
	else {
		i = 0;
		while (i < cfglen && cfgstr[i] && !group_found) {
			// skip all non-printable characters, blank spaces and commas between groups
			for (; i < cfglen && cfgstr[i] && (cfgstr[i] <= ' ' || cfgstr[i] == ','); i++); 
			// if a group started
			if (cfgstr[i] == '{') {
				group_layer = 1;
				// skip all non-printable characters and blank spaces at the beginning of the group
				for (i++; i < cfglen && cfgstr[i] && (cfgstr[i] <= ' '); i++); 

				// check if the group is the one that has been requested
				group_found = 1;
				for (j = 0; i < cfglen && cfgstr[i] && groupname[j]; i++, j++) {
					if (cfgstr[i] != groupname[j]) {
						group_found = 0;
						break;
					}
				}
				// if did not reach the last character of groupname
				if (groupname[j])
					group_found = 0;

				if (group_found)
					// start from this point
					*groupptr = cfgstr + i;

				// go through the rest of the group
				while (i < cfglen && group_layer > 0 && cfgstr[i]) {
					if (cfgstr[i] == '{')
						group_layer++;
					if (cfgstr[i] == '}')
						group_layer--;
					i++;
					// count if inside the requested group, except for the last symbol
					if (group_found && group_layer > 0)
						(*grouplen)++;
				}
			} 
			// skip characters outside groups (common part for those groups)
			else
				i++;
		}
	}

	return (*groupptr == NULL) ? 0 : 1;
}





/*
	initialize epoch
	input:
		pony_time_epoch* epoch --- pointer to a time epoch structure
*/
void pony_init_epoch(pony_time_epoch* epoch)
{
	epoch->Y = 0;
	epoch->M = 0;
	epoch->D = 0;
	epoch->h = 0;
	epoch->m = 0;
	epoch->s = 0;
}





// solution data handling
	/*
		initialize navigation solution structure
		input:
			pony_sol*    sol      --- pointer to a navigation solution structure
			char*        settings --- pointer to a settings string
			const size_t len      --- length of the settings string
		return value:
			1 if successful
			0 otherwise
		example:
			sol      = &(pony->imu->sol)
			settings = pony->imu->cfg
			len      = pony->imu->cfglength
	*/
char pony_init_sol(pony_sol* sol, char* settings, const size_t len)
{
	const size_t 
		metrics_count_default = 2,   // default number of metrics in solution structures
		metrics_count_limit   = 255; // maximum number of metrics in solution structures

	const char metrics_count_token[] = "metrics_count"; // token to look for in settings

	size_t i;
	int    size;
	char*  cfgptr;

	// validate
	if (sol == NULL)
		return 0;
	// pos & vel
	for (i = 0; i < 3; i++) {
		sol->  x[i] = 0;
		sol->llh[i] = 0;
		sol->  v[i] = 0;
	}
	sol->  x_valid = 0;
	sol->llh_valid = 0;
	sol->  v_valid = 0;
	sol->  x_std = -1; // stdev undefined
	sol->  v_std = -1; // stdev undefined
	// attitude
	for (i = 0; i < 4; i++)
		sol->q[i]   = 0; // quaternion
	sol->q_valid    = 0;
	for (i = 0; i < 9; i++)
		sol->L[i]   = 0; // attitude matrix
	sol->L_valid    = 0;
	for (i = 0; i < 3; i++)
		sol->rpy[i] = 0; // attitude angles
	sol->rpy_valid  = 0;
	// clock
	sol->dt         = 0;
	sol->dt_valid   = 0;
	// metrics
	size = -1;
	cfgptr = pony_locate_token(metrics_count_token, settings, len, '='); // try to find number of metrics in settings string
	if (cfgptr != NULL)      // if token found
		size = atoi(cfgptr); // parse the number
	return pony_alloc_null((void**)(&(sol->metrics)), size, &(sol->metrics_count), sizeof(double), metrics_count_default, metrics_count_limit);
}

	/*
		free solution data memory
		input:
			pony_sol *sol ---  pointer to a navigation solution structure
	*/
void pony_free_sol(pony_sol* sol)
{
	// validate
	if (sol == NULL)
		return;

	pony_free_null((void**)(&(sol->metrics)));
}





// IMU data handling subroutines
	/*
		initialize inertial navigation constants
	*/
void pony_init_imu_const()
{
	pony->imu_const.pi      = 3.14159265358979323846264338327950288; // pi with maximum quad-precision floating point digits as in IEEE 754-2008 (binary128)
	pony->imu_const.pi2     = pony->imu_const.pi*2;                  // pi x 2
	pony->imu_const.pi_2    = pony->imu_const.pi/2;                  // pi / 2
	pony->imu_const.rad2deg = 180/pony->imu_const.pi;                // 180/pi
	
	// Earth parameters as in Section 4 of GRS-80 by H. Moritz, Journal of Geodesy (2000) 74 (1): pp. 128-162
	pony->imu_const.u       = 7.292115e-5;                           // Earth's rotation rate, rad/s
	pony->imu_const.a       = 6378137.0;                             // Earth's ellipsoid semi-major axis, m
	pony->imu_const.mu      = 3986005e8;                             // Earth's geocentric gravitational constant (including the atmosphere)
	pony->imu_const.J2      = 1.08263e-3;                            // Earth's dynamical form factor 
	pony->imu_const.e2      = 6.6943800229e-3;                       // Earth's ellipsoid first eccentricity squared
	pony->imu_const.ge      = 9.7803267715;                          // Earth's normal gravity at the equator, m/s^2
	pony->imu_const.fg      = 5.302440112e-3;                        // Earth's normal gravity flattening
}
	
	/*
		initialize IMU structure
		input:
			pony_imu*   imu      --- pointer to an IMU structure
		return value:
			1 if successful
			0 otherwise
	*/
char pony_init_imu(pony_imu* imu)
{
	const size_t
	temp_count_default = 3,   // default number of additional temperature sensors
	temp_count_limit   = 255; // maximum number of additional temperature sensors

	const char metrics_count_token[] = "temp_count"; // token to look for in settings

	size_t i;
	int    size;
	char*  cfgptr;


	// validate
	if (imu == NULL)
		return 0;
	// validity flags
	imu-> w_valid = 0;
	imu-> f_valid = 0;
	imu->Tw_valid = 0;
	imu->Tf_valid = 0;
	imu-> W_valid = 0;
	imu-> g_valid = 0;
	imu->t = 0;
	// zero parameters
	for (i = 0; i < 3; i++) {
		imu->w [i] = 0; // gyroscopes
		imu->f [i] = 0; // accelerometers
		imu->Tw[i] = 0; // gyroscope temperature
		imu->Tf[i] = 0; // accelerometer temperature
		imu->W [i] = 0; // angular velocity of the local level reference frame
	}
	// default gravity acceleration vector
	imu->g[0] = 0;
	imu->g[1] = 0;
	imu->g[2] = -pony->imu_const.ge*(1 + pony->imu_const.fg/2); // middle value
	// init solution data
	if (!pony_init_sol(&(imu->sol), imu->cfg, imu->cfglength))
		return 0;
	// init additional temperature sensors
	size = -1;
	cfgptr = pony_locate_token(metrics_count_token, imu->cfg, imu->cfglength, '='); // try to find number of additional temperature sensors in configuration string
	if (cfgptr != NULL)      // if token found
		size = atoi(cfgptr); // parse the number
	imu->T_valid = 0;
	return pony_alloc_null((void**)(&(imu->T)), size, &(imu->T_count), sizeof(double), temp_count_default, temp_count_limit);
}

	/*
		free IMU memory
	*/
void pony_free_imu(pony_imu* imu)
{
	// validate
	if (imu == NULL)
		return;
	// configuration
	imu->cfg = NULL;
	imu->cfglength = 0;
	// solution
	pony_free_sol(&(imu->sol));
	// application-specific additional temperature sensors
	pony_free_null((void**)(&(imu->T)));
}





// GNSS data handling subroutines
	/*
		initialize GNSS settings
		input:
			pony_gnss* gnss ---  pointer to a GNSS structure
		return value:
			1
	*/
void pony_init_gnss_settings(pony_gnss* gnss)
{
	size_t i;

	// locate settings within gnss group in configuration: outside all subgroups, either before or after all of them
	pony_locatecfggroup("", gnss->cfg, gnss->cfglength, &(gnss->cfg_settings), &(gnss->settings_length));
	// mask for the sine of GNSS satellite elevation angle
	gnss->settings.sinEl_mask     =   0;
	// default stdev values
	gnss->settings.   code_sigma  =  20;
	gnss->settings.  phase_sigma  =   0.01;
	gnss->settings.doppler_sigma  =   0.5;
	// antenna position
	for (i = 0; i < 3; i++)			 
		gnss->settings.ant_pos[i] =   0;
	gnss->settings.ant_pos_tol    =  -1;
}

	/*
		initialize GNSS leap seconds from the configuration string
		input:
			pony_gnss* gnss ---  pointer to a GNSS structure
	*/
void pony_init_gnss_leap_sec(pony_gnss* gnss)
{
	const char leap_sec_token[] = "leap_sec";

	char* cfg_ptr;

	if (gnss == NULL)
		return;

	// default values
	gnss->leap_sec       = 0;
	gnss->leap_sec_valid = 0;
	// check if the configuration is provided in GNSS data
	if (gnss->cfg_settings == NULL || gnss->settings_length == 0)
		return;
	// look for leap seconds in configuration
	cfg_ptr = pony_locate_token(leap_sec_token, gnss->cfg_settings, gnss->settings_length, '=');
	if (cfg_ptr != NULL)
		gnss->leap_sec = atoi(cfg_ptr);
	else
		return;
	// check leap seconds
	gnss->leap_sec_valid = (gnss->leap_sec >= 0) ? 1 : 0;
}

	/*
		initialize GNSS GPS constants
		input:
			pony_gps_const* gps_const --- pointer to a GPS system constants structure
	*/
void pony_init_gnss_gps_const(pony_gps_const* gps_const)
{
	gps_const->mu =  3.986005e14;                      // Earth gravity constant as in IS-GPS-200J (22 May 2018), m^3/s^2
	gps_const->u  =  7.2921151467e-5;                  // Earth rotation rate as in IS-GPS-200J (22 May 2018), rad/s
	gps_const->a  =  6378137.0;                        // Earth ellipsoid semi-major axis as in WGS-84(G1762) 2014-07-08, m
	gps_const->e2 =  6.694379990141e-3;                // Earth ellipsoid first eccentricity squared as in WGS-84(G1762) 2014-07-08
	gps_const->F  = -4.442807633e-10;                  // relativistic correction constant as in IS-GPS-200J (22 May 2018), s/sqrt(m)
	gps_const->F1 =  1575.42e6;                        // nominal frequency for L1 signal as in IS-GPS-200J (22 May 2018)
	gps_const->L1 =  pony->gnss_const.c/gps_const->F1; // nominal wavelength for L1 signal
	gps_const->F2 =  1227.60e6;                        // nominal frequency for L2 signal as in IS-GPS-200J (22 May 2018)
	gps_const->L2 =  pony->gnss_const.c/gps_const->F2; // nominal wavelength for L2 signal
}

	/*
		initialize satellite data
		input:
			pony_gnss_sat* sat --- pointer to a GNSS satellite data structure
	*/
void pony_init_gnss_sat(pony_gnss_sat* sat)
{
	sat->eph         = NULL;
	sat->eph_valid   = 0;
	sat->eph_counter = 0;
	sat->obs         = NULL;
	sat->obs_valid   = NULL;
	sat->x_valid     = 0;
	sat->v_valid     = 0;
	sat->t_em_valid  = 0;
	sat->sinEl_valid = 0;
}

	/*
		free satellite data
		input:
			pony_gnss_sat** sat       --- pointer to a GNSS satellite data structure
			const size_t    sat_count --- number of sattelites
	*/
void pony_free_gnss_sat(pony_gnss_sat** sat, const size_t sat_count)
{
	size_t s;

	if (*sat == NULL) 
		return;

	for (s = 0; s < sat_count; s++) {
		// ephemeris
		pony_free_null((void**)(&((*sat)[s].eph)));
		// observations
		pony_free_null((void**)(&((*sat)[s].obs)));
		pony_free_null((void**)(&((*sat)[s].obs_valid)));
	}
	
	free((void*)(*sat));
	*sat = NULL;
}

	/*
		initialize GNSS GPS structure
		input:
			pony_gnss_gps* gps           --- pointer to a GPS constellation data structure
			const size_t   max_sat_count --- maximum number of sattelites
			const size_t   max_eph_count --- maximum number of ephemeris per satellite
		return value:
			1 if successful
			0 otherwise
	*/
char pony_init_gnss_gps(pony_gnss_gps* gps, const size_t max_sat_count, const size_t max_eph_count)
{
	size_t i;

	gps->sat = NULL;
	gps->max_sat_count = 0;
	gps->max_eph_count = 0;

	// try to allocate memory for satellite data
	gps->sat = (pony_gnss_sat*)calloc(max_sat_count, sizeof(pony_gnss_sat));
	if (gps->sat == NULL)
		return 0;
	gps->max_sat_count = max_sat_count;

	// initialize satellite data
	for (i = 0; i < gps->max_sat_count; i++) 
		pony_init_gnss_sat(gps->sat + i);

	// try to allocate memory for each satellite ephemeris
	for (i = 0; i < gps->max_sat_count; i++) {
		gps->sat[i].eph = (double*)calloc(max_eph_count, sizeof(double));
		if (gps->sat[i].eph == NULL)
			return 0;
	}
	gps->max_eph_count = max_eph_count;

	// observation types
	gps->obs_types = NULL;
	gps->obs_count = 0;

	// validity flags
	gps->iono_valid = 0;
	gps->clock_corr_valid = 0;

	return 1;
}

	/*
		free GNSS GPS memory
		input:
			pony_gnss_gps* gps --- pointer to a GPS constellation data structure
	*/
void pony_free_gnss_gps(pony_gnss_gps* gps)
{
	if (gps == NULL)
		return;
	
	//configuration
	gps->cfg = NULL;
	gps->cfglength = 0;
	
	// satellites
	pony_free_gnss_sat(&(gps->sat), gps->max_sat_count);
	gps->max_sat_count = 0;
	
	// observation types
	pony_free_null((void**)(&(gps->obs_types)));
	gps->obs_count = 0;
	
	// gnss_gps structure
	free((void*)gps);
}

	/*
		initialize GNSS GLONASS constants
		input:
			pony_glo_const* glo_const --- pointer to a GLONASS system constants structure
	*/
void pony_init_gnss_glo_const(pony_glo_const* glo_const)
{
	glo_const->mu  = 398600.4418e9; // Earth gravity constant as in PZ-90.11 (2014), m^3/s^2
	glo_const->J02 = 1082.62575e-6; // second degree zonal harmonic coefficient of normal potential as in PZ-90.11 (2014)
	glo_const->u   = 7.292115e-5;   // Earth rotation rate as in PZ-90.11 (2014), rad/s
	glo_const->a   = 6378136.0;     // Earth ellipsoid semi-major axis as in PZ-90.11 (2014), m
	glo_const->e2  = 0.0066943662;  // Earth ellipsoid first eccentricity squared as in PZ-90.11 (2014)
	glo_const->F01 = 1602e6;        // nominal centre frequency for L1 signal as in ICD GLONASS Edition 5.1 2008, Hz
	glo_const->dF1 = 562.5e3;       // nominal channel separation for L1 signal as in ICD GLONASS Edition 5.1 2008, Hz
	glo_const->F02 = 1246e6;        // nominal centre frequency for L2 signal as in ICD GLONASS Edition 5.1 2008, Hz
	glo_const->dF2 = 437.5e3;       // nominal channel separation for L2 signal as in ICD GLONASS Edition 5.1 2008, Hz
}

	/*
		initialize GNSS GLONASS structure
		input:
			pony_gnss_glo* glo           --- pointer to a GLONASS constellation data structure
			const size_t   max_sat_count --- maximum number of sattelites
			const size_t   max_eph_count --- maximum number of ephemeris per satellite
		return value:
			1 if successful
			0 otherwise
	*/
char pony_init_gnss_glo(pony_gnss_glo* glo, const size_t max_sat_count, const size_t max_eph_count)
{
	size_t i;

	glo->sat = NULL;
	glo->freq_slot = NULL;
	glo->max_sat_count = 0;
	glo->max_eph_count = 0;

	// try to allocate memory for satellite data
	glo->sat = (pony_gnss_sat*)calloc(max_sat_count, sizeof(pony_gnss_sat));
	if (glo->sat == NULL)
		return 0;
	glo->freq_slot = (int*)calloc(max_sat_count, sizeof(int));
	glo->max_sat_count = max_sat_count;

	// initialize satellite data
	for (i = 0; i < glo->max_sat_count; i++)
		pony_init_gnss_sat(glo->sat + i);

	// try to allocate memory for each satellite ephemeris
	for (i = 0; i < glo->max_sat_count; i++) {
		glo->sat[i].eph = (double*)calloc(max_eph_count, sizeof(double));
		if (glo->sat[i].eph == NULL)
			return 0;
	}
	glo->max_eph_count = max_eph_count;

	// observation types
	glo->obs_types = NULL;
	glo->obs_count = 0;

	return 1;
}

	/*
		free GNSS GLONASS memory
		input:
			pony_gnss_glo* glo --- pointer to a GLONASS constellation data structure
	*/
void pony_free_gnss_glo(pony_gnss_glo* glo)
{
	if (glo == NULL)
		return;
	
	//configuration
	glo->cfg = NULL;
	glo->cfglength = 0;
	
	// satellites
	pony_free_gnss_sat(&(glo->sat), glo->max_sat_count);
	glo->max_sat_count = 0;
	
	// observation types
	pony_free_null((void**)(&(glo->obs_types)));
	glo->obs_count = 0;
	
	// frequency slots
	pony_free_null((void**)(&(glo->freq_slot)));
	
	// gnss glonass structure
	free((void*)glo);
}

	/*
		initialize GNSS Galileo constants
		input:
			pony_gal_const* gal_const --- pointer to a Galileo system constants structure
	*/	
void pony_init_gnss_gal_const(pony_gal_const* gal_const)
{
	gal_const->mu  =  3.986004418e14;                    // Earth gravity constant as in Galileo OS SIS ICD Issue 1.2 (November 2015), m^3/s^2
	gal_const->u   =  7.2921151467e-5;                   // Earth rotation rate as in Galileo OS SIS ICD Issue 1.2 (November 2015), rad/s
	gal_const->a   =  6378137.0;                         // Earth ellipsoid semi-major axis as in GRS-80 // JoG March 2000 vol. 74 issue 1, m
	gal_const->e2  =  6.69438002290e-3;                  // Earth ellipsoid first eccentricity squared as in GRS-80 // JoG March 2000 vol. 74 issue 1
	gal_const->F   = -4.442807309e-10;                   // relativistic correction constant as in Galileo OS SIS ICD Issue 1.2 (November 2015), s/sqrt(m)
	gal_const->F1  =  1575.42e6;                         // nominal frequency for E1 signal as in Galileo OS SIS ICD Issue 1.2 (November 2015)
	gal_const->L1  =  pony->gnss_const.c/gal_const->F1;  // nominal wavelength for E1 signal
	gal_const->F5a =  1176.45e6;                         // nominal frequency for E5a signal as in Galileo OS SIS ICD Issue 1.2 (November 2015)
	gal_const->L5a =  pony->gnss_const.c/gal_const->F5a; // nominal wavelength for E5a signal
	gal_const->F5b =  1207.14e6;                         // nominal frequency for E5b signal as in Galileo OS SIS ICD Issue 1.2 (November 2015)
	gal_const->L5b =  pony->gnss_const.c/gal_const->F5b; // nominal wavelength for E5b signal
	gal_const->F6  =  1278.75e6;                         // nominal frequency for E6 signal as in Galileo OS SIS ICD Issue 1.2 (November 2015)
	gal_const->L6  =  pony->gnss_const.c/gal_const->F6;  // nominal wavelength for E6 signal
}

	/*
		initialize GNSS Galileo structure
		input:
			pony_gnss_gal* gal           --- pointer to a Galileo constellation data structure
			const size_t   max_sat_count --- maximum number of sattelites
			const size_t   max_eph_count --- maximum number of ephemeris per satellite

		return value:
			1 if successful
			0 otherwise
	*/
char pony_init_gnss_gal(pony_gnss_gal* gal, const size_t max_sat_count, const size_t max_eph_count)
{
	size_t i;

	gal->sat = NULL;
	gal->max_sat_count = 0;
	gal->max_eph_count = 0;

	// try to allocate memory for satellite data
	gal->sat = (pony_gnss_sat*)calloc(max_sat_count, sizeof(pony_gnss_sat));
	if (gal->sat == NULL)
		return 0;
	gal->max_sat_count = max_sat_count;

	// initialize satellite data
	for (i = 0; i < gal->max_sat_count; i++) 
		pony_init_gnss_sat(gal->sat + i);

	// try to allocate memory for each satellite ephemeris
	for (i = 0; i < gal->max_sat_count; i++) {
		gal->sat[i].eph = (double*)calloc(max_eph_count, sizeof(double));
		if (gal->sat[i].eph == NULL)
			return 0;
	}
	gal->max_eph_count = max_eph_count;

	// observation types
	gal->obs_types = NULL;
	gal->obs_count = 0;

	// validity flags
	gal->iono_valid = 0;
	gal->clock_corr_valid = 0;

	return 1;
}

	/*
		free GNSS Galileo memory
		input:
			pony_gnss_gal* gal --- pointer to a Galileo constellation data structure
	*/
void pony_free_gnss_gal(pony_gnss_gal* gal)
{
	if (gal == NULL)
		return;

	//configuration
	gal->cfg = NULL;
	gal->cfglength = 0;
	
	// satellites
	pony_free_gnss_sat(&(gal->sat), gal->max_sat_count);
	gal->max_sat_count = 0;
	
	// observation types
	pony_free_null((void**)(&(gal->obs_types)));
	gal->obs_count = 0;
	
	// gnss galileo structure
	free((void*)gal);
}

	/*
		initialize GNSS BeiDou constants
		input:
			pony_bds_const* bds_const --- pointer to a BeiDou system constants structure
	*/
void pony_init_gnss_bds_const(pony_bds_const* bds_const)
{
	bds_const->mu       =  3.986004418e14;                   // Earth gravity constant as in BeiDou SIS ICD OSS Version 2.1 (November 2016), m^3/s^2
	bds_const->u        =  7.292115e-5;                      // Earth rotation rate as in BeiDou SIS ICD OSS Version 2.1 (November 2016), rad/s
	bds_const->a        =  6378137.0;                        // Earth ellipsoid semi-major axis as in BeiDou SIS ICD OSS Version 2.1 (November 2016), m
	bds_const->e2       =  6.6943800229008e-3;               // Earth ellipsoid first eccentricity squared derived from flattening as in BeiDou SIS ICD OSS Version 2.1 (November 2016)
	bds_const->F        = -4.442807309043977e-10;            // relativistic correction constant derived from Earth gravity as in BeiDou SIS ICD OSS Version 2.1 (November 2016), s/sqrt(m)
	bds_const->leap_sec =  14;                               // leap seconds between BeiDou time and GPS time as of 01-Jan-2006
	bds_const->B1       =  1561.098e6;                       // nominal frequency for B1 signal as in BeiDou SIS ICD OSS Version 2.1 (November 2016)
	bds_const->L1       =  pony->gnss_const.c/bds_const->B1; // nominal wavelength for B1 signal
	bds_const->B2       =  1207.140e6;                       // nominal frequency for B2 signal as in BeiDou SIS ICD OSS Version 2.1 (November 2016)
	bds_const->L2       =  pony->gnss_const.c/bds_const->B2; // nominal wavelength for B2 signal
}

	/*
		initialize GNSS BeiDou structure
		input:
			pony_gnss_bds* bds           --- pointer to a BeiDou constellation data
			const size_t   max_sat_count --- maximum number of sattelites
			const size_t   max_eph_count --- maximum number of ephemeris per satellite
		return value:
			1 if successful
			0 otherwise
	*/
char pony_init_gnss_bds(pony_gnss_bds* bds, const size_t max_sat_count, const size_t max_eph_count)
{
	size_t i;

	bds->sat = NULL;
	bds->max_sat_count = 0;
	bds->max_eph_count = 0;

	// try to allocate memory for satellite data
	bds->sat = (pony_gnss_sat*)calloc(max_sat_count, sizeof(pony_gnss_sat));
	if (bds->sat == NULL)
		return 0;
	bds->max_sat_count = max_sat_count;

	// initialize satellite data
	for (i = 0; i < bds->max_sat_count; i++) 
		pony_init_gnss_sat(bds->sat + i);

	// try to allocate memory for each satellite ephemeris
	for (i = 0; i < bds->max_sat_count; i++) {
		bds->sat[i].eph = (double*)calloc(max_eph_count, sizeof(double));
		if (bds->sat[i].eph == NULL)
			return 0;
	}
	bds->max_eph_count = max_eph_count;

	// observation types
	bds->obs_types = NULL;
	bds->obs_count = 0;

	// validity flags
	bds->iono_valid = 0;
	bds->clock_corr_valid = 0;

	return 1;
}

	/*
		free GNSS BeiDou memory
		input:
			pony_gnss_bds* bds --- pointer to a BeiDou constellation data structure
	*/
void pony_free_gnss_bds(pony_gnss_bds* bds)
{
	if (bds == NULL)
		return;
	
	//configuration
	bds->cfg = NULL;
	bds->cfglength = 0;
	
	// satellites
	pony_free_gnss_sat(&(bds->sat), bds->max_sat_count);
	bds->max_sat_count = 0;
	
	// observation types
	pony_free_null((void**)(&(bds->obs_types)));
	bds->obs_count = 0;
	
	// gnss beidou structure
	free((void*)bds);
}

	/*
		initialize GNSS constants
	*/
void pony_init_gnss_const()
{
	pony->gnss_const.pi       = 3.1415926535898; // pi, circumference to diameter ratio, as in as in IS-GPS-200J, Galileo OS SIS ICD Issue 1.2 (November 2015), BeiDou SIS ICD OSS Version 2.1 (November 2016)
	pony->gnss_const.c        = 299792458;       // speed of light as in IS-GPS-200J (22 May 2018), ICD GLONASS Edition 5.1 2008, Galileo OS SIS ICD Issue 1.2 (November 2015), BeiDou SIS ICD OSS Version 2.1 (November 2016), m/s
	pony->gnss_const.sec_in_w = 604800;          // seconds in a week
	pony->gnss_const.sec_in_d = 86400;           // seconds in a day

	// constellation-specific constants
	pony_init_gnss_gps_const(&(pony->gnss_const.gps));
	pony_init_gnss_glo_const(&(pony->gnss_const.glo));
	pony_init_gnss_gal_const(&(pony->gnss_const.gal));
	pony_init_gnss_bds_const(&(pony->gnss_const.bds));
}

	/*
		initialize GNSS structure
		input:
			pony_gnss* gnss --- pointer to a GNSS data structure
		return value:
			1 if successful
			0 otherwise
	*/
char pony_init_gnss(pony_gnss* gnss)
{
	// memory allocation limits
	enum system_id                   {gps, glo, gal, bds};

	const size_t   max_sat_count[] = {36,  36,  36,  64 },
	               max_eph_count[] = {36,  24,  36,  36 };

	size_t grouplen;
	char*  groupptr;

	if (gnss == NULL)
		return 0;

	// gps
	gnss->gps = NULL;
	if (pony_locatecfggroup("gps:", gnss->cfg, gnss->cfglength, &groupptr, &grouplen)) {
		gnss->gps = (pony_gnss_gps*)calloc(1, sizeof(pony_gnss_gps));
		
		if (gnss->gps == NULL)
			return 0;
		
		gnss->gps->cfg       = groupptr;
		gnss->gps->cfglength = grouplen;

		if (!pony_init_gnss_gps(gnss->gps, max_sat_count[gps], max_eph_count[gps]))
			return 0;
	}

	// glonass
	gnss->glo = NULL;
	if (pony_locatecfggroup( "glo:", gnss->cfg, gnss->cfglength, &groupptr, &grouplen)) {
		gnss->glo = (pony_gnss_glo*)calloc(1, sizeof(pony_gnss_glo));
		
		if (gnss->glo == NULL)
			return 0;
		
		gnss->glo->cfg       = groupptr;
		gnss->glo->cfglength = grouplen;

		if (!pony_init_gnss_glo(gnss->glo, max_sat_count[glo], max_eph_count[glo]))
			return 0;
	}

	// galileo
	gnss->gal = NULL;
	if (pony_locatecfggroup("gal:", gnss->cfg, gnss->cfglength, &groupptr, &grouplen)) {
		gnss->gal = (pony_gnss_gal*)calloc(1, sizeof(pony_gnss_gal));
		
		if (gnss->gal == NULL)
			return 0;
		
		gnss->gal->cfg       = groupptr;
		gnss->gal->cfglength = grouplen;

		if (!pony_init_gnss_gal(gnss->gal, max_sat_count[gal], max_eph_count[gal]))
			return 0;
	}

	// beidou
	gnss->bds = NULL;
	if (pony_locatecfggroup("bds:", gnss->cfg, gnss->cfglength, &groupptr, &grouplen)) {
		gnss->bds = (pony_gnss_bds*)calloc(1, sizeof(pony_gnss_bds));
		
		if (gnss->bds == NULL)
			return 0;
		
		gnss->bds->cfg       = groupptr;
		gnss->bds->cfglength = grouplen;

		if (!pony_init_gnss_bds(gnss->bds, max_sat_count[bds], max_eph_count[bds]))
			return 0;
	}

	// gnss settings
	pony_init_gnss_settings(gnss);

	// gnss solution
	if (!pony_init_sol(&(gnss->sol), gnss->cfg_settings, gnss->settings_length))
		return 0;

	// gnss epoch
	pony_init_epoch(&(gnss->epoch));

	// leap seconds
	pony_init_gnss_leap_sec(gnss);

	return 1;
}

	/*
		free GNSS memory
		input:
			pony_gnss* gnss --- pointer to a GNSS data structure
	*/
void pony_free_gnss(pony_gnss* gnss)
{
	if (gnss == NULL)
		return;

	// configuration
	gnss->cfg = NULL;
	gnss->cfglength = 0;
	gnss->cfg_settings = NULL;
	gnss->settings_length = 0;
	
	// gps data
	pony_free_gnss_gps(gnss->gps);
	gnss->gps = NULL;
	
	// glonass data
	pony_free_gnss_glo(gnss->glo);
	gnss->glo = NULL;
	
	// galileo data
	pony_free_gnss_gal(gnss->gal);
	gnss->gal = NULL;
	
	// beidou data
	pony_free_gnss_bds(gnss->bds);
	gnss->bds = NULL;

	pony_free_sol(&(gnss->sol));
}





// air data handling subroutines
	/*
		initialize air data structure
		input:
			pony_air* air --- pointer to an air data structure
		return value:
			1
	*/
char pony_init_air(pony_air* air)
{
	// values
	air->t     = 0;
	air->alt   = 0;
	air->vv    = 0;
	air->speed = 0;
	
	// stdev 
	air->alt_std   = 30.0; // barometric altitude stdev
	air->vv_std    =  0.5; // barometric altitude trend stdev
	air->speed_std =  1.5; // airspeed measurement stdev
	
	// validity flags
	air->alt_valid   = 0;
	air->vv_valid    = 0;
	air->speed_valid = 0;

	return 1;
}

	/*
		free air data memory
	*/
void pony_free_air(void)
{
	if (pony->air == NULL)
		return;
	
	// configuration
	pony->air->cfg = NULL;
	pony->air->cfglength = 0;
	
	// air data
	free((void*)(pony->air));
}





// reference data handling subroutines
	/*
		initialize reference data structure
		input:
			pony_ref* ref --- pointer to a reference data structure
		return value:
			1 if successful
			0 otherwise
	*/
char pony_init_ref(pony_ref* ref)
{
	// time
	ref->t = 0;
	// default gravity acceleration vector
	ref->g[0] = 0;
	ref->g[1] = 0;
	ref->g[2] = -pony->imu_const.ge*(1 + pony->imu_const.fg/2); // middle value
	ref->g_valid = 0;
	// drop the solution
	if (!pony_init_sol(&(ref->sol), ref->cfg, ref->cfglength))
		return 0;

	return 1;
}

	/*
		free reference data memory
	*/
void pony_free_ref(void)
{
	if (pony->ref == NULL)
		return;

	// configuration
	pony->ref->cfg = NULL;
	pony->ref->cfglength = 0;

	// solution
	pony_free_sol(&(pony->ref->sol));
	
	// pony_ref structure
	free((void*)(pony->ref));
	pony->ref = NULL;
}





// general handling routines
	/*
		free all alocated memory and set pointers and counters to NULL
	*/
void pony_free()
{
	size_t i;

	// core
	pony_free_null((void**)(&(pony->core.plugins)));
	pony->core.plugin_count = 0;

	// configuration string
	pony_free_null((void**)(&(pony->cfg)));
	pony->cfglength = 0;
	pony->cfg_settings = NULL;
	pony->settings_length = 0;

	// imu
	if (pony->imu != NULL) {
		for (i = 0; i < pony->imu_count; i++)
			pony_free_imu(&(pony->imu[i]));
		free((void*)(pony->imu));
		pony->imu = NULL;
	}
	pony->imu_count = 0;

	// gnss
	if (pony->gnss != NULL) {
		for (i = 0; i < pony->gnss_count; i++)
			pony_free_gnss(&(pony->gnss[i]));
		free((void*)(pony->gnss));
		pony->gnss = NULL;
	}
	pony->gnss_count = 0;

	// air data
	pony_free_air();

	// reference data
	pony_free_ref();

	// solution
	pony_free_sol(&(pony->sol));
}










//                        CORE PONY FUNCTIONS FOR HOST APPLICATION

// basic functions
	/* 
		add plugin to the plugin execution list	
		input:
			newplugin --- pointer to plugin function (no arguments, no return value)
		return value: 
			1 if successful
			0 otherwise (failed to allocate/realocate memory, plugin limit reached)
	*/
char pony_add_plugin(void(*newplugin)(void))
{
	pony_plugin* reallocated_pointer;

	if (pony->core.plugin_count + 1 >= (size_t)UINT_MAX)
		return 0;

	reallocated_pointer = (pony_plugin*)realloc((void*)(pony->core.plugins), (pony->core.plugin_count + 1)*sizeof(pony_plugin));

	if (reallocated_pointer == NULL) { // failed to allocate/realocate memory
		pony_free();
		return 0;
	} 
	else
		pony->core.plugins = reallocated_pointer;

	pony->core.plugins[pony->core.plugin_count].func  = newplugin;
	pony->core.plugins[pony->core.plugin_count].cycle = 1;
	pony->core.plugins[pony->core.plugin_count].shift = 0;
	pony->core.plugins[pony->core.plugin_count].tick  = 0;
	pony->core.plugin_count++;

	return 1;
}

	/*
		initialize the bus, except for core
		input: 
			char* cfg --- pointer to a configuration string (see pony description)
		return value: 
			1 if successful
			0 otherwise (memory allocation or partial init failed)
	*/
char pony_init(char* cfg)
{
	const size_t 
		max_imu_count  = 10, // maximum number of imu data structures
		max_gnss_count = 10; // maximum number of gnss receiver data structures

	char 
		multi_imu_token [] = "imu[0]:",
		multi_gnss_token[] = "gnss[0]:";
	const size_t 
		multi_imu_index_position = 4,
		multi_gnss_index_position = 5;

	size_t grouplen;
	char* cfgptr;

	void* reallocated_pointer;

	size_t i;

	// determine configuration string length
	for (pony->cfglength = 0; cfg[pony->cfglength]; pony->cfglength++);

	// assign configuration string
	pony->cfg = (char*)malloc(sizeof(char)*(pony->cfglength+1));
	if (pony->cfg == NULL)
		return 0;
	for (i = 0; i < pony->cfglength; i++)
		pony->cfg[i] = cfg[i];
	pony->cfg[pony->cfglength] = '\0';

	// fetch a part of configuration that is outside of any group
	pony->cfg_settings = NULL;
	pony->settings_length = 0;
	pony_locatecfggroup("", pony->cfg, pony->cfglength, &pony->cfg_settings, &pony->settings_length);

	// imu init
	pony_init_imu_const();
	pony->imu = NULL;
	pony->imu_count = 0;
	// multiple imu support
	for (i = 0; i < max_imu_count; i++) {
		multi_imu_token[multi_imu_index_position] = '0' + (char)i;

		if ((i == 0 && pony_locatecfggroup("imu:",          pony->cfg, pony->cfglength, &cfgptr, &grouplen)) ||
			           pony_locatecfggroup(multi_imu_token, pony->cfg, pony->cfglength, &cfgptr, &grouplen)) {
			if (i >= pony->imu_count) {
				// try to allocate/reallocate memory
				reallocated_pointer = realloc(pony->imu, sizeof(pony_imu)*(i+1));
				if (reallocated_pointer == NULL) {
					pony_free();
					return 0;
				}
				else
					pony->imu = (pony_imu*)reallocated_pointer;
				// try to init new instances, except the i-th one
				for (; pony->imu_count < i; pony->imu_count++) {
					pony->imu[pony->imu_count].cfg = NULL;
					pony->imu[pony->imu_count].cfglength = 0;
					pony_init_imu(&(pony->imu[pony->imu_count]));
				}
				pony->imu_count = i+1;
			}

			// set configuration pointer
			pony->imu[i].cfg       = cfgptr;
			pony->imu[i].cfglength = grouplen;

			// try to init
			if (!pony_init_imu(&(pony->imu[i]))) {
				pony_free();
				return 0;
			}
		}
	}

	// gnss init
	pony_init_gnss_const();
	pony->gnss = NULL;
	pony->gnss_count = 0;
	// multiple gnss support
	for (i = 0; i < max_gnss_count; i++) {
		multi_gnss_token[multi_gnss_index_position] = '0' + (char)i;

		if ((i == 0 && pony_locatecfggroup("gnss:",          pony->cfg, pony->cfglength, &cfgptr, &grouplen)) ||
		               pony_locatecfggroup(multi_gnss_token, pony->cfg, pony->cfglength, &cfgptr, &grouplen)) {
			if (i >= pony->gnss_count) {
				// try to allocate/reallocate memory
				reallocated_pointer = realloc(pony->gnss, sizeof(pony_gnss)*(i+1));
				if (reallocated_pointer == NULL) {
					pony_free();
					return 0;
				}
				else
					pony->gnss = (pony_gnss*)reallocated_pointer;
				// try to init new instances, except the i-th one
				for (; pony->gnss_count < i; pony->gnss_count++) {
					pony->gnss[pony->gnss_count].cfg = NULL;
					pony->gnss[pony->gnss_count].cfglength = 0;
					pony_init_gnss(&(pony->gnss[pony->gnss_count]));
				}
				pony->gnss_count = i+1;
			}

			// set configuration pointer
			pony->gnss[i].cfg       = cfgptr;
			pony->gnss[i].cfglength = grouplen;

			// try to init
			if (!pony_init_gnss(&(pony->gnss[i]))) {
				pony_free();
				return 0;
			}
		}
	}

	// air data init
	pony->air = NULL;
	if (pony_locatecfggroup("air:", pony->cfg, pony->cfglength, &cfgptr, &grouplen)) { // if the group found in configuration
		// try to allocate memory
		pony->air = (pony_air*)calloc(1, sizeof(pony_air));
		if (pony->air == NULL) {
			pony_free();
			return 0;
		}

		// set configuration pointer
		pony->air->cfg = cfgptr;
		pony->air->cfglength = grouplen;

		// try to init
		if (!pony_init_air(pony->air)) {
			pony_free();
			return 0;
		}
	}

	// reference data init
	pony->ref = NULL;
	if (pony_locatecfggroup("ref:", pony->cfg, pony->cfglength, &cfgptr, &grouplen)) { // if the group found in configuration
		// try lo allocate memory
		pony->ref = (pony_ref*)calloc(1, sizeof(pony_ref));
		if (pony->ref == NULL) {
			pony_free();
			return 0;
		}

		// set configuration pointer
		pony->ref->cfg = cfgptr;
		pony->ref->cfglength = grouplen;

		// try to init
		if(!pony_init_ref(pony->ref)) {
			pony_free();
			return 0;
		}
	}

	// solution
	if (!pony_init_sol(&(pony->sol), pony->cfg_settings, pony->settings_length)) {
		pony_free();
		return 0;
	}

	// system time, operation mode
	pony->t = 0;
	pony->mode = 0;
	return 1;
}

	/*
		step through the plugin execution list, to be called by host application in a main loop
		return value:
			1 if successful (either staying in regular operation mode, or a termination is properly detected)
			0 otherwise
	*/
char pony_step(void)
{
	long int prev_mode; // 0 = init, <0 terminate, >0 normal operation, for unwanted side-effects protection
	size_t i;

	// loop through plugin execution list
	for (pony->core.current_plugin_id = 0; pony->core.current_plugin_id < pony->core.plugin_count; pony->core.current_plugin_id++) {
		i = pony->core.current_plugin_id;

		prev_mode = pony->mode;

		if (pony->mode <= 0                                                                                    // init/termination mode (every plugin)
			|| (pony->core.plugins[i].cycle > 0 && pony->core.plugins[i].tick == pony->core.plugins[i].shift)) // or the scheduled tick has come
			pony->core.plugins[i].func();                                                                      // execute the current plugin

		// unwanted side-effects protection
		if (0
			|| (prev_mode == 0 && pony->mode >  0 && i+1 != pony->core.plugin_count) // premature end of initialization
			|| (prev_mode <  0 && pony->mode >= 0)                                   // premature end of termination
			)
			pony->mode = prev_mode;                                                  // disable mode change

		pony->core.plugins[i].tick++;                                  // current tick increment
		if (pony->core.plugins[i].tick >= pony->core.plugins[i].cycle) // check to stay within the cycle
			pony->core.plugins[i].tick = 0;                            // reset tick

		if (pony->core.exit_plugin_id == i) { // if termination was initiated by the current plugin on the previous loop
			pony->core.exit_plugin_id = (size_t)UINT_MAX; // set to default
			pony->core.host_termination = 0;              // set to default
			pony_free();                                  // free memory
			break;
		}

		if ((pony->mode < 0 || pony->core.host_termination == 1) && pony->core.exit_plugin_id == (size_t)UINT_MAX) { // if termination was initiated by the current plugin on the current loop
			if (pony->mode > 0)
				pony->mode = -1;               // set mode to -1 for external termination cases

			if (pony->mode != 0)
				pony->core.exit_plugin_id = i; // set the index to use in the next loop
		}
	}

	if (pony->mode == 0) // if initialization ended
		pony->mode = 1;  // set operation mode to regular operation

	// success if either staying in regular operation mode, or a termination properly detected
	return (pony->mode >= 0) || (pony->core.exit_plugin_id < (size_t)UINT_MAX);
}

	/*
		terminate operation
		return value:
			1 if termination is due in the next step
			0 if termination had been already terminated by host or by plugin
	*/
char pony_terminate(void)
{
	if (pony->mode >= 0 && pony->core.host_termination != 1) {
		pony->core.host_termination = 1;

		return 1; // termination is due in the next step
	}

	return 0; // had been already terminated by host or by plugin 
}





// advanced scheduling functions
	/*
		remove all instances of a given plugin from the plugin execution list	
		input:
			plugin --- pointer to a plugin function to remove from execution list
		return value:
			number of plugin instances found, limited to 127
			0 if no instances found, or memory reallocation somehow failed
	*/
char pony_remove_plugin(void(*plugin)(void))
{
	size_t       i, j;
	char         flag = 0;
	pony_plugin* reallocated_pointer;

	for (i = 0; i < pony->core.plugin_count; i++) { // go through the execution list
		if (pony->core.plugins[i].func != plugin)   // if not the requested plugin, do nothing
			continue;
		// otherwise, remove the current plugin from the execution list
		for (j = i+1; j < pony->core.plugin_count; j++) // copy all succeeding plugins one position lower
			pony->core.plugins[j-1] = pony->core.plugins[j];
		// reset the last one
		j--;
		pony->core.plugins[j].func  = NULL;
		pony->core.plugins[j].cycle = 0;
		pony->core.plugins[j].shift = 0;
		pony->core.plugins[j].tick  = 0;
		pony->core.plugin_count--;
		if (flag < 0x7f)
			flag++;
	}

	// reallocate memory
	reallocated_pointer = (pony_plugin*)realloc((void*)(pony->core.plugins), pony->core.plugin_count*sizeof(pony_plugin));
	if (pony->core.plugin_count > 0 && reallocated_pointer == NULL) { // memory reallocation somehow failed
		pony_free();
		return 0;
	}
	pony->core.plugins = reallocated_pointer;

	return flag;
}

	/*
		replace all instances of the given plugin to another one in the plugin execution list
		input:
			oldplugin --- pointer to a plugin function to be replaced
			newplugin --- pointer to a plugin function to replace with
		return value:
			1 if successful
			0 otherwise (no instances found)
	*/	
char pony_replace_plugin(void(*oldplugin)(void), void(*newplugin)(void))
{
	size_t i;
	char flag;

	for (i = 0, flag = 0; i < pony->core.plugin_count; i++) {
		if (pony->core.plugins[i].func != oldplugin)
			continue;
		pony->core.plugins[i].func = newplugin;
		flag = 1;
	}

	return flag;
}

	/*
		add scheduled plugin to the plugin execution list
		input:
			newplugin --- pointer to a plugin function
			cycle     --- repeating cycle (in ticks of main cycle),
			              negative for suspended plugin,
			              zero for the plugin to be turned off (for further rescheduling)
			shift     --- shift from the beginning of the cycle, automatically shrunk to [0..cycle-1]
		return value:
			1 if successful
			0 otherwise (failed to allocate/realocate memory)
	*/
char pony_schedule_plugin(void(*newplugin)(void), int cycle, int shift)
{
	int abs_cycle;

	// add to the execution list
	if (!pony_add_plugin(newplugin))
		return 0;
	// shrink shift to [0..cycle-1]
	abs_cycle = abs(cycle);
	if (abs_cycle) {
		while (shift >= abs_cycle)
			shift -= abs_cycle;
		while (shift < 0)
			shift += abs_cycle;
	}
	else
		shift = 0;
	// set scheduling parameters
	pony->core.plugins[pony->core.plugin_count-1].cycle = cycle;
	pony->core.plugins[pony->core.plugin_count-1].shift = shift;
	pony->core.plugins[pony->core.plugin_count-1].tick  = 0;

	return 1;
}

	/*
		reschedule all instances of the plugin in the plugin execution list
		input:
			plugin --- pointer to a plugin function
			cycle  --- new repeating cycle (in ticks of main cycle),
			           negative for suspended plugin,
			           zero for the plugin to be turned off (for further rescheduling)
			shift  --- new shift from the beginning of the cycle, automatically shrunk to [0..cycle-1]
		return value:
			1 if successful
			0 otherwise (plugin not found in the execution list)
	*/
char pony_reschedule_plugin(void(*plugin)(void), int cycle, int shift)
{
	size_t i;
	int abs_cycle;
	char flag = 0;
	// shrink shift to [0..cycle-1]
	abs_cycle = abs(cycle);
	if (abs_cycle) {
		while (shift >= abs_cycle)
			shift -= abs_cycle;
		while (shift < 0)
			shift += abs_cycle;
	}
	else
		shift = 0;
	// go through execution list and set scheduling parameters, if found the plugin
	for (i = 0; i < pony->core.plugin_count; i++) {
		if (pony->core.plugins[i].func == plugin) {
			pony->core.plugins[i].cycle = cycle;
			pony->core.plugins[i].shift = shift;
			pony->core.plugins[i].tick  = 0;
			flag = 1;
		}
	}
	return flag;
}

	/*
		suspend all instances of the plugin in the plugin execution list
		input:
			plugin --- pointer to a plugin function
		return value:
			1 if successful
			0 otherwise (plugin not found)
	*/
char pony_suspend_plugin(void(*plugin)(void))
{
	size_t i;
	int cycle;
	char flag = 0;
	// go through execution list and set cycle to negative, if found the plugin
	for (i = 0; i < pony->core.plugin_count; i++) {
		if (pony->core.plugins[i].func == plugin) {
			cycle = pony->core.plugins[i].cycle;
			if (cycle > 0)
				pony->core.plugins[i].cycle = -cycle;
			flag = 1;
		}
	}
	return flag;
}

	/*
		resume all instances of the plugin in the plugin execution list	
		input:
			plugin --- pointer to a plugin function
		return value:
			1 if successful
			0 otherwise (plugin not found)
	*/
char pony_resume_plugin(void(*plugin)(void))
{
	size_t i;
	int cycle;
	char flag = 0;
	// go through execution list and set cycle to positive, if found the plugin
	for (i = 0; i < pony->core.plugin_count; i++) {
		if (pony->core.plugins[i].func == plugin) {
			cycle = pony->core.plugins[i].cycle;
			if (cycle < 0)
				pony->core.plugins[i].cycle = -cycle;
			flag = 1;
		}
	}
	return flag;
}





// basic parsing	
	/*
		locate a token (and delimiter, when given) within a configuration string
		skips groups enclosed in braces {...} and strings within quotes ("...")
		input:
			const char*  token --- pointer to a token to be found and located, only non-blank characters
			char*        src   --- pointer to a source string to search and locate in
			const size_t len   --- maximum length to search and locate within
			const char   delim --- either zero to be ignored, or non-zero mandatory delimiter to look next to the token
		return value:
			NULL if the token or delimiter (if delim != 0) not found, or token is invalid
			pointer to the next character after the token or delimiter (if delim != 0)
	*/
char* pony_locate_token(const char* token, char* src, const size_t len, const char delim)
{
	const char quote = '"', brace_open = '{', brace_close = '}', blank = ' ';

	size_t i, j, k, n, len1;

	if (token == NULL || src == NULL) // nothing to do
		return NULL;

	for (n = 0; token[n]; n++); // determine token length
	if (n == 0) // invalid token
		return NULL;
	if (len < n) // string too short
		return NULL;
	len1 = len - n;

	// look for the token
	for (i = 0, j = 0, k = 0; i <= len1 && src[i]; i++) // go throughout the string
		if (src[i] == quote) // skip quoted values
			for (i++; i < len1 && src[i] && src[i] != quote; i++);
		else if (src[i] == brace_open) // skip groups
			for (i++; i < len1 && src[i] && src[i] != brace_close; i++);
		else {
			for (j = 0, k = i; j < n && src[k] == token[j]; j++, k++);
			if (j == n) // token found
				break;
		}
	if (j < n) // token not found
		return NULL;

	if (!delim)
		return (src + k);

	// check for delimiter
	for (i = k; i < len && src[i] && src[i] <= blank; i++); // skip all non-printables		
	if (i >= len || src[i] != delim) // no delimiter found
		return NULL;
	else
		return (src + i + 1);
}





// time routines
	/*
		compare time epochs
		input:
			pony_time_epoch* date1 --- pointer to the first epoch structure
			pony_time_epoch* date2 --- pointer to the second epoch structure
		return value:
			+1 if date1 is later than date2
			 0 if equal (within 1/32768 sec, half-precision compliant)
			-1 otherwise
	*/
int pony_time_epochs_compare(pony_time_epoch* date1, pony_time_epoch* date2)
{
	const double time_precision = 1./(0x01<<15); // 1/32768, guaranteed non-zero in half precision

	int ix;
	double dx;

	ix = date1->Y - date2->Y; if (ix) return (ix > 0) ? 1 : -1;
	ix = date1->M - date2->M; if (ix) return (ix > 0) ? 1 : -1;
	ix = date1->D - date2->D; if (ix) return (ix > 0) ? 1 : -1;
	ix = date1->h - date2->h; if (ix) return (ix > 0) ? 1 : -1;
	ix = date1->m - date2->m; if (ix) return (ix > 0) ? 1 : -1;

	dx = date1->s - date2->s; return (dx > time_precision) ? 1 : ((dx < -time_precision) ? -1 : 0);
}

	/*
		calculate days elapsed from one date to another, based on Rata Die serial date from day one on 0001/01/01
		input:
			pony_time_epoch epoch_from --- starting epoch, only Y, M and D are used
			pony_time_epoch epoch_to   --- ending epoch, only Y, M and D are used
		return value:
			number of days elapsed from starting epoch to the ending one
	*/
long pony_time_days_between_dates(pony_time_epoch epoch_from, pony_time_epoch epoch_to)
{
	enum gregorian_calendar_constants {
		leap_yr_short_cycle =   4, // years
		leap_yr_med_cycle   = 100, // years
		leap_yr_long_cycle  = 400, // years
		feb      =  2,             // month
		yr_len_m = 12,			   // months
		days_of_yr_approx_1_num = 153, // days per month x 5 \.
		days_of_yr_approx_0_num = 457, // days x 5           |-- these parameters approximate days passed since March, 1st, for the current year: floor([153*m-457]/5), m = 3,4,..,14
		days_of_yr_approx_denom =   5, // denominator        /.
		yr_len_d        = 365};    // days

	// move February to the end of year, so that all leap days occur at the end
	if (epoch_to  .M <= feb)
		epoch_to  .Y--, epoch_to  .M += yr_len_m;
	if (epoch_from.M <= feb)
		epoch_from.Y--, epoch_from.M += yr_len_m;
	// calculate difference between two Rata Die day numbers
	return
		(epoch_to  .Y - epoch_from.Y)*yr_len_d +                                                                                                                                                           // full years
		(epoch_to  .Y/leap_yr_short_cycle - epoch_to  .Y/leap_yr_med_cycle + epoch_to  .Y/leap_yr_long_cycle + (days_of_yr_approx_1_num*epoch_to  .M - days_of_yr_approx_0_num)/days_of_yr_approx_denom) - // leap days 
		(epoch_from.Y/leap_yr_short_cycle - epoch_from.Y/leap_yr_med_cycle + epoch_from.Y/leap_yr_long_cycle + (days_of_yr_approx_1_num*epoch_from.M - days_of_yr_approx_0_num)/days_of_yr_approx_denom) + // and current year
		(epoch_to.D - epoch_from.D);                                                                                                                                                                       // remaining days
	// original Rata Die calculation:
	// (365*epoch_to.  Y + epoch_to.  Y/4 - epoch_to  .Y/100 + epoch_to  .Y/400 + (153*epoch_to  .M - 457)/5 + epoch_to  .D - 306) - 
	// (365*epoch_from.Y + epoch_from.Y/4 - epoch_from.Y/100 + epoch_from.Y/400 + (153*epoch_from.M - 457)/5 + epoch_from.D - 306);
}

	/*
		convert GPS week and seconds to GPS Gregorian date/time (DOES NOT include leap seconds)
		input:
			pony_time_epoch* epoch --- pointer to a epoch structure
			unsigned int     week  --- GPS week number
			double           sec   --- GPS seconds into the week, positive only accepted
		return value:
			1 if successful
			0 otherwise (invalid input)
		note:
			checked explicitly from Jan 6, 1980 to Dec 31, 5741 (1M+ days)
	*/
char pony_time_gps2epoch(pony_time_epoch* epoch, unsigned int week, double sec)
{
	//                            Mar  Apr  May  Jun  Jul  Aug  Sep  Oct  Nov  Dec  Jan
	const unsigned short dom[] = { 30,  60,  91, 121, 152, 183, 213, 244, 274, 305, 336}; // days of year since March 1 when months end
	unsigned long days, d1, d100, l100, d400, l400, dd;

	if (sec < 0)
		return 0; // invalid input

	days = (unsigned int)(sec /86400);  // days into week
	sec -= (double)      (days*86400);  // seconds into day

	days += week*7 + 138737;             // move starting day from January 6, 1980 to March 1, 1600 (beginning of 400-year leap cycle)
	d1 = days + 1;
	l400 = d1/146097;                    // extra days due to 400-year cycles (146 097 days each)
	d400 = days - l400;                  // days adjusted for 400-year cycles
	l100 = d400/36524;                   // missing days due to 100-year cycles (36 524 days each)
	d100 = d1   + l100 - l400;           // days adjusted for 100- and 400-year cycles
	dd = days - d100/1461 + l100 - l400; // days adjusted for all leap years, incl. 4-year cycles (1461 days each)
	// year
	epoch->Y = 1600 + dd/365;
	// day of year (starting from zero on March 1)
	epoch->D = dd%365;
	if (d1%146097 == 0 || (d100%1461 == 0 && d400%36524 != 0)) // add leap day
		epoch->D++;
	// month
	epoch->M = epoch->D/31; // exact for 340 days out of 366, less by one for the rest 26
	if (epoch->M < 11 && epoch->D > dom[epoch->M]) // if underestimated, adjust by one
		epoch->M++;
	// day of year (from zero) to day of month (from one)
	epoch->D += (epoch->M > 0) ? -dom[epoch->M-1] : 1;
	// move starting day from March 1 to January 1
	if (epoch->M > 9) {
		epoch->M -= 9;
		epoch->Y++;
	}
	else
		epoch->M += 3;
	// time of day
	epoch->h = (unsigned int)sec/3600;
	sec     -= epoch->h*3600.0;
	epoch->m = (unsigned int)sec/60;
	epoch->s = sec - epoch->m*60;

	return 1;
}

	/*
		convert GPS Gregorian date/time to GPS week and seconds (DOES NOT include leap seconds)
		input:
			unsigned int*    week  --- pointer to a GPS week number (NULL to skip)
			double*          sec   --- pointer to a GPS seconds into the week (NULL to skip)
			pony_time_epoch* epoch --- pointer to a epoch structure
		return value:
			1 if successful
			0 otherwise (invalid input)
	*/
char pony_time_epoch2gps(unsigned int* week, double* sec, pony_time_epoch* epoch)
{
	const pony_time_epoch base = {1980,1,6,0,0,0.0};

	long int days;

	if (epoch == NULL || (week == NULL && sec == NULL) || epoch->M <= 0 || epoch->D <= 0)
		return 0; // invalid input

	days = pony_time_days_between_dates(base, *epoch);
	if (days < 0) // the date happens to be before GPS time has started
		return 0;
	if (week != NULL)
		*week = days/7;
	days %= 7;
	if (sec != NULL)
		*sec = days*86400.0 + epoch->h*3600.0 + epoch->m*60 + epoch->s;

	return 1;
}





// linear algebra functions
	// conventional operations
		/*
			calculate dot product
			input:
				const double* u --- pointer to the first vector
				const double* v --- pointer to the second vector
				const size_t  n --- dimension
			return value:
				dot product u^T*v
		*/	
double pony_linal_dot(const double* u, const double* v, const size_t n)
{
	double res;
	size_t i;

	for (i = 1, res = u[0]*v[0]; i < n; i++)
		res += u[i]*v[i];

	return res;
}

		/*
			calculate l_2 vector norm
			input:
				const double* u --- pointer to a vector
				const size_t  n --- dimension
			return value:
				l_2 vector norm, i.e. sqrt(u^T*u)
		*/
double pony_linal_vnorm(const double* u, const size_t n)
{
	return sqrt(pony_linal_dot(u, u, n));
}

		/*
			calculate cross product for 3x1 vectors
			input:
				const double* u --- pointer to the first 3x1 vector
				const double* v --- pointer to the second 3x1 vector

			output:
				double* res     --- pointer to a cross product
		*/
void pony_linal_cross3x1(double* res, const double* u, const double* v)
{
	res[0] = u[1]*v[2] - u[2]*v[1];
	res[1] = u[2]*v[0] - u[0]*v[2];
	res[2] = u[0]*v[1] - u[1]*v[0];
}

		/*
			multiply two matrices 
			input:
				const double* a        --- pointer to the first n x n1 matrix 
				const double* b        --- pointer to the second n1 x m matrix
				const size_t  n, n1, m --- dimensions
			output:
				double* res            --- pointer to a n x m matrix, 
				                           res = a*b
		*/
void pony_linal_mmul(double* res, const double* a, const double* b, const size_t n, const size_t n1, const size_t m)
{
	size_t i, j, k, k0, ka, kb, p;

	for (i = 0, k = 0, k0 = 0; i < n; i++, k0 += n1) {
		for (j = 0; j < m; j++, k++) {
			ka = k0;
			kb = j;
			res[k] = a[ka]*b[kb];
			for (ka++, kb += m, p = 1; p < n1; ka++, kb += m, p++)
				res[k] += a[ka]*b[kb];
		}
	}
}

		/*
			multiply two matrices (first matrix is transposed)
			input:
				const double* a        --- pointer to the first n x m matrix 
				const double* b        --- pointer to the second n x n1 matrix 
				const size_t  n, n1, m --- dimensions
			output:
				double* res            --- pointer to a m x n1 matrix,
				                           res = a^T*b
		*/
void pony_linal_mmul1T(double* res, const double* a, const double* b, const size_t n, const size_t m, const size_t n1)
{
	size_t i, j, k, ka, kb, p;

	for (i = 0, k = 0; i < m; i++) {
		for (j = 0; j < n1; j++, k++) {
			ka = i;
			kb = j;
			res[k] = a[ka]*b[kb];
			for (ka += m, kb += n1, p = 1; p < n; ka += m, kb += n1, p++)
				res[k] += a[ka]*b[kb];
		}
	}
}

		/*
			multiply two matrices (second matrix is transposed)
			input:
				const double* a       --- pointer to the first n x m matrix 
				const double* b       --- pointer to the second n1 x m matrix 
				const size_t n, n1, m --- dimensions
			output:
				double* res           --- pointer to a n x n1 matrix,
				                          res = a*b^T
		*/
void pony_linal_mmul2T(double* res, const double* a, const double* b, const size_t n, const size_t m, const size_t n1)
{
	size_t i, j, k, k0, ka, kb, p;

	for (i = 0, k = 0, k0 = 0; i < n; i++, k0 += m) {
		for (j = 0; j < n1; j++, k++) {
			ka = k0;
			kb = j*m;
			res[k] = a[ka]*b[kb];
			for (ka++, kb++, p = 1; p < m; ka++, kb++, p++)
				res[k] += a[ka]*b[kb];
		}
	}
}

		/*
			multiply 4x1 quaternions
			input:
				const double* q --- pointer to the first  4x1 quaternion
				const double* r --- pointer to the second 4x1 quaternion
			output:
				double* res     --- pointer to a 4x1 quaternion, 
				                res = q x r, with res0, q0, r0 being scalar parts
		*/
void pony_linal_qmul(double* res, const double* q, const double* r)
{
	res[0] = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3];
	res[1] = q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2];
	res[2] = q[0]*r[2] + q[2]*r[0] + q[3]*r[1] - q[1]*r[3];
	res[3] = q[0]*r[3] + q[3]*r[0] + q[1]*r[2] - q[2]*r[1];
}
	
	// space rotation representation
		/*
			calculate quaternion q corresponding to 3x3 attitude matrix R
			input:
				const double* R --- pointer to a 3x3 attitude matrix
			output:
				double* q       --- pointer to a 4x1 quaternion with q0 being scalar part
		*/
void pony_linal_mat2quat(double* q, const double* R)
{
	size_t i;
	double _q4;

	// absolute values squared four times
	q[0] = 1 + R[0] + R[4] + R[8];
	q[1] = 1 + R[0] - R[4] - R[8];
	q[2] = 1 - R[0] + R[4] - R[8];
	q[3] = 1 - R[0] - R[4] + R[8];

	for (i = 0; i < 4; i++)
		if (q[i] >= 1)
			break;

	q[i] = sqrt(q[i])/2;
	_q4 = 1/(4*q[i]);
	switch (i) {
		case 0:
			q[1] = (R[5]-R[7])*_q4;
			q[2] = (R[6]-R[2])*_q4;
			q[3] = (R[1]-R[3])*_q4;
			break;
		case 1:
			q[0] = (R[5]-R[7])*_q4;
			q[2] = (R[1]+R[3])*_q4;
			q[3] = (R[6]+R[2])*_q4;
			break;
		case 2:
			q[0] = (R[6]-R[2])*_q4;
			q[1] = (R[1]+R[3])*_q4;
			q[3] = (R[5]+R[7])*_q4;
			break;
		case 3:
			q[0] = (R[1]-R[3])*_q4;
			q[1] = (R[6]+R[2])*_q4;
			q[2] = (R[5]+R[7])*_q4;
			break;
		default: // invalid matrix
			q[0] = 0;
			q[1] = 0;
			q[2] = 0;
			q[3] = 0;
			break;
	}
}

		/*
			calculate 3x3 attitude matrix R corresponding to quaternion q
			input:
				const double* q --- pointer to a 4x1 quaternion (with q0 being scalar part)
			output:
				double* R       --- pointer to a 3x3 attitude matrix
		*/
void pony_linal_quat2mat(double* R, const double* q)
{
	double q11, q22, q33, q10, q20, q30, q12, q23, q31;

	q11 = 2*q[1]*q[1];
	q22 = 2*q[2]*q[2];
	q33 = 2*q[3]*q[3];

	q10 = 2*q[1]*q[0];
	q20 = 2*q[2]*q[0];
	q30 = 2*q[3]*q[0];

	q12 = 2*q[1]*q[2];
	q23 = 2*q[2]*q[3];
	q31 = 2*q[3]*q[1];

	R[0] = 1 - q22 - q33; R[1] =     q12 + q30; R[2] =     q31 - q20;
	R[3] =     q12 - q30; R[4] = 1 - q33 - q11; R[5] =     q23 + q10;
	R[6] =     q31 + q20; R[7] =     q23 - q10; R[8] = 1 - q11 - q22;
}

		/*
			calculate 3x3 transition matrix R from E-N-U corresponding to roll, pitch and yaw
			input:
				const double* rpy --- pointer to a roll, pitch and yaw (radians, airborne frame: X longitudinal, Y normal, Z right-wing)
			output:
				double* R         --- pointer to a 3x3 transition matrix R from E-N-U
		*/
void pony_linal_rpy2mat(double* R, const double* rpy)
{
	double sr, cr, sp, cp, sy, cy;

	sr = sin(rpy[0]); cr = cos(rpy[0]);
	sp = sin(rpy[1]); cp = cos(rpy[1]);
	sy = sin(rpy[2]); cy = cos(rpy[2]);

	R[0] =     cp*sy;         R[1] =     cp*cy;         R[2] =     sp;
	R[3] = -cr*sp*sy + sr*cy; R[4] = -cr*sp*cy - sr*sy; R[5] =  cr*cp;
	R[6] =  sr*sp*sy + cr*cy; R[7] =  sr*sp*cy - cr*sy; R[8] = -sr*cp;
}

		/*
			calculate roll, pitch and yaw corresponding to 3x3 transition matrix R from E-N-U
			input:
				const double* R --- pointer to a 3x3 transition matrix R from E-N-U
			output:
				double* rpy     --- pointer to a roll, pitch and yaw (radians, airborne frame: X longitudinal, Z right-wing)
		*/
void pony_linal_mat2rpy(double* rpy, const double* R)
{
	const double pi2 = 6.283185307179586476925286766559005768; // pi*2, IEEE-754 quadruple precision
	double dp, dm;

	rpy[1] = atan2(R[2], sqrt(R[5]*R[5] + R[8]*R[8])); // pitch angle

	if (fabs(R[2]) < 0.5) {          // pitch magnitude below 30 deg
		rpy[0] = -atan2(R[8], R[5]); // roll angle
		rpy[2] =  atan2(R[0], R[1]); // yaw  angle
	}
	else {                           // pitch magnitude over 30 deg
		dp =  atan2(R[3]-R[7], R[6]+R[4]);
		dm = -atan2(R[3]+R[7], R[6]-R[4]);

		if (R[0] <= 0 && -R[8] <= 0 && dp > 0)      dp -= pi2;
		else if (R[0] >= 0 && -R[8] >= 0 && dp < 0) dp += pi2;
		else if (R[0] <= 0 && -R[8] >= 0 && dm > 0) dm -= pi2;
		else if (R[0] >= 0 && -R[8] <= 0 && dm < 0) dm += pi2;

		rpy[0] = (dp - dm)/2; // roll angle
		rpy[2] = (dp + dm)/2; // yaw  angle
	}
}

		/*
			calculate 3x3 rotation matrix R for 3x1 Euler vector e via Rodrigues' formula
			input:
				const double* e --- pointer to a 3x1 Euler vector
			output:
				double* R       --- pointer to a 3x3 rotation matrix,
				                    R = E + sin|e|/|e|*[e,] + (1-cos|e|)/|e|^2*[e,]^2
		*/
void pony_linal_eul2mat(double* R, const double* e)
{
	const size_t taylor_deg = 10;  // yields expansion error of 2^-105, which falls below IEEE 754-2008 quad precision rounding error when multiplied by an argument less than 2^-8
	const double eps = 1.0/0x0100; // 2^-8, guaranteed non-zero value in IEEE754 half-precision format

	double
		e0, e02,    // |e|, |e|^2
		e1, e2, e3, // e[i]^2
		s, c,       // sin|e|/|e|, (1 - cos|e|)/|e|^2
		ps, pc;     // Taylor expansion terms
	unsigned char i;

	// e[i]^2, |e|^2, |e|
	e1 = e[0]*e[0], e2 = e[1]*e[1], e3 = e[2]*e[2];
	e02 = e1 + e2 + e3;
	e0 = sqrt(e02);
	// sin|e|/|e|, (1 - cos|e|)/|e|^2
	if (e0 > eps) {
		s = sin(e0)/e0;
		c = (1 - cos(e0))/e02;
	}
	else {
		// Taylor expansion for sin(x)/x, (1-cos(x))/x^2 up to the defined degree (only even powers in each series)
		s = 1, c = 1/2.;
		for (i = 2, ps = -e02/6, pc = -e02/24; i < taylor_deg && ps != 0.0; i += 2, ps *= -e02/(i*(i+1.0)), pc *= -e02/((i+1.0)*(i+2.0)))
			s += ps, c += pc;
	}

	// rotation matrix
	R[0] =  1 - (e2 + e3)*c;      R[1] =  e[2]*s + e[0]*e[1]*c; R[2] = -e[1]*s + e[2]*e[0]*c;
	R[3] = -e[2]*s + e[0]*e[1]*c; R[4] =  1 - (e3 + e1)*c;      R[5] =  e[0]*s + e[1]*e[2]*c;
	R[6] =  e[1]*s + e[2]*e[0]*c; R[7] = -e[0]*s + e[1]*e[2]*c; R[8] =  1 - (e1 + e2)*c;
}

		/*
			calculate quaternion q for 3x1 Euler vector e
			input:
				const double* e --- pointer to a 3x1 Euler vector
			output:
				double* q       --- pointer to a 4x1 quaternion with q0 being scalar part
		*/
void pony_linal_eul2quat(double* q, const double* e  )
{
	const size_t taylor_deg = 12;  // yields expansion error of 2^-124, which falls below IEEE 754-2008 quad precision rounding error
	const double eps = 1.0/0x0100; // 2^-8, guaranteed non-zero value in IEEE754 half-precision format

	double
		e0, e02,    // |e|, |e|^2
		s, c,       // sin|e|/|e|, (1 - cos|e|)/|e|^2
		ps, pc;     // Taylor expansion terms
	unsigned char i;

	// e[i]^2, |e|^2, |e|
	e02 = (e[0]*e[0] + e[1]*e[1] + e[2]*e[2])/4;
	e0  = sqrt(e02);
	// sin|e/2|/|e/2|, cos|e/2|
	if (e0 > eps) {
		s = sin(e0)/e0;
		c = cos(e0);
	}
	else {
		// Taylor expansion for sin(x)/x, cos(x) up to the defined degree (only even powers in each series)
		s = 1, c = 1;
		for (i = 2, ps = -e02/6, pc = -e02/24; i < taylor_deg && ps != 0.0; i += 2, ps *= -e02/(i*(i+1.0)), pc *= -e02/((i+1.0)*(i+2.0)))
			s += ps, c += pc;
	}

	// normalized quaternion
	q[0] = c;
	for (i = 0; i < 3; i++)
		q[1+i] = e[i]*s/2;
}

	// routines for n x n upper-triangular matrices lined up in one-dimensional array
		/*
			convert index for upper-triangular matrix lined up in one-dimensional array: (i,j) -> k
			input:
				const size_t i, j --- row and column indexes in upper-triangular matrix,
				                      i,j = 0,1,...,n-1
				const size_t n    --- upper-triangular matrix dimension
			output:
				size_t* k         --- pointer to an index in one-dimensional array,
				                      k = 0,1,...,[n*(n+1)/2]-1
		*/
void pony_linal_u_ij2k(size_t* k, const size_t i, const size_t j, const size_t n)
{
	*k = i*(2*n - 1 - i)/2 + j;
}

		/*
			convert index for upper-triangular matrix lined up in one-dimensional array: k -> (i,j)
			input:
				const size_t  k    --- index in one-dimensional array,
				                       k = 0,1,...,[n(n+1)/2]-1
				const size_t  n    --- upper-triangular matrix dimension
			output:
				const size_t* i, j --- pointers to a row and column indexes in upper-triangular matrix,
				                       i,j = 0,1,...,n-1
									   NULL pointers omit assignment
		*/
void pony_linal_u_k2ij(size_t* i, size_t* j, const size_t k, const size_t n)
{
	float onehalf_plus_n = 0.5F + (float)n;
	size_t i0;

	i0 = (size_t)(floor(onehalf_plus_n - sqrt(onehalf_plus_n*onehalf_plus_n - 2.0F*(float)k)) + 0.5F);
	if (i != NULL)
		*i = i0;
	if (j != NULL)
		*j = k - (2*n - 1 - i0)*i0/2;
}

		/*
			fill the diagonal with array elements
			input:
				double* u       --- pointer to an upper-triangular n x n matrix
				                    lined up in one-dimensional array of n(n+1)/2 x 1
				const double* d --- pointer to an array of n diagonal elements
				const size_t n  --- dimension
		*/
void pony_linal_diag2u(double* u, const double* d, const size_t n)
{
	size_t i, k;

	for (i = 0, k = 0; i < n; k += n-i, i++)
		u[k] = d[i];
}

		/*
			multiply upper-triangular matrix lined up in one-dimensional array by a regular matrix
			input:
				const double* u    --- pointer to an upper-triangular n x n matrix
				                       lined up in one-dimensional array of n(n+1)/2 x 1
				const double* v    --- pointer to a regular n x m matrix
				const size_t  n, m --- dimensions
			output:
				double* res        --- pointer to n x m matrix,
				                       res = U*V
			note:
				overwriting input (double* res = double* v) is allowed
		*/
void pony_linal_u_mul(double* res, const double* u, const double* v, const size_t n, const size_t m)
{
	size_t i, j, k, p, p0, mn;

	mn = m*n;
	for (j = 0; j < m; j++) {
		for (i = 0, k = 0, p0 = j; i < n; i++, p0 += m) {
			res[p0] = u[k]*v[p0];
			for (p = p0+m, k++; p < mn; p += m, k++)
				res[p0] += u[k]*v[p];
		}
	}
}

		/*
			multiply transposed upper-triangular matrix lined up in one-dimensional array of n(n+1)/2 x 1, by a regular matrix
			input:
				const double* u   --- pointer to an upper-triangular matrix 
				                      lined up in one-dimensional array of n(n+1)/2 x 1
				const double* v   --- pointer to a regular n x m matrix
				const size_t  n,m --- dimensions
			output:
				double* res       --- pointer to a regular n x m matrix,
				                      res = U^T*V
			note:
				overwriting input (double* res = double* v) is allowed
		*/
void pony_linal_uT_mul(double* res, const double* u, const double* v, const size_t n, const size_t m)
{
	size_t i, j, k, k0, l, r, r0, ni;

	for (i = 0, k0 = n*(n+1)/2-1, r0 = m*n-1; i < n; k0 -= i+2, i++) {
		ni = n - i;
		for (j = 0; j < m; j++, r0--) {
			res[r0] = v[r0]*u[k0];
			for (l = 1, r = r0-m, k = k0-i-1; l < ni; l++, k -= i+l, r -= m)
				res[r0] += v[r]*u[k];
		}
	}
}

		/*
			multiply a regular matrix by upper-triangular matrix lined up in one-dimensional array of n(n+1)/2 x 1
			input:
				const double* v    --- pointer to a regular m x n matrix
				const double* u    --- pointer to an upper-triangular n x n matrix
				                       lined up in one-dimensional array of n(n+1)/2 x 1
				const size_t  m, n --- dimensions
			output:
				double* res        --- pointer to m x n matrix,
				                       res = V*U
			note:
				overwriting input (double* res = double* v) is allowed
		*/
void pony_linal_mul_u(double* res, const double* v, const double* u, const size_t m, const size_t n)
{
	size_t 
		i, j, k, k0, r, r0, 
		nj, mn, nn1;

	mn = m*n;
	nn1 = n*(n+1)/2 - 1;
	for (r0 = mn-1; r0 < mn; r0 -= n) {
		for (j = 0, k0 = nn1; j < n; k0 -= j+2, j++) {
			r = r0 - j;
			nj = n - j;
			res[r] = v[r]*u[k0];
			for (i = 1, k = k0-j-1; i < nj; i++, k -= i+j)
				res[r] += v[r-i]*u[k];
		}
		if (r0 < n)
			break;
	}		
}

		/*
			multiply a regular matrix by transposed upper-triangular matrix lined up in one-dimensional array of n(n+1)/2 x 1
			input:
				const double* v    --- pointer to a regular m x n matrix
				const double* u    --- pointer to an upper-triangular n x n matrix
				                       lined up in one-dimensional array of n(n+1)/2 x 1
				const size_t  m, n --- dimensions
			output:
				double* res        --- pointer to m x n matrix,
				                       res = V*U^T
			note:
				overwriting input (double* res = double* v) is allowed
		*/
void pony_linal_mul_uT(double* res, const double* v, const double* u, const size_t m, const size_t n)
{
	size_t i, j, ij, k, k0, p, n1;

	for (j = 0, k0 = 0, n1 = n; j < n; j++, k0 += n1, n1--)
		for (i = 0, ij = j; i < m; i++, ij +=n) {
			res[ij] = v[ij]*u[k0];
			for (k = k0+1, p = 1; p < n1; p++, k++)
				res[ij] += v[ij+p]*u[k];
		}
}

		/* 
			multiply a regular matrix by itself transposed, storing upper part of the result lined up in one-dimensional array of n(n+1)/2 x 1 
			input:
				const double* v    --- pointer to a regular n x m matrix
				const size_t  n, m --- dimensions
			output:
				double* res        --- pointer to an upper-triangular n x n matrix
				                       lined up in one-dimensional array of n(n+1)/2 x 1
				                       res = v*v^T
		*/
void pony_linal_msq2T_u(double* res, const double* v, const size_t n, const size_t m)
{
	size_t i, i1, j, k, im, i1m;

	for (i = 0, i1 = 0, k = 0; i < n; i++)
		for (i1 = i, im = i*m; i1 < n; i1++, k++)
			for (j = 0, i1m = i1*m, res[k] = 0; j < m; j++)
				res[k] += v[im + j]*v[i1m + j];
}

		/* 
			multiply a transposed regular matrix by itself, storing upper part of the result lined up in one-dimensional array of n(n+1)/2 x 1 
			input:
				const double* v    --- pointer to a regular m x n matrix
				const size_t  m, n --- dimensions
			output:
				double* res        --- pointer to an upper-triangular n x n matrix
				                       lined up in one-dimensional array of n(n+1)/2 x 1
				                       res = v^T*v
		*/
void pony_linal_msq1T_u(double* res, const double* v, const size_t m, const size_t n)
{
	size_t j, j1, i, k, mn;

	mn = m*n;
	for (j = 0, j1 = 0, k = 0; j < n; j++)
		for (j1 = j; j1 < n; j1++, k++)
			for (i = 0, res[k] = 0; i < mn; i += n)
				res[k] += v[i+j]*v[i+j1];
}

		/*
			invert upper-triangular matrix lined up in one-dimensional array of m(m+1)/2 x 1
			input:
				const double* u --- pointer to an upper-triangular matrix
				                    lined up in one-dimensional array of n(n+1)/2 x 1
				const size_t  n --- dimension
			output:
				double* res     --- pointer to an upper-triangular matrix
				                    lined up in one-dimensional array of n(n+1)/2 x 1,
				                    res = U^-1
			note:
				overwriting input (double* res = double* u) is allowed
		*/
void pony_linal_u_inv(double* res, const double* u, const size_t n)
{
	size_t i, j, k, k0, p, q, p0, r;
	double s;

	for (j = 0, k0 = n*(n+1)/2 - 1; j < n; k0 -= j+2, j++) {
		res[k0] = 1/u[k0]; // division by zero if matrix is not invertible
		for (i = j+1, k = k0-j-1; i < n; k -= i+1, i++) {
			p0 = k-(i-j);
			for (p = k, q = k0, r = 0, s = 0; q > k; p--, q -= j+r+1, r++)
				s += u[p]*res[q];
			res[k] = -s/u[p0]; // division by zero if matrix is not invertible
		}
	}
}

		/*
			calculate square (with right transposition) of upper-triangular matrix lined up in one-dimensional array of n(n+1)/2 x 1
			input:
				const double* u --- pointer to an upper-triangular matrix 
				                    lined up in one-dimensional array of n(n+1)/2 x 1
				const size_t  n --- dimension
			output:
				double* res     --- pointer to a symmetric matrix
				                    lined up in one-dimensional array of n(n+1)/2 x 1,
				                    res = U*U^T
			note:
				overwriting input (double* res = double* u) is allowed
		*/
void pony_linal_uuT(double* res, const double* u, const size_t n)
{
	size_t i, j, k, k0, k1, m, m1, p;

	for (i = 0, k0 = 0; i < n; i++)
		for (j = 0, m = n-i, k1 = k0; j < m; j++, k0++) {
			res[k0] = u[k0]*u[k1];
			for (k = k0+1, k1++, p = 1, m1 = m-j; p < m1; p++, k++, k1++)
				res[k0] += u[k]*u[k1];
		}
}

		/*
			calculate square (with left transposition) of upper-triangular matrix lined up in one-dimensional array of n(n+1)/2 x 1
			input:
				const double* u --- pointer to an upper-triangular matrix 
				                    lined up in one-dimensional array of n(n+1)/2 x 1
				const size_t  n --- dimension
			output:
				double* res     --- pointer to a symmetric matrix
				                    lined up in one-dimensional array of n(n+1)/2 x 1,
				                    res = U^T*U
			note:
				overwriting input (double* res = double* u) is allowed
		*/
void pony_linal_uTu(double* res, const double* u, const size_t n)
{
	size_t i, j, k, k0, k1, p;

	for (i = 0, k0 = n*(n+1)/2-1; i < n; i++)
		for (j = 0; j <= i; j++, k0--) {
			k1 = k0 - (i-j);
			res[k0] = u[k0]*u[k1];
			for (p = i+1, k = k0-p, k1 -= p; p < n; p++, k -= p, k1 -= p)
				res[k0] += u[k]*u[k1];
		}
}

		/*
			calculate Cholesky upper-triangular factorization P = S*S^T,
			where P is symmetric positive-definite matrix
			input:
				const double* P --- pointer to an upper-triangular part of symmetric n x n positive-definite matrix R
				                    lined in one-dimensional array n(n+1)/2 x 1
				const size_t  n --- dimension
			output:
				double* S       --- pointer to an upper-triangular part of a Cholesky factor S
				                    lined in one-dimensional array n(n+1)/2 x 1
			note:
				overwriting input (double* P = double* S) is allowed
		*/
void pony_linal_chol(double* S, const double* P, const size_t n)
{
	size_t i, j, k, k0, p, q, p0;
	double s;

	for (j = 0, k0 = n*(n+1)/2 - 1; j < n; k0 -= j+2, j++) {
		p0 = k0+j;
		for (p = k0+1, s = 0; p <= p0; p++)
			s += S[p]*S[p];
		S[k0] = sqrt(P[k0] - s);
		for (i = j+1, k = k0-j-1; i < n; k -= i+1, i++) {
			for (p = k0+1, q = k+1, s = 0; p <= p0; p++, q++)
				s += S[p]*S[q];
			S[k] = (S[k0] == 0) ? 0 : (P[k] - s)/S[k0];
		}
	}
}

	// square root Kalman filtering
		/*
			check measurement residual magnitude against predicted covariance level
			input:
				double*      x       --- pointer to a current estimate of n x 1 state vector
				double*      S       --- pointer to an upper-truangular part of the Cholesky factor of current covariance matrix
				                         lined in one-dimensional array n(n+1)/2 x 1
				double       z       --- scalar measurement value
				double*      h       --- pointer to a linear measurement model matrix, so that z = h*x + r
				double       sigma   --- measurement error a priori standard deviation, so that sigma = sqrt(E[r^2])
				double       k_sigma --- confidence coefficient, 3 for 3-sigma (99.7%), 2 for 2-sigma (95%), etc.
				const size_t n       --- state vector size
			return value:
				1 if measurement residual magnitude lies below the predicted covariance level, 
				  i.e. |z - h*x| < k_sigma*sqrt(h*S*S^T*h^T + sigma^2)
				0 otherwise
		*/
char pony_linal_check_measurement_residual(double* x, double* S, double z, double* h, double sigma, double k_sigma, const size_t n)
{
	size_t i, j, k;

	double s;

	sigma = sigma*sigma;
	for (i = 0; i < n; i++) {
		// dz = z - h*x: residual
		z -= h[i]*x[i];
		// s = h*S*S^T*h^T: predicted variance 
		for (j = 0, k = i, s = 0; j <= i; j++, k += n-j)
			s += h[j]*S[k];
		sigma += s*s;
	}
	sigma = sqrt(sigma);

	return (fabs(z) < k_sigma*sigma) ? 1 : 0;
}

		/*
			perform square root Kalman filter update phase
			input:
				double*      x     --- pointer to a current estimate of n x 1 state vector
				double*      S     --- pointer to an upper-truangular part of the Cholesky factor of current covariance matrix
				                       lined in one-dimensional array n(n+1)/2 x 1
				double       z     --- scalar measurement value
				double*      h     --- pointer to a linear measurement model matrix, so that z = h*x + r
				double       sigma --- measurement error a priori standard deviation, so that sigma = sqrt(E[r^2])
				const size_t n     --- state vector size
			output:
				double* x --- pointer to an updated estimate of state vector (overwrites input)
				double* S --- pointer to an upper-truangular part of the Cholesky factor of updated covariance matrix
				              lined in one-dimensional array n(n+1)/2 x 1 (overwrites input)
				double* K --- pointer to a Kalman gain
			return value:
				measurement residual before update
		*/
double pony_linal_kalman_update(double* x, double* S, double* K, double z, double* h, double sigma, const size_t n)
{
	double d, d1, sd, sd1, f, e;
	size_t i, j, k;

	// e0 stored in K
	for (i = 0; i < n; i++)
		K[i] = 0;

	// d0
	d = sigma*sigma;

	// S
	for (i = 0; i < n; i++) {
		// f = S^T*h
		f = S[i]*h[0];
		for (j = 1, k = i+n-1; j <= i; j++, k += n-j)
			f += S[k]*h[j];
		if (f == 0)
			continue; // optimization for sparse matrices
		// d
		d1 = d + f*f;
		sd = sqrt(d);
		sd1 = sqrt(d1);
		// S^+, e
		for (j = 0, k = i; j <= i; j++, k += n-j) {
			e = K[j];
			K[j] += S[k]*f;
			S[k] *= sd/sd1;
			if (e != 0)               // optimization for degenerate case
				S[k] -= e*f/(sd*sd1); // d = 0 (sigma = 0) allowed only if e = 0
		}
		d = d1;

		// dz
		z -= h[i]*x[i];
	}

	// K, x
	for (i = 0; i < n; i++) {
		if (K[i] != 0) { // optimization for degenerate case
			K[i] /= d;   // d = 0 (sigma = 0) allowed only for K[i] = 0
			x[i] += K[i]*z;
		}
	}
	return z;
}

		/*
			perform square root Kalman filter prediction phase: identity state transition, scalar process noise covariance
			Q = q^2*I = E[(x_i - x_i-1)*(x_i - x_i-1)^T]
			input:
				double*      S  --- pointer to an upper-truangular part of the Cholesky factor of current covariance matrix
				                    lined in one-dimensional array n(n+1)/2 x 1
				double       q2 --- process noise variance, q2 = q^2 >= 0
				const size_t n  --- state vector size
			output:
				double* S --- pointer to an upper-truangular part of a Cholesky factor of predicted covariance matrix
				              lined in one-dimensional array n(n+1)/2 x 1 (overwrites input)
		*/
void pony_linal_kalman_predict_I_qI(double* S, double q2, const size_t n)
{
	size_t i;

	pony_linal_uuT(S, S, n);    // P = S*S^T
	for (i = 0; i < n; i++)
		S[i*(2*n-i+1)/2] += q2; // P = P + Q, Q = q2*I
	pony_linal_chol(S, S, n);   // P = S*S^T
}

		/*
			perform square root Kalman filter prediction phase: identity state transition, reduced scalar process noise covariance
			    [q^2*I 0 0]                                     
			Q = [  0   0 0] = E[(x_i - x_i-1)*(x_i - x_i-1)^T]
			    [  0   0 0]
			input:
				double*      S  --- pointer to an upper-truangular part of the Cholesky factor of current covariance matrix
				                    lined in one-dimensional array n(n+1)/2 x 1
				double       q2 --- nonzero process noise variance, q2 = q^2 >= 0
				const size_t n  --- state vector size
				const size_t m  --- nonzero process noise vector size
			output:
				double* S --- pointer to an upper-truangular part of the Cholesky factor of predicted covariance matrix
				              lined in one-dimensional array n(n+1)/2 x 1 (overwrites input)
		*/
void pony_linal_kalman_predict_I_qIr(double* S, double q2, const size_t n, const size_t m)
{
	size_t i;

	pony_linal_uuT(S, S, n);    // P = S*S^T
	for (i = 0; i < m; i++)
		S[i*(2*n-i+1)/2] += q2; // P = P + Q, Q = [q2*I 0; 0 0]
	pony_linal_chol(S, S, n);   // P = S*S^T
}

		/*
			perform square root Kalman filter prediction phase: identity state transition, diagonal process noise covariance
			    [q_1^2  0   0 0]  
			    [  .    .   . .]
			Q = [  0  q_m^2 0 0] = E[(x_i - x_i-1)*(x_i - x_i-1)^T]
			    [  0    0   0 0]
			    [  0    0   0 0]
			input:
				double*      S  --- pointer to an upper-truangular part of the Cholesky factor of current covariance matrix
				                    lined in one-dimensional array n(n+1)/2 x 1
				double*      q2 --- pointer to a nonzero process noise variance vector, q2[i] = q_i^2 >= 0, i = 1..m
				const size_t n  --- state vector size
				const size_t m  --- nonzero process noise vector size
			output:
				double* S --- pointer to an upper-truangular part of the Cholesky factor of predicted covariance matrix
				              lined in one-dimensional array n(n+1)/2 x 1 (overwrites input)
		*/
void pony_linal_kalman_predict_I_diag(double* S, double* q2, const size_t n, const size_t m)
{
	size_t i;

	pony_linal_uuT(S, S, n);       // P = S*S^T
	for (i = 0; i < m; i++)
		S[i*(2*n-i+1)/2] += q2[i]; // P = P + Q, Q = [diag(q2) 0; 0 0]
	pony_linal_chol(S, S, n);      // P = S*S^T
}

		/*
			perform square root Kalman filter prediction phase: identity state transition
			          [q_11 .. q_1m 0 0]  
			[Q 0 0]   [  .  ..   .  . .]
			[0 0 0] = [q_1m .. q_mm 0 0] = E[(x_i - x_i-1)*(x_i - x_i-1)^T]
			[0 0 0]   [  0  ..   0  0 0]
			          [  0  ..   0  0 0]
			input:
				double*      S --- pointer to an upper-truangular part of the Cholesky factor of current covariance matrix
				                   lined in one-dimensional array n(n+1)/2 x 1
				double*      Q --- upper triangular part of the nonzero process noise covariance, lined in one-dimensional array m(m+1)/2 x 1 

				const size_t n --- state vector size
				const size_t m --- nonzero process noise vector size
			output:
				double* S --- pointer to an upper-truangular part of the Cholesky factor of predicted covariance matrix
				              lined in one-dimensional array n(n+1)/2 x 1 (overwrites input)		
		*/
void pony_linal_kalman_predict_I(double* S, double* Q, const size_t n, const size_t m)
{
	size_t i, k0, j, k1;

	pony_linal_uuT(S, S, n);      // P = S*S^T
	for (i = 0; i < m; i++) {
		k0 = i*(2*n-i+1)/2;
		k1 = i*(2*m-i+1)/2;
		for (j = 0; j < m-i; j++)
			S[k0+j] += Q[k1+j];   // P = P + [Q 0; 0 0]
	}
	pony_linal_chol(S, S, n);     // P = S*S^T
}

		/*
			perform square root Kalman filter prediction phase: upper triangular state transition, diagonal process noise covariance
			                         [U11 . U1n]
			x_i = F*x_i-1,       F = [ 0  .  . ]
			                         [ 0  0 Unn]
			      [q_1^2  0   0 0]  
			      [  .    .   . .]
			Q   = [  0  q_m^2 0 0] = E[(x_i - x_i-1)*(x_i - x_i-1)^T]
			      [  0    0   0 0]
			      [  0    0   0 0]
			input:
				double*      x  --- pointer to a current estimate of n x 1 state vector
				double*      S  --- pointer to an upper-truangular part of the Cholesky factor of current covariance matrix
				                    lined in one-dimensional array n(n+1)/2 x 1
				double*      U  --- pointer to an upper-truangular state transition matrix
				                    lined in one-dimensional array m(m+1)/2 x 1
				double*      q2 --- pointer to a nonzero process noise variance vector, q2[i] = q_i^2 >= 0, i = 1..r
				const size_t n  --- state vector size
				const size_t m  --- nonzero process noise vector size
			output:
				double* x --- pointer to a predicted estimate of state vector (overwrites input)
				double* S --- pointer to an upper-truangular part of the Cholesky factor of predicted covariance matrix
				              lined in one-dimensional array n(n+1)/2 x 1 (overwrites input)
		*/
void pony_linal_kalman_predict_U_diag(double* x, double* S, double* U, double* q2, const size_t n, const size_t m)
{
	size_t i, j, j1, k0, k1, k2, k, k10, j10;

	pony_linal_u_mul(x, U, x, n, 1);    // x = F*x  	
	for (i = 0, k0 = 0; i < n; i++) {
		k10 = i*(2*n-i+1)/2;            // start from the diagonal in U
		j10 = n-i-1;
		for (j = i; j < n; j++, k0++) {
			k1 = k10;
			S[k0] *= U[k1];             // S = F*S
			k2 = j*(2*n-j+1)/2;         // finish at the diagonal in S
			for (j1 = j10, k1++, k = k0+j1; j1 > 0 && k <= k2; j1--, k1++, k += j1)
				S[k0] += S[k]*U[k1];
		}
	}
	pony_linal_uuT(S, S, n);            // P = S*S^T
	for (i = 0; i < m; i++)
		S[i*(2*n-i+1)/2] += q2[i];      // P = P + Q, Q = [diag(q2) 0; 0 0]
	pony_linal_chol(S, S, n);           // P = S*S^T
}

		/*
			perform square root Kalman filter prediction phase: upper triangular state transition
			                               [U11 . U1n]
			x_i     = F*x_i-1,         F = [ 0  .  . ] 
			                               [ 0  0 Unn]
			          [q_11 .. q_1m 0 0]  
			[Q 0 0]   [  .  ..   .  . .]
			[0 0 0] = [q_1m .. q_mm 0 0] = E[(x_i - x_i-1)*(x_i - x_i-1)^T]
			[0 0 0]   [  0  ..   0  0 0]
			          [  0  ..   0  0 0]
			input:
				double*      x --- pointer to a current estimate of n x 1 state vector
				double*      S --- pointer to an upper-truangular part of the Cholesky factor of current covariance matrix
				                   lined in one-dimensional array n(n+1)/2 x 1
				double*      U --- pointer to an upper-truangular state transition matrix
				                   lined in one-dimensional array n(n+1)/2 x 1
				double*      Q --- pointer to an upper triangular part of the nonzero process noise covariance
				                   lined in one-dimensional array m(m+1)/2 x 1 
				const size_t n --- nonzero process noise vector size
				const size_t m --- state vector size
			output:
				double* x --- pointer to a predicted estimate of state vector (overwrites input)
				double* S --- pointer to an upper-truangular part of the Cholesky factor of predicted covariance matrix
				              lined in one-dimensional array n(n+1)/2 x 1 (overwrites input)
		*/
void pony_linal_kalman_predict_U(double* x, double* S, double* U, double* Q, const size_t n, const size_t m)
{
	size_t i, j, j1, k0, k1, k2, k, k10, j10;

	pony_linal_u_mul(x, U, x, n, 1);    // x = F*x  	
	for (i = 0, k0 = 0; i < n; i++) {
		k10 = i*(2*n-i+1)/2;            // start from the diagonal in U
		j10 = n-i-1;
		for (j = i; j < n; j++, k0++) {
			k1 = k10;
			S[k0] *= U[k1];             // S = F*S
			k2 = j*(2*n-j+1)/2;         // finish at the diagonal in S
			for (j1 = j10, k1++, k = k0+j1; j1 > 0 && k <= k2; j1--, k1++, k += j1)
				S[k0] += S[k]*U[k1];
		}
	}
	pony_linal_uuT(S, S, n);            // P = S*S^T
	for (i = 0; i < m; i++) {
		k0 = i*(2*n-i+1)/2;
		k1 = i*(2*m-i+1)/2;
		for (j = 0; j < m-i; j++)
			S[k0+j] += Q[k1+j];         // P = P + [Q 0; 0 0]
	}
	pony_linal_chol(S, S, n);           // P = S*S^T
}
