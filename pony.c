// Dec-2019
//
// PONY core source code

#include <stdlib.h>
#include <math.h>

#include "pony.h"


// core functions to be used in host application
char pony_add_plugin( void(*newplugin)(void) );	// add plugin to the plugin execution list,		input: pointer to plugin function,				output: OK/not OK (1/0)
char pony_init(char*);							// initialize the bus, except for core,			input: configuration string (see description),	output: OK/not OK (1/0)
char pony_step(void);							// step through the plugin execution list,														output: OK/not OK (1/0)
char pony_terminate(void);						// terminate operation,																			output: OK/not OK (1/0)



// bus instance
pony_bus pony = {
	pony_bus_version,			// ver
	pony_add_plugin,			// add_plugin
	pony_init,					// init
	pony_step,					// step
	pony_terminate,				// terminate
	{ NULL, 0, -1, 0 } };		// core.plugins, core.plugin_count, core.exit_plugin_id, core.host_termination







// service subroutines

	// locate parameter group within a configuration string
	// input:
	// char* groupname	-	group identifier (see documentation)
	//						or 
	//						empty string to locate a substring that is outside of any group
	// char* cfgstr	-	configuration string to parse
	// int cfglen		-	number of characters in cfgstr to parse
	//
	// output:
	// char** groupptr	-	reference to a pointer to the starting character of the group contents within a configuration string
	// int* grouplen	-	reference to a number of characters in the group contents
	//
	// return value:		1 if the requested group found and grouplen > 0
	//						0 otherwise
	//
	// working example:
	// groupname = "gnss:"
	// cfgstr = "{gnss: {gps: eph_in="gpsa.nav", obs_in="gpsa.obs"}}, out="sol.txt""
	// cfglen = 66
char pony_locatecfggroup(const char* groupname, char* cfgstr, const int cfglen, char** groupptr, int* grouplen)
{
	int i, j;
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
			for (; cfgstr[i] && (cfgstr[i] <= ' ' || cfgstr[i] == ',') && i < cfglen; i++); 
			// if no group started at this point
			if (cfgstr[i] != '{')
				break;
			// if a group started
			else {
				group_layer = 1;
				while (group_layer > 0 && cfgstr[i] && i < cfglen) {
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
		for (i = 0; i < cfglen && cfgstr[i] && !group_found; i++) {
			// skip all non-printable characters, blank spaces and commas between groups
			for (; cfgstr[i] && (cfgstr[i] <= ' ' || cfgstr[i] == ',') && i < cfglen; i++); 
			// if a group started
			if (cfgstr[i] == '{') {
				group_layer = 1;
				// skip all non-printable characters and blank spaces at the beginning of the group
				for (i++; cfgstr[i] && (cfgstr[i] <= ' ') && i < cfglen; i++); 

				// check if the group is the one that has been requested
				group_found = 1;
				for (j = 0; cfgstr[i] && groupname[j] && i < cfglen; i++, j++)
					if (cfgstr[i] != groupname[j]) {
						group_found = 0;
						break;
					}
				// if did not reach the last character of groupname
				if (groupname[j])
					group_found = 0;

				if (group_found)
					// start from this point
					*groupptr = cfgstr + i;

				// go through the rest of the group
				while (group_layer > 0 && cfgstr[i] && i < cfglen) {
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
		}
	}

	return (*groupptr == NULL) ? 0 : 1;
}




// initialize epoch
void pony_init_epoch(pony_time_epoch *epoch)
{
	epoch->Y = 0;
	epoch->M = 0;
	epoch->D = 0;
	epoch->h = 0;
	epoch->m = 0;
	epoch->s = 0;
}




// initialize solution
void pony_init_solution(pony_sol *sol)
{
	int i;

	// pos & vel
	for (i = 0; i < 3; i++) {
		sol->x[i]	= 0;
		sol->llh[i]	= 0;
		sol->v[i]	= 0;
	}
	sol->x_valid	= 0;
	sol->llh_valid	= 0;
	sol->v_valid	= 0;
	// attitude
	for (i = 0; i < 4; i++)
		sol->q[i]	= 0; // quaternion
	sol->q_valid	= 0;
	for (i = 0; i < 9; i++)
		sol->L[i]	= 0; // attitude matrix
	sol->L_valid	= 0;
	for (i = 0; i < 3; i++)
		sol->rpy[i]	= 0; // attitude angles
	sol->rpy_valid	= 0;
	// clock
	sol->dt			= 0;
	sol->dt_valid	= 0;

}




// imu data handling subroutines
	// initialize imu structure
char pony_init_imu(void)
{
	// validity flags
	pony.imu->w_valid = 0;
	pony.imu->f_valid = 0;
	// drop the solution
	pony_init_solution( &(pony.imu->sol) );

	return 1;
}

	// free imu memory
void pony_free_imu(void)
{
	if (pony.imu == NULL)
		return;
	free(pony.imu);
	pony.imu = NULL;
}




// gnss data handling subroutines
	// initialize gnss settings
char pony_init_gnss_settings(pony_gnss *gnss)
{
	int i;

	pony_locatecfggroup( "", gnss->cfg, gnss->cfglength, &(gnss->cfg_settings), &(gnss->settings_length) );

	gnss->settings.sinEl_mask = 0;
	gnss->settings.code_sigma = 20;
	gnss->settings.phase_sigma = 0.01;
	for (i = 0; i < 3; i++)
		gnss->settings.ant_pos[i] = 0;
	gnss->settings.ant_pos_tol = -1;
	gnss->settings.leap_sec_def = 0;

	return 1;
}

	// initialize gnss gps constants
void pony_init_gnss_gps_const(pony_gnss *gnss)
{
	gnss->gps_const.pi			=  3.1415926535898;		// pi as in IS-GPS-200J (22 May 2018)
	gnss->gps_const.c			=  299792458;			// speed of light as in IS-GPS-200J (22 May 2018), m/s
	gnss->gps_const.mu			=  3.986005e14;			// Earth gravity constant as in IS-GPS-200J (22 May 2018), m^3/s^2
	gnss->gps_const.u			=  7.2921151467e-5;		// Earth rotation rate as in IS-GPS-200J (22 May 2018), rad/s
	gnss->gps_const.a			=  6378137.0;			// Earth ellipsoid semi-major axis as in WGS-84(G1762) 2014-07-08, m
	gnss->gps_const.e2			=  6.694379990141e-3;	// Earth ellipsoid first eccentricity squared as in WGS-84(G1762) 2014-07-08
	gnss->gps_const.F			= -4.442807633e-10;		// relativistic correction constant as in IS-GPS-200J (22 May 2018), s/sqrt(m)
	gnss->gps_const.sec_in_w	= 604800;				// seconds in a week
	gnss->gps_const.sec_in_d	=  86400;				// seconds in a day
	gnss->gps_const.F1			=  1575.42e6;			// nominal frequency for L1 signal
	gnss->gps_const.L1			= gnss->gps_const.c/gnss->gps_const.F1;		// nominal wavelength for L1 signal
	gnss->gps_const.F2			=  1227.60e6;			// nominal frequency for L2 signal
	gnss->gps_const.L2			= gnss->gps_const.c/gnss->gps_const.F2;		// nominal wavelength for L2 signal
}

	// initialize gnss gps structure
char pony_init_gnss_gps(pony_gnss_gps *gps, const int max_sat_count, const int max_eph_count)
{
	int i;

	gps->sat = NULL;
	gps->max_sat_count = 0;
	gps->max_eph_count = 0;

	// try to allocate memory for satellite data
	gps->sat = (pony_gnss_sat*)calloc( max_sat_count, sizeof(pony_gnss_sat) );
	if (gps->sat == NULL)
		return 0;
	gps->max_sat_count = max_sat_count;

	// initialize satellite data
	for (i = 0; i < gps->max_sat_count; i++) {
		gps->sat[i].eph			= NULL;
		gps->sat[i].eph_valid	= 0;
		gps->sat[i].obs			= NULL;
		gps->sat[i].obs_valid	= NULL;
		gps->sat[i].x_valid		= 0;
		gps->sat[i].v_valid		= 0;
		gps->sat[i].t_em_valid	= 0;
		gps->sat[i].sinEl_valid	= 0;
	}

	// try to allocate memory for each satellite ephemeris
	for (i = 0; i < gps->max_sat_count; i++) {
		gps->sat[i].eph = (double *)calloc( max_eph_count, sizeof(double) );
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

	// free gnss gps memory
void pony_free_gnss_gps(pony_gnss_gps *gps)
{
	int i;

	if (gps == NULL)
		return;

	// satellites
	if (gps->sat != NULL) 
	{
		for (i = 0; i < gps->max_sat_count; i++) 
			// ephemeris
			if (gps->sat[i].eph != NULL)
			{
				free(gps->sat[i].eph);
				gps->sat[i].eph = NULL;
			}
	
		free(gps->sat);
		gps->sat = NULL;
	}

	// gnss_gps structure
	free(gps);
}

	// initialize gnss glonass constants
void pony_init_gnss_glo_const(pony_gnss *gnss)
{
	gnss->glo_const.c			= 299792458;		// speed of light as in ICD GLONASS Edition 5.1 2008, m/s
	gnss->glo_const.mu			= 398600.4418e9;	// Earth gravity constant as in PZ-90.02, m^3/s^2
	gnss->glo_const.J02			= 1082625.75e-9;	// second zonal harmonic of geopotential as in PZ-90.02
	gnss->glo_const.u			= 7.292115e-5;		// Earth rotation rate as in PZ-90.02, rad/s
	gnss->glo_const.a			= 6378136.0;		// Earth ellipsoid semi-major axis as in PZ-90.02, m
	gnss->glo_const.e2			= 0.0066943662;		// Earth ellipsoid first eccentricity squared as in PZ-90.02
	gnss->glo_const.sec_in_d	=  86400;			// seconds in a day
	gnss->glo_const.F01			= 1602e6;			// nominal centre frequency for L1 signal as in ICD GLONASS Edition 5.1 2008, Hz
	gnss->glo_const.dF1			= 562.5e3;			// nominal channel separation for L1 signal as in ICD GLONASS Edition 5.1 2008, Hz
	gnss->glo_const.F02			= 1246e6;			// nominal centre frequency for L2 signal as in ICD GLONASS Edition 5.1 2008, Hz
	gnss->glo_const.dF2			= 437.5e3;			// nominal channel separation for L2 signal as in ICD GLONASS Edition 5.1 2008, Hz
}

	// initialize gnss glonass structure
char pony_init_gnss_glo(pony_gnss_glo *glo, const int max_sat_count, const int max_eph_count)
{
	int i;

	glo->sat = NULL;
	glo->freq_slot = NULL;
	glo->max_sat_count = 0;
	glo->max_eph_count = 0;

	// try to allocate memory for satellite data
	glo->sat = (pony_gnss_sat *)calloc( max_sat_count, sizeof(pony_gnss_sat) );
	if (glo->sat == NULL)
		return 0;
	glo->freq_slot = (int *)calloc( max_sat_count, sizeof(int) );
	glo->max_sat_count = max_sat_count;

	// initialize satellite data
	for (i = 0; i < glo->max_sat_count; i++) {
		glo->sat[i].eph			= NULL;
		glo->sat[i].eph_valid	= 0;
		glo->sat[i].obs			= NULL;
		glo->sat[i].obs_valid	= NULL;
		glo->sat[i].x_valid		= 0;
		glo->sat[i].v_valid		= 0;
		glo->sat[i].t_em_valid	= 0;
		glo->sat[i].sinEl_valid	= 0;
	}

	// try to allocate memory for each satellite ephemeris
	for (i = 0; i < glo->max_sat_count; i++) {
		glo->sat[i].eph = (double *)calloc( max_eph_count, sizeof(double) );
		if (glo->sat[i].eph == NULL)
			return 0;
	}
	glo->max_eph_count = max_eph_count;

	// observation types
	glo->obs_types = NULL;
	glo->obs_count = 0;

	return 1;
}

	// free gnss glonass memory
void pony_free_gnss_glo(pony_gnss_glo *glo)
{
	int i;

	if (glo == NULL)
		return;

	// satellites
	if (glo->sat != NULL) 
	{
		for (i = 0; i < glo->max_sat_count; i++) 
			// ephemeris
			if (glo->sat[i].eph != NULL)
			{
				free(glo->sat[i].eph);
				glo->sat[i].eph = NULL;
			}
	
		free(glo->sat);
		glo->sat = NULL;
	}
	if (glo->freq_slot != NULL) {
		free(glo->freq_slot);
		glo->freq_slot = NULL;
	}

	// gnss glonass structure
	free(glo);
}

	// initialize gnss galileo constants
void pony_init_gnss_gal_const(pony_gnss *gnss)
{
	gnss->gal_const.pi			=  3.1415926535898;		// pi as in Galileo OS SIS ICD Issue 1.2 (November 2015)
	gnss->gal_const.c			=  299792458;			// speed of light as in Galileo OS SIS ICD Issue 1.2 (November 2015), m/s
	gnss->gal_const.mu			=  3.986004418e14;		// Earth gravity constant as in Galileo OS SIS ICD Issue 1.2 (November 2015), m^3/s^2
	gnss->gal_const.u			=  7.2921151467e-5;		// Earth rotation rate as in Galileo OS SIS ICD Issue 1.2 (November 2015), rad/s
	gnss->gal_const.a			=  6378137.0;			// Earth ellipsoid semi-major axis as in GRS-80 // JoG March 2000 vol. 74 issue 1, m
	gnss->gal_const.e2			=  6.69438002290e-3;	// Earth ellipsoid first eccentricity squared as in GRS-80 // JoG March 2000 vol. 74 issue 1
	gnss->gal_const.F			= -4.442807309e-10;		// relativistic correction constant as in Galileo OS SIS ICD Issue 1.2 (November 2015), s/sqrt(m)
	gnss->gal_const.sec_in_w	= 604800;				// seconds in a week
	gnss->gal_const.sec_in_d	=  86400;				// seconds in a day
	gnss->gal_const.F1			=  1575.42e6;			// nominal frequency for E1 signal
	gnss->gal_const.L1			= gnss->gal_const.c/gnss->gal_const.F1;		// nominal wavelength for E1 signal
	gnss->gal_const.F5a			=  1176.45e6;			// nominal frequency for E5a signal
	gnss->gal_const.L5a			= gnss->gal_const.c/gnss->gal_const.F5a;	// nominal wavelength for E5a signal
	gnss->gal_const.F5b			=  1207.14e6;			// nominal frequency for E5b signal
	gnss->gal_const.L5b			= gnss->gal_const.c/gnss->gal_const.F5b;	// nominal wavelength for E5b signal
	gnss->gal_const.F6			=  1278.75e6;			// nominal frequency for E6 signal
	gnss->gal_const.L6			= gnss->gal_const.c/gnss->gal_const.F6;		// nominal wavelength for E6 signal
	
}

	// initialize gnss galileo structure
char pony_init_gnss_gal(pony_gnss_gal *gal, const int max_sat_count, const int max_eph_count)
{
	int i;

	gal->sat = NULL;
	gal->max_sat_count = 0;
	gal->max_eph_count = 0;

	// try to allocate memory for satellite data
	gal->sat = (pony_gnss_sat*)calloc( max_sat_count, sizeof(pony_gnss_sat) );
	if (gal->sat == NULL)
		return 0;
	gal->max_sat_count = max_sat_count;

	// initialize satellite data
	for (i = 0; i < gal->max_sat_count; i++) {
		gal->sat[i].eph			= NULL;
		gal->sat[i].eph_valid	= 0;
		gal->sat[i].obs			= NULL;
		gal->sat[i].obs_valid	= NULL;
		gal->sat[i].x_valid		= 0;
		gal->sat[i].v_valid		= 0;
		gal->sat[i].t_em_valid	= 0;
		gal->sat[i].sinEl_valid	= 0;
	}

	// try to allocate memory for each satellite ephemeris
	for (i = 0; i < gal->max_sat_count; i++) {
		gal->sat[i].eph = (double *)calloc( max_eph_count, sizeof(double) );
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

	// free gnss galileo memory
void pony_free_gnss_gal(pony_gnss_gal *gal)
{
	int i;

	if (gal == NULL)
		return;

	// satellites
	if (gal->sat != NULL) 
	{
		for (i = 0; i < gal->max_sat_count; i++)
			//ephemeris
			if (gal->sat[i].eph != NULL)
			{
				free(gal->sat[i].eph);
				gal->sat[i].eph = NULL;
			}
	
		free(gal->sat);
		gal->sat = NULL;
	}

	// gnss galileo structure
	free(gal);
}

	// initialize gnss structure
char pony_init_gnss(pony_gnss *gnss)
{
	// memory allocation limitations
	const int max_sat_count = 36;
	const int max_gps_eph_count = 36;
	const int max_glo_eph_count = 24;
	const int max_gal_eph_count = 36;

	int grouplen;
	char* groupptr;

	if (gnss == NULL)
		return 0;

	// gps
	pony_init_gnss_gps_const(gnss);
	gnss->gps = NULL;
	if ( pony_locatecfggroup("gps:", gnss->cfg, gnss->cfglength, &groupptr, &grouplen) )
	{
		gnss->gps = (pony_gnss_gps*)calloc( 1, sizeof(pony_gnss_gps) );
		if (gnss->gps == NULL)
			return 0;
		gnss->gps->cfg = groupptr;
		gnss->gps->cfglength = grouplen;

		if ( !pony_init_gnss_gps(gnss->gps, max_sat_count, max_gps_eph_count) )
			return 0;
	}

	// glonass
	pony_init_gnss_glo_const(gnss);
	gnss->glo = NULL;
	if (pony_locatecfggroup( "glo:", gnss->cfg, gnss->cfglength, &groupptr, &grouplen) )
	{
		gnss->glo = (pony_gnss_glo*)calloc( 1, sizeof(pony_gnss_glo) );
		if (gnss->glo == NULL)
			return 0;
		gnss->glo->cfg = groupptr;
		gnss->glo->cfglength = grouplen;

		if ( !pony_init_gnss_glo(gnss->glo, max_sat_count, max_glo_eph_count) )
			return 0;
	}

	// galileo
	pony_init_gnss_gal_const(gnss);
	gnss->gal = NULL;
	if ( pony_locatecfggroup("gal:", gnss->cfg, gnss->cfglength, &groupptr, &grouplen) )
	{
		gnss->gal = (pony_gnss_gal*)calloc( 1, sizeof(pony_gnss_gal) );
		if (gnss->gal == NULL)
			return 0;
		gnss->gal->cfg = groupptr;
		gnss->gal->cfglength = grouplen;

		if ( !pony_init_gnss_gal(gnss->gal, max_sat_count, max_gal_eph_count) )
			return 0;
	}

	// gnss settings
	pony_init_gnss_settings(gnss);

	// gnss solution
	pony_init_solution( &(gnss->sol) );

	// gnss epoch
	pony_init_epoch( &(gnss->epoch) );

	gnss->leap_sec = 0;
	gnss->leap_sec_valid = 0;

	return 1;
}

	// free gnss memory
void pony_free_gnss(pony_gnss *gnss)
{
	if (gnss == NULL)
		return;

	pony_free_gnss_gps(gnss->gps);
	gnss->gps = NULL;

	pony_free_gnss_glo(gnss->glo);
	gnss->glo = NULL;

	pony_free_gnss_glo(gnss->glo);
	gnss->glo = NULL;
}




// general handling routines
	// free all alocated memory and set pointers and counters to NULL
void pony_free()
{
	int i;

	// core
	if (pony.core.plugins != NULL)
		free(pony.core.plugins);
	pony.core.plugins = NULL;
	pony.core.plugin_count = 0;

	// configuration string
	if (pony.cfg != NULL)
		free(pony.cfg);
	pony.cfg = NULL;
	pony.cfglength = 0;

	// imu
	pony_free_imu();

	// gnss
	if (pony.gnss != NULL)
	{
		for (i = 0; i < pony.gnss_count; i++)
			pony_free_gnss( &(pony.gnss[i]) );
		free(pony.gnss);
		pony.gnss = NULL;
	}
	pony.gnss_count = 0;

}











///
///     Core pony functions for host application
///



// add plugin to the plugin execution list
//
// input: pointer to plugin function
//
// output: OK/not OK (1/0)
char pony_add_plugin( void(*newplugin)(void) )
{
	void(**reallocated_pointer)(void);

	reallocated_pointer = ( void(**)(void) )realloc( pony.core.plugins, (pony.core.plugin_count + 1) * sizeof( void(*)(void) ) );

	if (reallocated_pointer == NULL)	// failed to allocate/realocate memory
	{
		pony_free();
		return 0;
	}
	else
		pony.core.plugins = reallocated_pointer;

	pony.core.plugins[pony.core.plugin_count] = newplugin;
	pony.core.plugin_count++;

	return 1;
}


// initialize the bus, except for core
// 
// input: configuration string (see description)
//
// output: OK/not OK (1/0)
char pony_init(char* cfg)
{
	const int max_gnss_count = 10;
	
	char multi_gnss_token[] = "gnss[0]:";
	const int multi_gnss_index_position = 5;

	int grouplen;
	char* groupptr;

	pony_gnss *reallocated_pointer;

	int i;

	// determine configuration string length
	for (pony.cfglength = 0; cfg[pony.cfglength]; pony.cfglength++);

	// assign configuration string
	pony.cfg = (char *)malloc( sizeof(char) * (pony.cfglength + 1) );
	if (pony.cfg == NULL)
		return 0;
	for (i = 0; i < pony.cfglength; i++)
		pony.cfg[i] = cfg[i];
	pony.cfg[pony.cfglength] = '\0';

	// fetch a part of configuration that is outside of any group
	pony.cfg_settings = NULL;
	pony.settings_length = 0;
	pony_locatecfggroup("", pony.cfg, pony.cfglength, &pony.cfg_settings, &pony.settings_length);
	
	// imu init
	pony.imu = NULL;
	if ( pony_locatecfggroup("imu:", pony.cfg, pony.cfglength, &groupptr, &grouplen) ) // if the group found in configuration
	{
		// try to allocate memory
		pony.imu = (pony_imu*)calloc( 1, sizeof(pony_imu) );
		if (pony.imu == NULL) {
			pony_free();
			return 0;
		}

		// set configuration pointer
		pony.imu->cfg = groupptr;
		pony.imu->cfglength = grouplen;

		// try to init
		if ( !pony_init_imu() ) {
			pony_free();
			return 0;
		}
	}

	// gnss init
	pony.gnss = NULL;
	pony.gnss_count = 0;
		// multiple gnss mode support
	for (i = 0; i < max_gnss_count; i++)
	{
		multi_gnss_token[multi_gnss_index_position] = '0' + (char)i;

		if ( (i == 0 && pony_locatecfggroup("gnss:", pony.cfg, pony.cfglength, &groupptr, &grouplen) ) ||
			pony_locatecfggroup(multi_gnss_token, pony.cfg, pony.cfglength, &groupptr, &grouplen) ) {
			if (i >= pony.gnss_count) {
				// try to allocate/reallocate memory
				reallocated_pointer = (pony_gnss*)realloc(pony.gnss, sizeof(pony_gnss)*(i+1));
				if (reallocated_pointer == NULL) {
					pony_free();
					return 0;
				}
				else
					pony.gnss = reallocated_pointer;
				// try to init new instances, except the i-th one
				for ( ; pony.gnss_count < i; pony.gnss_count++) {
					pony.gnss[pony.gnss_count].cfg = NULL;
					pony.gnss[pony.gnss_count].cfglength = 0;
					pony_init_gnss( &(pony.gnss[pony.gnss_count]) );
				}
				pony.gnss_count = i+1;
			}

			// set configuration pointer
			pony.gnss[i].cfg = groupptr;
			pony.gnss[i].cfglength = grouplen;

			// try to init
			if ( !pony_init_gnss( &(pony.gnss[i]) ) ) {
				pony_free();
				return 0;
			}
		}
	}

	// system time, operation mode and solution
	pony.t = 0;
	pony.mode = 0;
	pony_init_solution( &(pony.sol) );
	
	return 1;
}




// step through the plugin execution list, to be called by host application in a main loop
//
// output: OK/not OK (1/0)
char pony_step(void)
{
	int i;

	// loop through plugin execution list
	for (i = 0; i < pony.core.plugin_count; i++)
	{
		pony.core.plugins[i](); // execute the current plugin

		if (pony.core.exit_plugin_id == i)	// if termination was initiated by the current plugin on the previous loop
		{
			pony.core.exit_plugin_id = -1;		// set to default
			pony.core.host_termination = 0;		// set to default
			pony_init_solution(&(pony.sol));	// drop the solution
			pony_free();						// free memory
			break;
		}

		if ((pony.mode < 0 || pony.core.host_termination == 1) && pony.core.exit_plugin_id == -1)	// if termination was initiated by the current plugin on the current loop
		{
			if (pony.mode > 0)
				pony.mode = -1;					// set mode to -1 for external termination cases

			if (pony.mode != 0)
				pony.core.exit_plugin_id = i;	// set the index to use in the next loop

		}
	}

	if (pony.mode == 0)		// if initialization ended
		pony.mode = 1;		// set operation mode to regular

							// success if either staying in regular operation mode, or a termination properly detected
	return (pony.mode >= 0) || (pony.core.exit_plugin_id >= 0);
}

// terminate operation
//
// output: OK/not OK (1/0)
char pony_terminate()
{
	if (pony.mode >= 0 && pony.core.host_termination != 1)
	{
		pony.core.host_termination = 1;

		return 1; // termination started
	}

	return 0; // had been already terminated by host or by plugin 
}











// linear algebra functions
	// conventional operations
		// dot product
double pony_linal_dot(double *u, double *v, const int m) {
	double res;
	int i;

	res = u[0]*v[0];
	for (i = 1; i < m; i++)
		res += u[i]*v[i];

	return res;
}

		// matrix multiplication res = a*b, a is n x n1, b is n1 x m, res is n x m
void pony_linal_mmul(double *res,  double *a, double *b, const int n, const int n1, const int m) {
	int i, j, k, k0, ka, kb, p;

	for (i = 0, k = 0, k0 = 0; i < n; i++, k0 += n1)
		for (j = 0; j < m; j++, k++) {
			ka = k0;
			kb = j;
			res[k] = a[ka]*b[kb];
			for (ka++, kb += m, p = 1; p < n1; ka++, kb += m, p++)
				res[k] += a[ka]*b[kb];
		}
}

		// matrix multiplication with the second argument transposed res = a*b^T, a is n x m, b is n1 x m, res is n x n1
void pony_linal_mmul2T(double *res,  double *a, double *b, const int n, const int m, const int n1) {
	int i, j, k, k0, ka, kb, p;

	for (i = 0, k = 0, k0 = 0; i < n; i++, k0 += m)
		for (j = 0; j < n1; j++, k++) {
			ka = k0;
			kb = j*m;
			res[k] = a[ka]*b[kb];
			for (ka++, kb++, p = 1; p < m; ka++, kb++, p++)
				res[k] += a[ka]*b[kb];
		}
}

	// routines for m x m upper-triangular matrices lined up in a single-dimension array
		// index conversion for upper-triangular matrix lined up in a single-dimension array: (i,j) -> k
void pony_linal_u_ij2k(int *k, const int i, const int j, const int m) {
	*k = ( i*(2*m - 1 - i) )/2 + j;
}

		// index conversion for upper-triangular matrix lined up in a single-dimension array: k -> (i,j)
void pony_linal_u_k2ij(int *i, int *j, const int k, const int m) {
	double onehalf_plus_m = 0.5+m;
	*i = (int)(floor( onehalf_plus_m - sqrt(onehalf_plus_m*onehalf_plus_m - 2.0*k) ) + 0.5);
	*j = k - ( ( 2*m - 1 - (*i) )*(*i) )/2;
}

		// upper-triangular matrix lined up in a single-dimension array multiplication: res = U*v
			// overwriting input (double *res = double *v) allowed
void pony_linal_u_mul(double *res, double *u, double *v, const int n, const int m) {
	int i, j, k, p, p0, mn;

	mn = m*n;
	for (j = 0; j < m; j++)
		for (i = 0, k = 0, p0 = j; i < n; i++, p0 += m) {
			res[p0] = u[k]*v[p0];
			for (p = p0+m, k++; p < mn; p += m, k++)
				res[p0] += u[k]*v[p];
		}
}

		// upper-triangular matrix lined up in a single-dimension array transposed multiplication by vector: res = U^T*v
void pony_linal_uT_mul_v(double *res, double *u, double *v, const int m) {
	int i, j, k;

	for (i = 0; i < m; i++)
		res[i] = u[i]*v[0];
	for (j = 1, k = m; j < m; j++)
		for (i = j; i < m; i++, k++)
			res[i] += u[k]*v[j];
}

		// inversion of upper-triangular matrix lined up in a single-dimension array: res = U^-1
			// overwriting input (double *res = double *u) allowed
void pony_linal_u_inv(double *res, double *u, const int m) {

	int i, j, k, k0, p, q, p0, r;
	double s;

	for (j = 0, k0 = (m+2)*(m-1)/2; j < m; k0 -= j+2, j++) {
		res[k0] = 1/u[k0];
		for (i = j+1, k = k0-j-1; i < m; k -= i+1, i++) {
			p0 = k-(i-j);
			for (p = k, q = k0, r = 0, s = 0; q > k; p--, q -= j+r+1, r++)
				s += u[p]*res[q];
			res[k] = -s/u[p0];
		}
	}

}

		// square (with transposition) of upper-triangular matrix lined up in a single-dimension array: res = U U^T
			// overwriting input (double *res = double *u) allowed
void pony_linal_uuT(double *res, double *u, const int m) {

	int i, j, k, p, q, r;

	for (i = 0, k = 0; i < m; i++)
		for (j = i; j < m; j++, k++) {
			pony_linal_u_ij2k(&p, j,j, m);
			res[k] = u[k]*u[p];
			for (q = k+1, p++, r = j+1; r < m; q++, p++, r++)
				res[k] += u[q]*u[p];
		}

}

	// Cholesky upper-triangular factorization P = S*S^T, where P is symmetric positive-definite matrix
		// input:	P - upper-triangular part of symmetric m-by-m positive-definite R lined in a single-dimension array
		// output:	S - upper-triangular part of a Cholesky factor S lined in a single-dimension array
		// overwriting input (double *P == double *S) allowed
void pony_linal_chol(double *S, double *P, const int m) {

	int i, j, k, k0, p, q, p0;
	double s;

	for (j = 0, k0 = (m+2)*(m-1)/2; j < m; k0 -= j+2, j++) {
		p0 = k0+j;
		for (p = k0+1, s = 0; p <= p0; p++)
			s += S[p]*S[p];
		S[k0] = sqrt(P[k0] - s);
		for (i = j+1, k = k0-j-1; i < m; k -= i+1, i++) {
			for (p = k0+1, q = k+1, s = 0; p <= p0; p++, q++)
				s += S[p]*S[q];
			S[k] = (S[k0] == 0)? 0 : (P[k] - s)/S[k0];
		}
	}

}

	// square root Kalman filtering
double pony_linal_kalman_update(double *x, double *S, double *K, double z, double *h, double sigma, const int m) {

	double d, d1, sdd1, f, e;
	int i, j, k;

	// e0 stored in K
	for (i = 0; i < m; i++)
		K[i] = 0;

	// d0
	d = sigma*sigma;

	// S
	for (i = 0; i < m; i++) {
		// f = S^T*h
		f = S[i]*h[0];
		for (j = 1, k = i+m-1; j <= i; j++, k += m-j)
			f += S[k]*h[j];
		// d
		d1 = d + f*f;
		sdd1 = sqrt(d*d1);
		// S^+, e
		for (j = 0, k = i; j <= i; j++, k += m-j) {
			e = K[j];
			K[j] += S[k]*f;
			S[k] = (S[k]*d - e*f)/sdd1;
		}
		d = d1;
	}

	// dz
	for (i = 0; i < m; i++)
		z -= h[i]*x[i];

	// K, x
	for (i = 0; i < m; i++) {
		K[i] /= d;
		x[i] += K[i]*z;
	}

	return z;
}
