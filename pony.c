// Jul-2019
//

// PONY core source code

#include <stdlib.h>

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
	{NULL, 0, -1, NULL, 0}};	// core.plugins, core.plugin_count, core.exit_plugin_id, core.cfg, core.cfglength







// service subroutines

	// compare substrings of two strings up to a given length
	//
	// input: pointers to strings s1, s2, substring length
	// 
	// output: 0 - equal, 1 - difference detected
char pony_strncmpeff(char* s1, char* s2, int substrlen)
{
	int i;
	for (i = 0; i < substrlen; i++)
		if (s1[i] != s2[i])
			return 1;
	return 0;
}

	// locate a substring within a string given theirs lengths
	// 
	// input: str - source string, len - source string length, susbtr - substring to locate, substrlen - susbtring length
	// 
	// output: pointer to the first occurence of substring within the source string
char* pony_locatesubstrn(char* str, int len, char* substr, int substrlen)
{
	int n = 0;
	while (n + substrlen <= len )
	{
		if (pony_strncmpeff(str + n, substr, substrlen) == 0)
		{
			return str + n;
		}
		n++;
	}
	return NULL;
}


	// locate a substring within a configuration group
	// 
	// input: str - pointer to a string containing the configuration group, substr - substring to locate, substrlen - substring length
	// 
	// output: pointer to the first occurence of substring within the configuration group
char* pony_locatesubstreff(char* str, char* substr, int substrlen)  
{
	char* res = str;

	int in = 0;

	while ( (*res) != '\0' )
	{
		if (in == 0 && pony_strncmpeff(res, substr, substrlen) == 0)
		{
			return res;
		}
		if (res[0] == '{')
		{
			in++;
		}
		if (res[0] == '}')
		{
			in--;
		}

		res++;
	}
	return NULL;
}




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

	// locate configuration substring that is outside of any group
	if (groupname[0] == '\0') {
		for (i = 0; cfgstr[i] && i < cfglen; i++) {
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
		for (; cfgstr[i] && (cfgstr[i] <= ' ' || cfgstr[i] == ',') && i < cfglen; i++);
		// start from this point
		*groupptr = cfgstr + i;

		// determine the length, counting until the end of the string or when a group started or ended
		for (; (*groupptr)[*grouplen] && (*groupptr)[*grouplen] != '{' && (*groupptr)[*grouplen] != '}'; (*grouplen)++);
	}

	// locate configuration substring inside a requested group
	else {
		for (i = 0; cfgstr[i] && !group_found && i < cfglen; i++) {
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
	pony_locatecfggroup( "", gnss->cfg, gnss->cfglength, &(gnss->settings_cfg), &(gnss->settings_length) );

	gnss->settings.sinEl_mask = 0;

	return 1;
}

	// initialize gnss gps constants
void pony_init_gnss_gps_const(void)
{
	int r;

	for (r = 0; r < pony.gnss_count; r++) {
	pony.gnss[r].gps_const.pi = 3.1415926535898;	// pi as in IS-GPS-200J (22 May 2018)
	pony.gnss[r].gps_const.c = 299792458;			// speed of light as in IS-GPS-200J (22 May 2018), m/s
	pony.gnss[r].gps_const.mu = 3.986005e14;		// Earth grav constant as in IS-GPS-200J (22 May 2018), m^3/s^2
	pony.gnss[r].gps_const.u = 7.2921151467e-5;	// Earth rotation rate as in IS-GPS-200J (22 May 2018), rad/s
	pony.gnss[r].gps_const.a = 6378137.0;			// Earth ellipsoid semi-major axis as in WGS-84(G1762) 2014-07-08, m
	pony.gnss[r].gps_const.F = -4.442807633e-10;	// relativistic correction constant as in IS-GPS-200J (22 May 2018), s/sqrt(m)
	pony.gnss[r].gps_const.sec_in_w = 604800;		// seconds in a week
	pony.gnss[r].gps_const.sec_in_d =  86400;		// seconds in a day
	}
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
			if (gps->sat[i].eph != NULL)
			{
				free(gps->sat[i].eph);
				gps->sat[i].eph = NULL;
			}
	
		free(gps->sat);
	}
	gps->sat = NULL;

	// gnss_gps structure
	free(gps);
}


	// initialize gnss glo structure
char pony_init_gnss_glo(pony_gnss_glo *glo, const int max_sat_count, const int max_eph_count)
{
	int i;

	glo->sat = NULL;
	glo->max_sat_count = 0;
	glo->max_eph_count = 0;

	// try to allocate memory for satellite data
	glo->sat = (pony_gnss_sat*)calloc( max_sat_count, sizeof(pony_gnss_sat) );
	if (glo->sat == NULL)
		return 0;
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
			if (glo->sat[i].eph != NULL)
			{
				free(glo->sat[i].eph);
				glo->sat[i].eph = NULL;
			}
	
		free(glo->sat);
	}
	glo->sat = NULL;

	// gnss_gps structure
	free(glo);
}

	// initialize gnss structure
char pony_init_gnss(pony_gnss *gnss)
{
	// memory allocation limitations
	const int max_sat_count = 36;
	const int max_gps_eph_count = 36;
	const int max_glo_eph_count = 24;

	int grouplen;
	char* groupptr;

	if (gnss == NULL)
		return 0;

	// gps
	pony_init_gnss_gps_const();
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

	// gnss settings
	pony_init_gnss_settings(gnss);

	// gnss solution
	pony_init_solution( &(gnss->sol) );

	// gnss epoch
	pony_init_epoch( &(gnss->epoch) );

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
	pony.core.plugins = ( void(**)(void) )realloc( pony.core.plugins, (pony.core.plugin_count + 1) * sizeof( void(*)(void) ) );

	if (pony.core.plugins == NULL)	// failed to allocate/realocate memory
	{
		pony_free();
		return 0;
	}

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
	pony_locatecfggroup("", pony.cfg, pony.cfglength, &pony.core.cfg, &pony.core.cfglength);
	
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
			pony_locatecfggroup(multi_gnss_token, pony.cfg, pony.cfglength, &groupptr, &grouplen) )
			while (i >= pony.gnss_count) 
			{
				pony.gnss_count++;
				// try to allocate/reallocate memory
				pony.gnss = (pony_gnss*)realloc(pony.gnss, sizeof(pony_gnss)*pony.gnss_count);
				if (pony.gnss == NULL) {
					pony_free();
					return 0;
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
			pony_init_solution( &(pony.sol) );	// drop the solution
			pony_free();						// free memory
			break;
		}

		if (pony.mode < 0 && pony.core.exit_plugin_id == -1)	// if termination was initiated by the current plugin on the current loop
			pony.core.exit_plugin_id = i;							// set the index to use in the next loop
	}

	if (pony.mode == 0)		// if initialization ended
		pony.mode = 1;			// set operation mode to regular

	// success if either staying in regular operation mode, or a termination by a plugin properly detected
	return (pony.mode >= 0) || (pony.core.exit_plugin_id >= 0);
}

// terminate operation
char pony_terminate()
{
	int i;

	// step through plugin execution list with mode switched to termination
	pony.mode = -1;
	for (i = 0; i < pony.core.plugin_count; i++)
		pony.core.plugins[i]();

	pony_init_solution( &(pony.sol) );	// drop the solution
	pony_free();						// free memory

	return 1; // always succeed in termination
}











// if using linear algebra functions
#ifdef PONY_LINAL
#include <math.h>

	// conventional operations
		// dot product
double pony_linal_dot(double *u, double *v, const int m) {
	double res = 0;
	int i;

	for (i = 0; i < m; i++)
		res += u[i]*v[i];

	return res;
}

	// routines for m x m upper-triangular matrices lined up in a single-dimension array
		// index conversion: (i,j) -> k
void pony_linal_u_ij2k(int *k, const int i, const int j, const int m) {
	*k = ( i*(2*m - 1 - i) )/2 + j;
}

		// index conversion: k -> (i,j)
void pony_linal_u_k2ij(int *i, int *j, const int k, const int m) {
	double onehalf_plus_m = 0.5+m;
	*i = (int)(floor( onehalf_plus_m - sqrt(onehalf_plus_m*onehalf_plus_m - 2.0*k) ) + 0.5);
	*j = k - ( ( 2*m - 1 - (*i) )*(*i) )/2;
}

		// matrix multiplication by vector: res = U*v
void pony_linal_u_mul_v(double *res, double *u, double *v, const int m) {
	int i, j, k;

	for (i = 0, k = 0; i < m; i++) {
		res[i] = u[k]*v[i];
		for (j = i+1, k++; j < m; j++, k++)
			res[i] += u[k]*v[j];
	}
}

		// transposed matrix multiplication by vector: res = U^T*v
void pony_linal_uT_mul_v(double *res, double *u, double *v, const int m) {
	int i, j, k;

	for (i = 0; i < m; i++)
		res[i] = u[i]*v[0];
	for (j = 1, k = m; j < m; j++)
		for (i = j; i < m; i++, k++)
			res[i] += u[k]*v[j];
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

#endif
