// Sep-2020
//
/*	pony_gnss_sol 
	
	pony plugins that provide GNSS navigation solutions:

	- pony_gnss_sol_pos_code
		Computes conventional least-squares standalone position and clock error from code pseudoranges 
		for all available receivers/antennas.
	- pony_gnss_sol_check_elevation_mask
		Checks if satellites are below elevation mask and drop their measurement validity flags if the case.
		Multi-receiver/multi-system capable. Stores separate value for each receiver in gnss->settings structure.
	- pony_gnss_sol_select_observables
		Sets validity flags for specific observables according to templates in configuration.
		Template set 'obs_use' enumerates the exclusive selection of observable types to stay valid.
		Template set 'obs_off' lists observable types subject to exclusion from further calculations.
*/

#include <stdlib.h>
#include <math.h>

#include "../pony.h"

// pony bus version check
#define PONY_GNSS_SOL_BUS_VERSION_REQUIRED 7
#if PONY_BUS_VERSION < PONY_GNSS_SOL_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif

// internal defines
#define PONY_GNSS_SOL_POS_PRECISION (1.0/0x3000) // approx 10^-4, meters
#define PONY_GNSS_SOL_POS_MAX_ITERATIONS 32
#define PONY_GNSS_SOL_MAX_COORD 10e6             // meters

// internal routines
	// gnss calculations
void   pony_gnss_sol_pos_code_single_receiver(pony_gnss *gnss, double *x, double *S, double *h, double *K, const size_t m, const size_t maxk);
	// models
double pony_gnss_sol_noise_coeff_from_elev(double sinEl);                                  // uniform atmospheric layer, pathlength-proportional noise level
char   pony_gnss_sol_ecef2llh(double *ecef, double *llh, const double a, const double e2); // ecef cartesian to longitude-latitude-height
	// observable codes
size_t pony_gnss_sol_select_observables_read_wildcards(char *dest,  char *src, const size_t src_len, const char *token);
char   pony_gnss_sol_select_observables_match_wildcard(char *obstype, char *wildcard);
	// memory handling
char   pony_gnss_sol_alloc_ppointer_gnss_count(double ***ptr, const size_t internal_size);
void   pony_gnss_sol_free_ppointer_gnss_count(void ***ptr);
void   pony_gnss_sol_free_null(void **ptr);                                                // free memory with NULL-check and NULL-assignment














// plugin definitions

/* pony_gnss_sol_pos_code - pony plugin
	
	Computes conventional least-squares standalone position and clock error from code pseudoranges 
	for all available receivers/antennas. 

	description:
		- sets sol.x_valid flag to the number of valid measurements used in position estimate;
		- estimates receiver clock error for each satellite constellation separately, 
		  then puts average value into sol.dt;
		- calculates both ECEF and geographical coordinates.
	uses:
		pony->gnss_count
		pony->gnss[].sol.x
		pony->gnss[].settings.sinEl_mask
		pony->gnss[].settings.code_sigma
		pony->gnss[].gps/glo/gal/bds->max_sat_count
		pony->gnss[].gps/glo/gal/bds->    obs_count
		pony->gnss[].gps/glo/gal/bds->    obs_types
		pony->gnss[].gps/glo/gal/bds->sat[].Deltatsv
		pony->gnss[].gps/glo/gal/bds->sat[].    x
		pony->gnss[].gps/glo/gal/bds->sat[].    x_valid
		pony->gnss[].gps/glo/gal/bds->sat[].  obs
		pony->gnss[].gps/glo/gal/bds->sat[].  obs_valid
		pony->gnss[].gps/glo/gal/bds->sat[].sinEl
		pony->gnss[].gps/glo/gal/bds->sat[].sinEl_valid
	changes:
		pony->gnss[].sol.  x
		pony->gnss[].sol.  x_valid
		pony->gnss[].sol. dt
		pony->gnss[].sol. dt_valid
		pony->gnss[].sol.llh
		pony->gnss[].sol.llh_valid
	cfg parameters:
		none
*/
void pony_gnss_sol_pos_code(void) {

	const size_t sys_count = 4;    // gps, glonass, galileo and beidou supported
	const size_t m = 3+sys_count, maxk = (m*m+m)/2;

	static double 
		**x = NULL, **S = NULL,	// least squares and covariance square root matrix
		**h = NULL, **K = NULL;	// current observation coefficients in measurement model and Kalman gain

	size_t i, k, r;

	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return;

	// init
	if (pony->mode == 0) {

		// allocate memory
		if (   !pony_gnss_sol_alloc_ppointer_gnss_count(&x,m)
			|| !pony_gnss_sol_alloc_ppointer_gnss_count(&S,maxk)
			|| !pony_gnss_sol_alloc_ppointer_gnss_count(&h,m)
			|| !pony_gnss_sol_alloc_ppointer_gnss_count(&K,m) ) {
			pony->mode = -1;
			return;
		}

		for (r = 0; r < pony->gnss_count; r++)
			if (pony->gnss[r].cfg == NULL)
				continue;
			else 
				for (i = 0; i < m; i++) {
					pony_linal_u_ij2k(&k, i, i, m);
					S[r][k] = PONY_GNSS_SOL_MAX_COORD; // initial covariance for solution
				}

	}

	// terminate
	if (pony->mode < 0) {

		// free memory
		pony_gnss_sol_free_ppointer_gnss_count( (void ***)(&x) );
		pony_gnss_sol_free_ppointer_gnss_count( (void ***)(&S) );
		pony_gnss_sol_free_ppointer_gnss_count( (void ***)(&h) );
		pony_gnss_sol_free_ppointer_gnss_count( (void ***)(&K) );

		return;

	}

	// regular processing
	else {
		
		// run through all receivers
		for (r = 0; r < pony->gnss_count; r++)
			if (pony->gnss[r].cfg == NULL)
				continue;
			else
				pony_gnss_sol_pos_code_single_receiver(&(pony->gnss[r]), x[r], S[r], h[r], K[r],m,maxk);

	}

}

/* pony_gnss_sol_check_elevation_mask - pony plugin
	
	Checks if satellites are below elevation mask and drop their measurement validity flags if the case.
	Multi-receiver/multi-system capable. Stores separate value for each receiver in gnss->settings structure.

	description:
		none
	uses:
		pony->gnss_count
		pony->gnss[].gps/glo/gal/bds->max_sat_count
		pony->gnss[].gps/glo/gal/bds->    obs_count
		pony->gnss[].gps/glo/gal/bds->sat[].sinEl
		pony->gnss[].gps/glo/gal/bds->sat[].sinEl_valid
	changes:
		pony->gnss[].settings.sinEl_mask
		pony->gnss[].gps/glo/gal/bds->sat[].  obs_valid
	cfg parameters:
		{gnss: elev_mask} - gnss satellite elevation mask, degrees
			type   : floating point
			range  : -90 to +90
			default: +5
			example: {gnss: elev_mask = 10}

*/
void pony_gnss_sol_check_elevation_mask(void) {

	enum system_id {gps, glo, gal, bds, sys_count};

	const char   elev_mask_token[] = "elev_mask"; // elevetion mask parameter name in configuration
	const double elev_mask_default = 5;           // elevation mask default value, degrees

	char *cfg_ptr;                                // pointer to a substring in configuration
	size_t r, sys, s, i, maxsat, maxobs;
	pony_gnss_sat *sat;

	// requires gnss structure initialized
	if (pony->gnss == NULL)
		return;

	// init
	if (pony->mode == 0) {
		
		for (r = 0; r < pony->gnss_count; r++)
			if (pony->gnss[r].cfg != NULL) {
				// parse alignment duration from configuration string
				cfg_ptr = pony_locate_token(elev_mask_token, pony->gnss[r].cfg_settings, pony->gnss[r].settings_length, '=');
				if (cfg_ptr != NULL)
					pony->gnss[r].settings.sinEl_mask = atof(cfg_ptr);		
				// if not found or invalid, set to default
				if (cfg_ptr == NULL || -90 < pony->gnss[r].settings.sinEl_mask || pony->gnss[r].settings.sinEl_mask < 90)
					pony->gnss[r].settings.sinEl_mask = elev_mask_default;
				// convert to sine
				pony->gnss[r].settings.sinEl_mask = sin(pony->gnss[r].settings.sinEl_mask*pony->gnss_const.pi/180);
			}

	}

	// terminate
	if (pony->mode < 0) {
		// nothing to do here
	}

	// regular processing
	else {
		
		for (r = 0; r < pony->gnss_count; r++)
			if (pony->gnss[r].cfg == NULL)
				continue;
			else
				for (sys = 0; sys < sys_count; sys++) {
					maxsat = 0;
					maxobs = 0;
					sat = NULL;
					switch (sys) {
						case gps:
							if (pony->gnss[r].gps == NULL)
								continue;
							maxsat	= pony->gnss[r].gps->max_sat_count;
							maxobs	= pony->gnss[r].gps->obs_count;
							sat		= pony->gnss[r].gps->sat;
							break;
						case glo:
							if (pony->gnss[r].glo == NULL)
								continue;
							maxsat	= pony->gnss[r].glo->max_sat_count;
							maxobs	= pony->gnss[r].glo->obs_count;
							sat		= pony->gnss[r].glo->sat;
							break;
						case gal:
							if (pony->gnss[r].gal == NULL)
								continue;
							maxsat	= pony->gnss[r].gal->max_sat_count;
							maxobs	= pony->gnss[r].gal->obs_count;
							sat		= pony->gnss[r].gal->sat;
							break;
						case bds:
							if (pony->gnss[r].bds == NULL)
								continue;
							maxsat	= pony->gnss[r].bds->max_sat_count;
							maxobs	= pony->gnss[r].bds->obs_count;
							sat		= pony->gnss[r].bds->sat;
							break;
						default:
							continue;
					}
					for (s = 0; s < maxsat; s++)
						if (sat[s].sinEl_valid && sat[s].sinEl < pony->gnss[r].settings.sinEl_mask)
							for (i = 0; i < maxobs; i++) // drop observation flags if elevation is below the mask
								sat[s].obs_valid[i] = 0;
				}
	}

}

/* pony_gnss_sol_select_observables - pony plugin
	
	Sets validity flags for specific observables according to templates in configuration.
	Template set 'obs_use' enumerates the exclusive selection of observable types to stay valid.
	Template set 'obs_off' lists observable types subject to exclusion from further calculations.

	description:
		Designation of observable types must comply with RINEX specifications, case-sensitive, with e.g.
		- C1C for code pseudoranges on L1 frequency, C/A channel, 
		- L2X for carrier phase on L2 frequency, codeless, 
		- ... etc. 
		See Section 5.1 Observation codes of RINEX v3.02, or similar sections in other versions.
		'?' (question mark) for wildcard (any character).
	uses:
		pony->gnss_count
		pony->gnss[].gps/glo/gal/bds->max_sat_count
		pony->gnss[].gps/glo/gal/bds->    obs_count
	changes:
		pony->gnss[].gps/glo/gal/bds->sat[].  obs_valid
	cfg parameters:
		1. [optional]
		{gnss:       obs_use } - the exclusive selection of all     observable types to stay valid
		and/or
		{gnss: {gps: obs_use}} -                      --"-- gps     --"--
		and/or
		{gnss: {glo: obs_use}} -                      --"-- glonass --"--
		and/or
		{gnss: {gal: obs_use}} -                      --"-- galileo --"--
		and/or
		{gnss: {bds: obs_use}} -                      --"-- beidou  --"--
			type   : square brackets-enclosed, comma-separated list of three-character templates
			range  : as of RINEX specification
			default: none
			example: {gnss:       obs_use = [C??, L??] } - use only code pseudorange and carrier phase
			         {gnss: {gps: obs_use = [?1?]      } - use only L1 band in gps
		2. [optional]
		{gnss:       obs_off } -         observable types subject to exclusion from further calculations
		and/or
		{gnss: {gps: obs_off}} - gps     --"--
		and/or
		{gnss: {glo: obs_off}} - glonass --"--
		and/or
		{gnss: {gal: obs_off}} - galileo --"--
		and/or
		{gnss: {bds: obs_off}} - beidou  --"--
			type   : square brackets-enclosed, comma-separated list of three-character templates
			range  : as of RINEX specification
			default: none
			example: {gnss:       obs_off = [C2?, L1C] } - exclude code pseudoranges on L2 and C/A carrier phase on L1
			         {gnss: {glo: obs_off = [??X]      } - exclude all codeless glonass measurements
*/
void pony_gnss_sol_select_observables(void) {

	enum system_id {gps, glo, gal, bds, sys_count};
	const char obs_use_token[] = "obs_use", obs_off_token[] = "obs_off";
	const size_t wildcard_len = 3;

	static char ***obs_use_wildcards = NULL, ***obs_off_wildcards = NULL;
	static size_t **obs_use_count = NULL, **obs_off_count = NULL;

	size_t i, j, r, sys, s, cfg_len, maxsat, maxobs, common_use, common_off;
	char *cfg, (*obstypes)[4];
	pony_gnss_sat *sat;

	// requires gnss initialized
	if (pony->gnss == NULL) 
		return;

	// init
	if (pony->mode == 0) {

		// allocate memory
		obs_use_wildcards	= (char  ***)calloc( pony->gnss_count, sizeof(char  **) );
		obs_off_wildcards	= (char  ***)calloc( pony->gnss_count, sizeof(char  **) );
		obs_use_count		= (size_t **)calloc( pony->gnss_count, sizeof(size_t *) );
		obs_off_count		= (size_t **)calloc( pony->gnss_count, sizeof(size_t *) );
		if (   obs_use_wildcards	== NULL 
			|| obs_off_wildcards	== NULL 
			|| obs_use_count		== NULL 
			|| obs_off_count		== NULL) {
			pony->mode = -1;
			return;
		}

		for (r = 0; r < pony->gnss_count; r++) { // go for all receivers
			if (pony->gnss[r].cfg == NULL)
				continue;
			obs_use_wildcards[r] = (char  **)calloc( sys_count, sizeof(char *) );
			obs_off_wildcards[r] = (char  **)calloc( sys_count, sizeof(char *) );
			obs_use_count    [r] = (size_t *)calloc( sys_count, sizeof(size_t) );
			obs_off_count    [r] = (size_t *)calloc( sys_count, sizeof(size_t) );
			if (   obs_use_wildcards[r] == NULL 
				|| obs_off_wildcards[r] == NULL 
				|| obs_use_count	[r] == NULL 
				|| obs_off_count	[r] == NULL) {
				pony->mode = -1;
				return;
			}

			cfg		= pony->gnss[r].cfg_settings;
			cfg_len	= pony->gnss[r].settings_length;
			common_use = pony_gnss_sol_select_observables_read_wildcards(NULL, cfg, cfg_len, obs_use_token);
			common_off = pony_gnss_sol_select_observables_read_wildcards(NULL, cfg, cfg_len, obs_off_token);

			for (sys = 0; sys < sys_count; sys++) { // go for all systems and the common configuration string
				switch (sys) {
					case gps:
						if (pony->gnss[r].gps == NULL)
							continue;
						cfg		= pony->gnss[r].gps->cfg;
						cfg_len	= pony->gnss[r].gps->cfglength;
						break;
					case glo:
						if (pony->gnss[r].glo == NULL)
							continue;
						cfg		= pony->gnss[r].glo->cfg;
						cfg_len	= pony->gnss[r].glo->cfglength;
						break;
					case gal:
						if (pony->gnss[r].gal == NULL)
							continue;
						cfg		= pony->gnss[r].gal->cfg;
						cfg_len	= pony->gnss[r].gal->cfglength;
						break;
					case bds:
						if (pony->gnss[r].bds == NULL)
							continue;
						cfg		= pony->gnss[r].bds->cfg;
						cfg_len	= pony->gnss[r].bds->cfglength;
						break;
					default:
						continue;
				}
				// count number of wildcards first
				obs_use_count[r][sys] = common_use + pony_gnss_sol_select_observables_read_wildcards(NULL, cfg, cfg_len, obs_use_token);
				obs_off_count[r][sys] = common_off + pony_gnss_sol_select_observables_read_wildcards(NULL, cfg, cfg_len, obs_off_token);

				obs_use_wildcards[r][sys] = (char *)calloc( obs_use_count[r][sys]*wildcard_len+1, sizeof(char) );
				obs_off_wildcards[r][sys] = (char *)calloc( obs_off_count[r][sys]*wildcard_len+1, sizeof(char) );
				if (   obs_use_wildcards[r][sys] == NULL 
					|| obs_off_wildcards[r][sys] == NULL) {
					pony->mode = -1;
					return;
				}
				// actually read wildcards from the configuration for a specific constellation
				pony_gnss_sol_select_observables_read_wildcards(obs_use_wildcards[r][sys], cfg, cfg_len, obs_use_token);
				pony_gnss_sol_select_observables_read_wildcards(obs_off_wildcards[r][sys], cfg, cfg_len, obs_off_token);
				// add from the common configuration
				cfg		= pony->gnss[r].cfg_settings;
				cfg_len	= pony->gnss[r].settings_length;
				s = obs_use_count[r][sys] - common_use;
				pony_gnss_sol_select_observables_read_wildcards(obs_use_wildcards[r][sys] + s*wildcard_len, cfg, cfg_len, obs_use_token);
				s = obs_off_count[r][sys] - common_off;
				pony_gnss_sol_select_observables_read_wildcards(obs_off_wildcards[r][sys] + s*wildcard_len, cfg, cfg_len, obs_off_token);
			}
		}

	}

	// terminate
	if (pony->mode < 0) {
		
		// free memory
		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].cfg == NULL)
				continue;
			for (sys = 0; sys < sys_count; sys++) {
				if (obs_use_wildcards != NULL && obs_use_wildcards[r] != NULL)
					pony_gnss_sol_free_null( (void **)( &(obs_use_wildcards[r][sys]) ) );
				if (obs_off_wildcards != NULL && obs_off_wildcards[r] != NULL)
					pony_gnss_sol_free_null( (void **)( &(obs_off_wildcards[r][sys]) ) );
			}
			if (obs_use_wildcards != NULL)
				pony_gnss_sol_free_null( (void **)( &(obs_use_wildcards[r]) ) );
			if (obs_off_wildcards != NULL)
				pony_gnss_sol_free_null( (void **)( &(obs_off_wildcards[r]) ) );
			if (obs_use_count != NULL)
				pony_gnss_sol_free_null( (void **)( &(obs_use_count    [r]) ) );
			if (obs_off_count != NULL)
				pony_gnss_sol_free_null( (void **)( &(obs_off_count    [r]) ) );
		}
		pony_gnss_sol_free_null( (void **)(&obs_use_wildcards) );
		pony_gnss_sol_free_null( (void **)(&obs_off_wildcards) );
		pony_gnss_sol_free_null( (void **)(&obs_use_count    ) );
		pony_gnss_sol_free_null( (void **)(&obs_off_count    ) );

	}

	// regular processing
	else {
		
		if (pony->gnss == NULL) // requires gnss initialized
			return;

		for (r = 0; r < pony->gnss_count; r++)
			if (pony->gnss[r].cfg == NULL)
				continue;
			else
				for (sys = 0; sys < sys_count; sys++) {
					switch (sys) {
						case gps:
							if (pony->gnss[r].gps == NULL)
								continue;
							maxsat		= pony->gnss[r].gps->max_sat_count;
							sat			= pony->gnss[r].gps->sat;
							maxobs		= pony->gnss[r].gps->obs_count;
							obstypes	= pony->gnss[r].gps->obs_types;
							break;
						case glo:
							if (pony->gnss[r].glo == NULL)
								continue;
							maxsat		= pony->gnss[r].glo->max_sat_count;
							sat			= pony->gnss[r].glo->sat;
							maxobs		= pony->gnss[r].glo->obs_count;
							obstypes	= pony->gnss[r].glo->obs_types;
							break;
						case gal:
							if (pony->gnss[r].gal == NULL)
								continue;
							maxsat		= pony->gnss[r].gal->max_sat_count;
							sat			= pony->gnss[r].gal->sat;
							maxobs		= pony->gnss[r].gal->obs_count;
							obstypes	= pony->gnss[r].gal->obs_types;
							break;
						case bds:
							if (pony->gnss[r].bds == NULL)
								continue;
							maxsat		= pony->gnss[r].bds->max_sat_count;
							sat			= pony->gnss[r].bds->sat;
							maxobs		= pony->gnss[r].bds->obs_count;
							obstypes	= pony->gnss[r].bds->obs_types;
							break;
						default:
							continue;
					}

					for (i = 0; i < maxobs; i++) {
						// select only those present in obs_use list
						for (j = 0; j < obs_use_count[r][sys]*wildcard_len; j += wildcard_len)
							if (!pony_gnss_sol_select_observables_match_wildcard(obstypes[i], obs_use_wildcards[r][sys] + j) ) { // does not match mandatory wildcard
								for (s = 0; s < maxsat; s++) // drop validity for all satellites
									sat[s].obs_valid[i] = 0;
								break;
							}
						// exclude those present in obs_off list
						for (j = 0; j < obs_off_count[r][sys]*wildcard_len; j += wildcard_len)
							if (pony_gnss_sol_select_observables_match_wildcard(obstypes[i], obs_off_wildcards[r][sys] + j) ) // does match exclusion wildcard
							{
								for (s = 0; s < maxsat; s++) // drop validity for all satellites
									sat[s].obs_valid[i] = 0;
								break;
							}
					}
				}

	}

}






// internal routines
	// gnss calculations
void pony_gnss_sol_pos_code_single_receiver(pony_gnss *gnss, double *x, double *S, double *h, double *K, const size_t m, const size_t maxk) {

	enum sys_index {gps, glo, gal, bds, sys_count};

	const char code_id = 'C';
	const double
		k_sigma = 5,
		dx2     = PONY_GNSS_SOL_POS_PRECISION*PONY_GNSS_SOL_POS_PRECISION; // m^2

	size_t i, j, k, s, n, sys, maxsat, maxobs, sys_obs[sys_count];
	pony_gnss_sat *sat;
	double z, rho, r, sigma, sigma_pos, sigma_clock[sys_count];
	char v, (*obs_types)[4];

	// check if current coordinates fall within specified range
	for (k = 0; k < 3 && fabs(gnss->sol.x[k]) <= PONY_GNSS_SOL_MAX_COORD; k++);
	if (k < 3) { // invalid coordinates detected, set them all to zero
		for (k = 0; k < 3; k++)
			gnss->sol.x[k] = 0;
		gnss->sol.x_valid = 0;
	}

	// reset validity flag
	v = 0;
	for (i = 0; i < PONY_GNSS_SOL_POS_MAX_ITERATIONS; i++) {

		n = 0; // measurements used so far

		// init estimates
		for (k = 0; k < m; k++)
			x[k] = 0;
		for (k = 0; k < maxk; k++)
			S[k] = 0;
		for (j = 0; j < m; j++) {
				pony_linal_u_ij2k(&k, j, j, m);
				S[k] = PONY_GNSS_SOL_MAX_COORD; // reset estimated coordinate covariances
		}

		// process measurements
		for (sys = 0; sys < sys_count; sys++) {
			sys_obs[sys] = 0;
			maxsat = 0; 
			maxobs = 0;
			obs_types = NULL;
			sat = NULL;
			switch (sys) {
				case gps:
					if (gnss->gps == NULL) 
						continue;
					maxsat		= gnss->gps->max_sat_count;
					sat			= gnss->gps->sat;
					maxobs		= gnss->gps->obs_count;
					obs_types	= gnss->gps->obs_types;
					break;
				case glo:
					if (gnss->glo == NULL)
						continue;
					maxsat		= gnss->glo->max_sat_count;
					sat			= gnss->glo->sat;
					maxobs		= gnss->glo->obs_count;
					obs_types	= gnss->glo->obs_types;
					break;
				case gal:
					if (gnss->gal == NULL)
						continue;
					maxsat		= gnss->gal->max_sat_count;
					sat			= gnss->gal->sat;
					maxobs		= gnss->gal->obs_count;
					obs_types	= gnss->gal->obs_types;
					break;
				case bds:
					if (gnss->bds == NULL)
						continue;
					maxsat		= gnss->bds->max_sat_count;
					sat			= gnss->bds->sat;
					maxobs		= gnss->bds->obs_count;
					obs_types	= gnss->bds->obs_types;
					break;
				default:
					continue;
			}

			// run through all satellites
			for (s = 0; s < maxsat; s++) {

				if ( !(sat[s].x_valid) 
					|| (sat[s].sinEl_valid && sat[s].sinEl < gnss->settings.sinEl_mask) )
					continue; // skip satellite if its coordinates aren't valid, or elevation angle known and below the mask

				// calculated distance from current solution to satellite
				rho = 0;
				for (k = 0; k < 3; k++) 
					rho += (sat[s].x[k] - gnss->sol.x[k])*(sat[s].x[k] - gnss->sol.x[k]);
				rho = sqrt(rho);

				// measurement model coefficients
				for (k = 0; k < 3; k++)
					h[k] = (sat[s].x[k] - gnss->sol.x[k])/rho;
				for (; k < m; k++)
					h[k] = 0;
				h[3+sys] = -1;

				// run through all measurements
				for (j = 0; j < maxobs; j++) {
					// check if valid code observation
					if ( obs_types[j][0] != code_id || !(sat[s].obs_valid[j]) )
						continue;
					// measurement
					z = rho - sat[s].obs[j] - sat[s].Deltatsv*pony->gnss_const.c;
					// measurement sigma
					if (sat[s].sinEl_valid)
						sigma = gnss->settings.code_sigma*pony_gnss_sol_noise_coeff_from_elev(sat[s].sinEl);
					else
						sigma = gnss->settings.code_sigma;
					// estimate update
					r = pony_linal_kalman_update(x, S, K, z, h, sigma, m);
					sys_obs[sys]++;
					n++; // measurements used
				}
			}

		}

		// add cartesian coordinate estimates to current solution
		for (k = 0; k < 3; k++)
			gnss->sol.x[k] += x[k];

		// check number of measurements vs number of unknowns
		if ( n >= 3 + (sys_obs[gps]>0) + (sys_obs[glo]>0) + (sys_obs[gal]>0) + (sys_obs[bds]>0) )
			v = 1;
		else
			break;

		// check to stop iterations
		if (x[0]*x[0] + x[1]*x[1] + x[2]*x[2] < dx2)
			break;
	}

	if (!v || i >= PONY_GNSS_SOL_POS_MAX_ITERATIONS)
		return; // not enough measurements of iteration limit reached

	// check estimate error covariances
	for (k = 0, sigma_pos = 0, i = 0; i < 3; k += m-i, i++)
		sigma_pos += pony_linal_dot(S+k,S+k,m-i);
	sigma_pos = sqrt(sigma_pos);
	for (; i < m; k += m-i, i++)
		sigma_clock[i-3] = sqrt(pony_linal_dot(S+k,S+k,m-i));

	for (sys = 0, gnss->sol.dt = 0, gnss->sol.dt_valid = 0; sys < sys_count; sys++) {
		if (!sys_obs[sys] || sigma_clock[sys] > k_sigma*gnss->settings.code_sigma)
			continue;
		gnss->sol.dt += x[3+sys];
		gnss->sol.dt_valid++;
	}
	if (gnss->sol.dt_valid)
		gnss->sol.dt = (gnss->sol.dt/gnss->sol.dt_valid)/pony->gnss_const.c; // receiver clock error in seconds
	else
		return; // regard position as invalid if the clock error is too large

	if ( sigma_pos > k_sigma*gnss->settings.code_sigma)
		return; // position considered invalid

	gnss->sol.x_valid = (char)n;
	gnss->sol.llh_valid = pony_gnss_sol_ecef2llh(gnss->sol.x, gnss->sol.llh, pony->gnss_const.gps.a, pony->gnss_const.gps.e2); // geodetic solution

}

	// models
double pony_gnss_sol_noise_coeff_from_elev(double sinEl) {

	return 20*(sqrt(0.1 + sinEl*sinEl) - sinEl);

}
		// ecef cartesian to longitude-latitude-height
		// a - the Earth ellipsoid semimajor axis, e2 - its eccentricity squared
		// return value - llh valid/not valid
char pony_gnss_sol_ecef2llh(double *ecef, double *llh, const double a, const double e2) 
{

	const double dr2 = PONY_GNSS_SOL_POS_PRECISION*PONY_GNSS_SOL_POS_PRECISION;

	int i;
	double	r_e2, r_e, a2,
			s_phi, c_phi, 
			dphi, dh, 
			R2, f1, f2;

	r_e2 = ecef[0]*ecef[0] + ecef[1]*ecef[1];
	r_e = sqrt(r_e2);
	a2 = a*a;

	// starting approximation
	llh[0] = atan2(ecef[1],ecef[0]);	
	llh[1] = atan2(ecef[2],r_e);
	llh[2] = sqrt(r_e2 + ecef[2]*ecef[2]) - a;

	// solve f1 = 0, f2 = 0 by iterations
	for (i = 0; i < PONY_GNSS_SOL_POS_MAX_ITERATIONS; i++) {

		// intermediate quantities
		s_phi = sin(llh[1]);
		c_phi = cos(llh[1]);
		R2 = a/sqrt(1-e2*s_phi*s_phi);
		f1 = r_e - (R2+llh[2])*c_phi;
		f2 = ecef[2] - ((1-e2)*R2+llh[2])*s_phi;

		// increments
		dphi	= (f1*s_phi - f2*c_phi)/(R2 + llh[2]);
		dh		=  f1*c_phi + f2*s_phi;

		llh[1] -= dphi;
		llh[2] += dh;

		if (dphi*dphi*a2 + dh*dh < dr2)
			break;

	}

	return (i < PONY_GNSS_SOL_POS_MAX_ITERATIONS && fabs(llh[2]) < PONY_GNSS_SOL_MAX_COORD) ? 1 : 0;

}

	// observable codes
		// read 3-character observable wildcards from src to dest, returns number of wildcards found, dest == NULL only counts wildcards
size_t pony_gnss_sol_select_observables_read_wildcards(char *dest,  char *src, const size_t src_len, const char *token) {

	const char delim = '=', quote = '"', brace_open = '{', brace_close = '}', bracket_open = '[', bracket_close = ']';
	const size_t wildcard_len = 3;

	size_t i, j, k, n, len1;

	for (n = 0; token[n]; n++); // determine token length
	if (n == 0)
		return 0;
	len1 = src_len - n;

	// look for the token
	for (i = 0, j = 0, k = 0; i < len1 && src[i]; i++) // throughout the string
		if (src[i] == quote) // skip quoted values
			for (i++; src[i] && i < len1 && src[i] != quote; i++);
		else if (src[i] == brace_open) // skip groups
			for (i++; src[i] && i < len1 && src[i] != brace_close; i++);
		else {
			for (j = 0, k = i; j < n && src[k] == token[j]; j++, k++); // check token
			if (j == n) // token found
				break;
		}
	if (j < n) // token not found
		return 0;

	// try to parse
	for (i = k+1; i < src_len && src[i] && src[i] <= ' '; i++); // skip all non-printables
	if (i >= src_len || src[i] != delim) // no delimiter found
		return 0; 

	for (i++; i < src_len && src[i] && src[i] <= ' '; i++); // skip delimiter and all non-printables
	if (i >= src_len || src[i] != bracket_open) // no opening bracket found
		return 0; 

	for (i++; i < src_len && src[i] && src[i] <= ' '; i++); // skip opening bracket and all non-printables
	for (n = 0; i < src_len && src[i] && src[i] != bracket_close; ) { // collecting 3-character wildcards
		for (k = n*wildcard_len, j = 0; j < wildcard_len && i < src_len && src[i] > ' ' && src[i] != bracket_close; k++, j++, i++) {
			if (dest != NULL)
				dest[k] = src[i];
		}
		if (j == wildcard_len) // full 3-character wildcard copied
			n++;
		for (; i < src_len && src[i] > ' ' && src[i] != bracket_close; i++); // skip all excess printables
		for (; i < src_len && src[i] && src[i] <= ' '; i++); // skip all non-printables
	}

	if (src[i] != bracket_close)
		return 0; // no closing bracket found

	return n;

}

		// check if the obstype matched 3-character wildcard
char pony_gnss_sol_select_observables_match_wildcard(char *obstype, char *wildcard) {

	const size_t wildcard_len = 3;
	const char anychar = '?';

	size_t i;

	for (i = 0; i < wildcard_len && obstype[i] && wildcard[i] && (wildcard[i] == anychar || obstype[i] == wildcard[i]); i++);
	
	return (i == wildcard_len)? 1 : 0;
}

	// memory handling
		// free memory with NULL-check and NULL-assignment
void pony_gnss_sol_free_null(void **ptr) {
	
	if (*ptr == NULL)
		return;
	
	free(*ptr);
	*ptr = NULL;

}
		// allocate [gnss_count x internal_size ] double array
char pony_gnss_sol_alloc_ppointer_gnss_count(double ***ptr, const size_t internal_size) {

	size_t r;

	if (*ptr != NULL)
		return 1; // already allocated

	*ptr = (double **)calloc( pony->gnss_count, sizeof(double *) );
	if (*ptr == NULL)
		return 0;

	for (r = 0; r < pony->gnss_count; r++) {
		(*ptr)[r] = NULL;
		(*ptr)[r] = (double *)calloc( internal_size, sizeof(double) );
		if ( (*ptr)[r] == NULL )
			return 0;
	}

	return 1;
}

void pony_gnss_sol_free_ppointer_gnss_count(void ***ptr) {

	size_t r;

	if (*ptr == NULL) 
		return;
	
	for (r = 0; r < pony->gnss_count; r++)
		pony_gnss_sol_free_null( &((*ptr)[r]) );
	free(*ptr);
	*ptr = NULL;

}