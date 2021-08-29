// Aug-2021
/*	pony_gnss_io_rinex 
	
	pony plugins for GNSS RINEX input/output:

	- pony_gnss_io_rinex_v3_read_obs_from_file 
		Reads observation data (measurements) from RINEX v3.0x files, including header and data records.
		Supports GPS, GLONASS, Galileo and BeiDou systems.
		Multi-receiver/multi-antenna capable, with syncronization option.
		Does not support header updates after END OF HEADER line.
		Only limited number of header labels are processed. 

	- pony_gnss_io_rinex_read_eph_from_file
		Reads navigation data (ephemeris) from RINEX v2-3 files, including header and data records.
		Supports GPS, GLONASS, Galileo and BeiDou systems.
		Multi-receiver/multi-antenna capable.
		Does not support header updates after END OF HEADER line.
		Only limited number of header labels are processed.
	
*/

#include <stdio.h>
#include <stdlib.h>

#include "../../pony.h"

// pony bus version check
#define PONY_GNSS_IO_RINEX_BUS_VERSION_REQUIRED 7
#if PONY_BUS_VERSION < PONY_GNSS_IO_RINEX_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif

// internal defines
#define PONY_GNSS_IO_RINEX_BUFFER_SIZE 2048

// internal routines
	// RINEX routines
		// observation files
char   pony_gnss_io_rinex_v3_obs_file_header_read(pony_gnss *gnss, FILE *fp, char *buf);
char   pony_gnss_io_rinex_v3_obs_file_record_read(pony_gnss *gnss, FILE *fp, char *buf);
int    pony_gnss_io_rinex_v3_obs_file_record_read_parse_entry(char *buf, const size_t width, const char *fmt, void *dest);
char   pony_gnss_io_rinex_obs_file_header_parse_sys_obstypes(size_t *obs_count, char (**obs_types)[4], pony_gnss_sat *sat,  char *buf, const size_t len, const size_t sat_count, char flag);
char   pony_gnss_io_rinex_obs_file_header_parse_glo_slot_frq(pony_gnss_glo *,  char *buf, const size_t len, char flag);
char   pony_gnss_io_rinex_obs_file_record_parse_obs_line(pony_gnss_sat *sat,  char *buf, const size_t len, const size_t sat_count, const size_t obs_count);
		// ephemeris files
char   pony_gnss_io_rinex_eph_file_header_read(pony_gnss *gnss, FILE *fp, char *buf);
char   pony_gnss_io_rinex_eph_file_header_parse_iono_corr(pony_gnss *gnss,  char *buf, const size_t len);
char   pony_gnss_io_rinex_eph_file_header_parse_iono_corr_line(double *iono,  char *buf, const size_t len, const size_t field_count);
char   pony_gnss_io_rinex_eph_file_header_parse_time_corr(pony_gnss *gnss,  char *buf, const size_t len);
void   pony_gnss_io_rinex_eph_file_header_parse_time_corr_line(double *clock_corr, char *time_sys, char *valid,  char *buf, const size_t len);
void   pony_gnss_io_rinex_eph_file_records_read(pony_gnss_sat *sat, pony_time_epoch *epoch, FILE *fp, char *buf, double *eph, unsigned int *sn, const size_t sys, const size_t maxsat, const size_t maxeph);
char   pony_gnss_io_rinex_eph_file_parse_record_header(double *eph, char *buf, const size_t len);
size_t pony_gnss_io_rinex_eph_file_parse_record_lines(double *eph,  FILE *fp, char *buf, const size_t lines);
char   pony_gnss_io_rinex_file_header_parse_leap_sec(pony_gnss *gnss,  char *buf, const size_t len, char bds_only);

	// service routines
void   pony_gnss_io_rinex_free_null(void **ptr); // free memory with NULL-check and NULL-assignment
	   
void   pony_gnss_io_rinex_drop_flags_pony_sats(pony_gnss_sat *sat, const size_t sat_count, const size_t obs_count);
void   pony_gnss_io_rinex_drop_flags_pony_sol(pony_sol *sol);
	   
void   pony_gnss_io_rinex_charrep(char *s, char oldc, char newc);
char   pony_gnss_io_rinex_read_token_str(char *value,  const char *token, const char *src, const size_t len);
char   pony_gnss_io_rinex_read_token_double(double *dest, const char *token, const char *src, const size_t len);
char   pony_gnss_io_rinex_find_token(const char *token, const char *src, const size_t len);
char   pony_gnss_io_rinex_epoch_check(pony_time_epoch *epoch);
char   pony_gnss_io_rinex_epoch_from_array(pony_time_epoch *epoch, double *YMDhms);
int    pony_gnss_io_rinex_resolve_2digit_year(int Y);
double pony_gnss_io_rinex_dmod(double x, double y);













// plugin definitions

/* pony_gnss_io_rinex_v3_read_obs_from_file - pony plugin
	
	Reads observation data (measurements) from RINEX v3.0x files, including header and data records.
	Supports GPS, GLONASS, Galileo and BeiDou systems.
	Multi-receiver/multi-antenna capable, with syncronization option.
	Does not support header updates after END OF HEADER line.
	Only limited number of header labels are processed. 

	description:
		supported header labels are
			- required
				RINEX VERSION / TYPE
				SYS / # / OBS TYPES
				TIME OF FIRST OBS
				END OF HEADER
			- optional
				GLONASS SLOT / FRQ #
				LEAP SECONDS	
	uses:
		pony->gnss_count
		pony->gnss[].gps/glo/gal/bds->max_sat_count
	changes:
		pony->mode
		pony->gnss[].epoch
		pony->gnss[].leap_sec
		pony->gnss[].leap_sec_valid
		pony->gnss[].gps/glo/gal/bds->obs_count
		pony->gnss[].gps/glo/gal/bds->obs_types
		pony->gnss[].gps/glo/gal/bds->sat[]. obs
		pony->gnss[].gps/glo/gal/bds->sat[]. obs_valid
		pony->gnss[].gps/glo/gal/bds->sat[].t_em_valid
		pony->gnss[].gps/glo/gal/bds->sat[].   x_valid
		pony->gnss[].gps/glo/gal/bds->sat[].   v_valid
		pony->gnss[].glo->freq_slot
		pony->gnss[].sol.  x_valid
		pony->gnss[].sol.llh_valid
		pony->gnss[].sol.  v_valid
		pony->gnss[].sol.  q_valid
		pony->gnss[].sol.  L_valid
		pony->gnss[].sol.rpy_valid
		pony->gnss[].sol. dt_valid
	cfg parameters:
		{gnss: obs_in} - gnss rinex observation data input source
			type   : string
			range  : valid file name
			default: none
			example: {gnss: obs_in = "data/191025_0255/191025_0255_rover.19o"}
		gnss_sync - multi-receiver observations synchronization threshold, sec
			type   : float
			range  : >0
			default: none
			example: gnss_sync = 0.09
			negative values result in sync turned off
*/
void pony_gnss_io_rinex_v3_read_obs_from_file(void) {

	const char 
		cfg_obs_token  [] = "obs_in",    // gnss rinex observation data input source              parameter name in configuration
		gnss_sync_token[] = "gnss_sync"; // multi-receiver observations synchronization threshold parameter name in configuration
	
	static FILE   **fp        = NULL;    // file pointers for each receiver
	static char   **buf       = NULL;    // data buffers  for each receiver
	static double   gnss_sync = -1;      // multi-receiver observations synchronization threshold in seconds, negative for sync off

	size_t          r, count;                     // common counting variables
	pony_time_epoch latest_epoch = {0,0,0,0,0,0}; // latest epoch to sync

	// requires gnss data initialized
	if (pony->gnss == NULL)
		return;

	// init
	if (pony->mode == 0)	{

		// allocate memory
		fp	= (FILE **)calloc( pony->gnss_count, sizeof(FILE *) );
		buf	= (char **)calloc( pony->gnss_count, sizeof(char *) );
		if (fp == NULL || buf == NULL) {
			printf("\nERROR: memory allocation failed for rinex observation data");
			pony->mode = -1;
			return;
		}
		// initialize data for each receiver
		for (r = 0; r < pony->gnss_count; r++) {

			// initialize file pointer
			fp[r] = NULL;
			// requires gnss data initialized for current receiver
			if (pony->gnss[r].cfg == NULL)
				continue;
			// allocate data buffer
			buf[r] = (char *)calloc( PONY_GNSS_IO_RINEX_BUFFER_SIZE, sizeof(char) );
			if (buf[r] == NULL) {
				printf("\nERROR: memory allocation failed for rinex observation data buffers");
				pony->mode = -1;
				return;
			}
			// observation file name from configuration
			if (  !pony_gnss_io_rinex_read_token_str(buf[r],  cfg_obs_token, pony->gnss[r].cfg_settings, pony->gnss[r].settings_length) 
				|| buf[r][0] == '\0' ) {
				printf("\n\tERROR: could not find GNSS observations RINEX file name in the configuration for gnss[%d]:\n\t\t'%s'",
					r, pony->gnss[r].cfg_settings);
				pony->mode = -1;
				return;
			}
			// open the file
			fp[r] = fopen(buf[r],"r");
			if (fp[r] == NULL) {
				printf("\n\terror: could not open GNSS observations RINEX file '%s' for gnss[%d]",buf[r],r);
				pony->mode = -1;
				return;
			}
			printf("\n\t'%s' opened",buf[r]);
			// read RINEX header
			if ( !pony_gnss_io_rinex_v3_obs_file_header_read(&(pony->gnss[r]), fp[r], buf[r]) ) {
				printf("\n\tERROR: GNSS observations RINEX file header not parsed for gnss[%d]", r);
				pony->mode = -1;
				return;
			}

		}

		// check for gnss_sync option
		if ( pony_gnss_io_rinex_read_token_double(&gnss_sync, gnss_sync_token, pony->cfg_settings, pony->settings_length)
			&& gnss_sync > 0)
			printf("\n\n\t GNSS observations will be synchronized by time of day within %.3f msec to the latest one\n", gnss_sync*1e3);
		else
			gnss_sync = -1;

		// no return, try to read the first observation chunk in RINEX records
	}

	// terminate
	if (pony->mode < 0)	{

		// close files, if not yet
		if (fp != NULL) {
			for (r = 0; r < pony->gnss_count; r++)
				if (pony->gnss[r].cfg == NULL)
					continue;
				else if (fp[r] != NULL)
					fclose(fp[r]);
			free(fp);
			fp = NULL;
		}
		// free memory
		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].cfg == NULL)
				continue;
			pony_gnss_io_rinex_free_null( (void **)(&(buf[r])) );
		}
		pony_gnss_io_rinex_free_null( (void **)(&buf) );

		return;

	}

	// regular processing
	else {

		// try to parse data records for all available receivers, count total measurements
		for (r = 0, count = 0; r < pony->gnss_count; r++)
			if (fp[r] == NULL)
				continue;
			else
				count += pony_gnss_io_rinex_v3_obs_file_record_read(&(pony->gnss[r]), fp[r], buf[r]);
		if (!count) // no single measurement parsed
			pony->mode = -1;
		// epoch synchronization
		if (gnss_sync <= 0) // no observation time skew check
			return;
		// try to synchronize observations
			// look for the latest epoch in all available receivers
		for (r = 0; r < pony->gnss_count; r++)
			if (fp[r] == NULL)
				continue;
			else if (pony_time_epochs_compare(&latest_epoch, &(pony->gnss[r].epoch)) == -1)
				latest_epoch = pony->gnss[r].epoch;
			// for receivers falling behind, read succeeding records
		for (r = 0; r < pony->gnss_count; r++)
			if (fp[r] == NULL)
				continue;
			else
				while ( 
					pony_gnss_io_rinex_dmod(
						(       latest_epoch.h*3600.0 +        latest_epoch.m*60 +        latest_epoch.s) - // latest   epoch time of day
						(pony->gnss[r].epoch.h*3600.0 + pony->gnss[r].epoch.m*60 + pony->gnss[r].epoch.s),  // receiver epoch time of day
						pony->gnss_const.sec_in_d)                                                          // modulo 86400 (seconds in a day)
						>
						gnss_sync )
					if ( !pony_gnss_io_rinex_v3_obs_file_record_read(&(pony->gnss[r]), fp[r], buf[r]) ) { // read next observation data record
						pony->mode = -1;
						break;
					}
		
	}

}

/* pony_gnss_io_rinex_read_eph_from_file - pony plugin
	
	Reads navigation data (ephemeris) from RINEX v2-3 files, including header and data records.
	Supports GPS, GLONASS, Galileo and BeiDou systems.
	Multi-receiver/multi-antenna capable.
	Does not support header updates after END OF HEADER line.
	Only limited number of header labels are processed.

	description:
		supported header labels are
			- mandatory
				RINEX VERSION / TYPE
				END OF HEADER
			- optional
				IONOSPHERIC CORR
				TIME SYSTEM CORR
				LEAP SECONDS
	uses:
		pony->gnss_count
		pony->gnss[].gps/glo/gal/bds->max_eph_count
		pony->gnss[].gps/glo/gal/bds->max_sat_count
	changes:
		pony->mode
		pony->gnss[].leap_sec
		pony->gnss[].leap_sec_valid
		pony->gnss[].gps/bds->iono_a
		pony->gnss[].gps/bds->iono_b
		pony->gnss[].gal->iono
		pony->gnss[].gps/gal/bds->iono_valid
		pony->gnss[].gps/glo/gal/bds->clock_corr
		pony->gnss[].gps/glo/gal/bds->clock_corr_to
		pony->gnss[].gps/glo/gal/bds->clock_corr_valid
		pony->gnss[].gps/glo/gal/bds->sat[].eph
		pony->gnss[].gps/glo/gal/bds->sat[].eph_valid
	cfg parameters:
		{gnss:       eph_in } - gnss rinex mixed navigation data input source
		and/or
		{gnss: {gps: eph_in}} - gnss rinex gps     navigation data input source
		and/or
		{gnss: {glo: eph_in}} - gnss rinex glonass navigation data input source
		and/or
		{gnss: {gal: eph_in}} - gnss rinex galileo navigation data input source
		and/or
		{gnss: {bds: eph_in}} - gnss rinex beidou  navigation data input source
			type   : string
			range  : valid file name
			default: none
			example: {gnss: eph_in = "data/191025_0255/191025_0255_rover.19n"}
			         {gnss: {gps: eph_in = "data/191025_0255/191025_0255_rover.19n"}}
			         {gnss: {glo: eph_in = "data/191025_0255/191025_0255_rover.19r"}}
			         {gnss: {gal: eph_in = "data/191025_0255/191025_0255_rover.19e"}}
			         {gnss: {bds: eph_in = "data/191025_0255/191025_0255_rover.19c"}}
*/
void pony_gnss_io_rinex_read_eph_from_file(void) {

	enum sys_index {gps, glo, gal, bds, sys_count};
	const char cfg_eph_token[] = "eph_in";
	
	static FILE         ***fp            = NULL; // file pointer for each receiver for each system (for mixed rinex files only one is used per receiver)
	static char         ***buf           = NULL; // reading buffer for each receiver for each system (for mixed rinex files only one is used per receiver)
	static double       ***eph           = NULL; // latest ephemeris set parsed for each receiver for each system
	static unsigned int  **sn            = NULL; // current satellite number for each receiver for each system
	static size_t         *max_eph_count = NULL; // maximum number of ephemeris for each system
	static size_t         *max_sat_count = NULL; // maximum number of satellites for each system

	char 
		fname [PONY_GNSS_IO_RINEX_BUFFER_SIZE] = "", // system-specific file name, if found in configuration
		fname0[PONY_GNSS_IO_RINEX_BUFFER_SIZE] = ""; // mixed data file name,      if found in configuration
	pony_gnss_sat *sat;    // pointer to particular satellite data
	size_t         r, sys; // receiver and system index variables

	// requires gnss data initialized
	if (pony->gnss == NULL)
		return;

	// init
	if (pony->mode == 0)	{

		// determine maximum number of ephemeris and satellites
		max_eph_count = (size_t *)calloc( sys_count, sizeof(size_t) );
		max_sat_count = (size_t *)calloc( sys_count, sizeof(size_t) );
		for (r = 0; r < pony->gnss_count; r++) {
			// skip if current receiver uninitialized
			if (pony->gnss[r].cfg == NULL)
				continue;
			if (pony->gnss[r].gps != NULL) { // for gps system
				if (pony->gnss[r].gps->max_eph_count > max_eph_count[gps]) max_eph_count[gps] = pony->gnss[r].gps->max_eph_count;
				if (pony->gnss[r].gps->max_sat_count > max_sat_count[gps]) max_sat_count[gps] = pony->gnss[r].gps->max_sat_count;
			}
			if (pony->gnss[r].glo != NULL) { // for glonass system
				if (pony->gnss[r].glo->max_eph_count > max_eph_count[glo]) max_eph_count[glo] = pony->gnss[r].glo->max_eph_count;
				if (pony->gnss[r].glo->max_sat_count > max_sat_count[glo]) max_sat_count[glo] = pony->gnss[r].glo->max_sat_count;
			}
			if (pony->gnss[r].gal != NULL) { // for galileo system
				if (pony->gnss[r].gal->max_eph_count > max_eph_count[gal]) max_eph_count[gal] = pony->gnss[r].gal->max_eph_count;
				if (pony->gnss[r].gal->max_sat_count > max_sat_count[gal]) max_sat_count[gal] = pony->gnss[r].gal->max_sat_count;
			}
			if (pony->gnss[r].bds != NULL) { // for beidou system
				if (pony->gnss[r].bds->max_eph_count > max_eph_count[bds]) max_eph_count[bds] = pony->gnss[r].bds->max_eph_count;
				if (pony->gnss[r].bds->max_sat_count > max_sat_count[bds]) max_sat_count[bds] = pony->gnss[r].bds->max_sat_count;
			}
		}
		// check if at least one system initialized
		for (sys = 0; sys < sys_count; sys++)
			if (max_eph_count[sys] > 0)
				break;
		if (sys == sys_count) // neither GPS, GLONASS, Galileo or BeiDou data initialized on the bus
			return; 
		// allocate buffers for receivers
		eph = (double      ***)calloc( pony->gnss_count, sizeof(double      **) );
		fp  = (FILE        ***)calloc( pony->gnss_count, sizeof(FILE        **) );
		buf = (char        ***)calloc( pony->gnss_count, sizeof(char        **) );
		sn  = (unsigned int **)calloc( pony->gnss_count, sizeof(unsigned int *) );
		if (eph == NULL || fp == NULL || buf == NULL || sn == NULL) {
			printf("\nERROR: memory allocation failed for rinex ephemeris data");
			pony->mode = -1;
			return;
		}
		// initialize data for each receiver
		for (r = 0; r < pony->gnss_count; r++) {
			// initialize file pointer
			fp[r] = NULL;
			if (pony->gnss[r].cfg == NULL)
				continue;
			// allocate variables for each supported system
			fp [r] = (FILE         **)calloc( sys_count, sizeof(FILE       *) ); 
			buf[r] = (char         **)calloc( sys_count, sizeof(char       *) ); 
			sn [r] = (unsigned int  *)calloc( sys_count, sizeof(unsigned int) );
			eph[r] = (double       **)calloc( sys_count, sizeof(double     *) );
			if (fp[r] == NULL || buf[r] == NULL || sn[r] == NULL || eph[r] == NULL) {
				printf("\nERROR: memory allocation failed for rinex ephemeris receiver data");
				pony->mode = -1;
				return;
			}
			for (sys = 0; sys < sys_count; sys++) {
				eph[r][sys] = (double *)calloc( max_eph_count[sys], sizeof(double) );
				if (eph[r][sys] == NULL && max_eph_count[sys] > 0) {
					printf("\nERROR: memory allocation failed for rinex ephemeris gnss system data");
					pony->mode = -1;
					return;
				}
			}		
			// determine files from configuration
				// check for a mixed multi-gnss file
			pony_gnss_io_rinex_read_token_str(fname0,  cfg_eph_token, pony->gnss[r].cfg_settings, pony->gnss[r].settings_length); 	
				// gps
			fp[r][gps] = NULL;
			if (pony->gnss[r].gps != NULL) // look for navigation data file for gps
				if ( pony_gnss_io_rinex_read_token_str(fname,  cfg_eph_token, pony->gnss[r].gps->cfg, pony->gnss[r].gps->cfglength) )
					fp[r][gps] = fopen(fname , "r"); // try to open for reading
				else
					fp[r][gps] = fopen(fname0, "r"); // try to open for reading
				// glonass
			fp[r][glo] = NULL;
			if (pony->gnss[r].glo != NULL) // look for navigation data file for glonass
				if ( pony_gnss_io_rinex_read_token_str(fname,  cfg_eph_token, pony->gnss[r].glo->cfg, pony->gnss[r].glo->cfglength) )
					fp[r][glo] = fopen(fname, "r"); // try to open for reading
				else
					fp[r][glo] = fopen(fname0, "r"); // try to open for reading
				// galileo
			fp[r][gal] = NULL;
			if (pony->gnss[r].gal != NULL) // look for navigation data file for galileo
				if ( pony_gnss_io_rinex_read_token_str(fname,  cfg_eph_token, pony->gnss[r].gal->cfg, pony->gnss[r].gal->cfglength) )
					fp[r][gal] = fopen(fname , "r"); // try to open for reading
				else
					fp[r][gal] = fopen(fname0, "r"); // try to open for reading
				// beidou
			fp[r][bds] = NULL;
			if (pony->gnss[r].bds != NULL) // look for navigation data file for beidou
				if ( pony_gnss_io_rinex_read_token_str(fname,  cfg_eph_token, pony->gnss[r].bds->cfg, pony->gnss[r].bds->cfglength) )
					fp[r][bds] = fopen(fname, "r"); // try to open for reading
				else
					fp[r][bds] = fopen(fname0, "r"); // try to open for reading
				// check for required files found
			if (    (pony->gnss[r].gps != NULL && fp[r][gps] == NULL) 
				 || (pony->gnss[r].glo != NULL && fp[r][glo] == NULL) 
				 || (pony->gnss[r].gal != NULL && fp[r][gal] == NULL)
				 || (pony->gnss[r].bds != NULL && fp[r][bds] == NULL) ) { // at least one of required ephemeris file does not exist
						printf("\n\tERROR: failed to get all required ephemeris RINEX files from configuration for gnss[%d]:\n\t\t'%s'",
							r, pony->gnss[r].cfg_settings);
						pony->mode = -1;
						return;
					}
			// allocate buffers and read RINEX headers
			for (sys = 0; sys < sys_count; sys++) {
				// skip unopened files
				if (fp[r][sys] == NULL)
					continue;
				// memory buffer
				buf[r][sys] = (char *)calloc( PONY_GNSS_IO_RINEX_BUFFER_SIZE, sizeof(char) );
				if (buf[r][sys] == NULL) {
					printf("\nERROR: memory allocation failed for rinex ephemeris data buffer");
					pony->mode = -1;
					return;
				}
				// parse navigation data header
				if ( !pony_gnss_io_rinex_eph_file_header_read(&(pony->gnss[r]), fp[r][sys], buf[r][sys]) ) {
					printf("\n\tERROR: ephemeris RINEX file header not parsed for gnss[%d]", r);
					pony->mode = -1;
					return;
				}
			}

		}

	}

	// terminate
	if (pony->mode < 0)	{

		// close files and free memory, if not already
		if (fp != NULL) {
			// for each receiver
			for (r = 0; r < pony->gnss_count; r++) {
				// skip uninitialized
				if (fp[r] == NULL)
					continue;
				// for each system
				for (sys = 0; sys < sys_count; sys++) {
					if (fp[r][sys] == NULL) 
						continue; 
					fclose(fp[r][sys]);
					fp[r][sys] = NULL;
				}
				free(fp[r]);
				fp[r] = NULL;
			}
			free(fp);
			fp = NULL;
		}
		// free memory
		for (r = 0; r < pony->gnss_count; r++) {
			if (pony->gnss[r].cfg == NULL)
				continue;
			if (sn != NULL)
				pony_gnss_io_rinex_free_null( (void **)(&(sn[r])) );
			if (eph != NULL && eph[r] != NULL) {
				for (sys = 0; sys < sys_count; sys++)
					pony_gnss_io_rinex_free_null( (void **)(&(eph[r][sys])) );
				pony_gnss_io_rinex_free_null( (void **)(&(eph[r])) );
			}
			if (buf != NULL && buf[r] != NULL) {
				for (sys = 0; sys < sys_count; sys++)
					pony_gnss_io_rinex_free_null( (void **)(&(buf[r][sys])) );
				pony_gnss_io_rinex_free_null( (void **)(&(buf[r])) );
			}
		}
		pony_gnss_io_rinex_free_null( (void **)(&sn				) );
		pony_gnss_io_rinex_free_null( (void **)(&eph			) );
		pony_gnss_io_rinex_free_null( (void **)(&buf			) );
		pony_gnss_io_rinex_free_null( (void **)(&max_eph_count	) );
		pony_gnss_io_rinex_free_null( (void **)(&max_sat_count	) );

	}

	// regular processing
	else {
		// for each receiver
		for (r = 0; r < pony->gnss_count; r++)
			// skip uninitialized
			if (fp[r] != NULL)
				// for each system
				for (sys = 0; sys < sys_count; sys++)
					// skip uninitialized
					if (fp[r][sys] != NULL) {
						// assign pointers
						sat = NULL;
						switch (sys) {
							case gps:
								if (pony->gnss[r].gps != NULL) 
									sat = pony->gnss[r].gps->sat; 
								else
									continue;
								break;
							case glo:
								if (pony->gnss[r].glo != NULL) 
									sat = pony->gnss[r].glo->sat; 
								else
									continue;
								break;
							case gal:
								if (pony->gnss[r].gal != NULL) 
									sat = pony->gnss[r].gal->sat; 
								else
									continue;
								break;
							case bds:
								if (pony->gnss[r].bds != NULL) 
									sat = pony->gnss[r].bds->sat; 
								else
									continue;
								break;
							default:
								continue;
						}
						// parse records from file
						pony_gnss_io_rinex_eph_file_records_read(
							sat, &(pony->gnss[r].epoch), 
							fp[r][sys], buf[r][sys], eph[r][sys], &(sn[r][sys]), 
							sys, max_sat_count[sys], max_eph_count[sys]);
					}
	}

}








// internal routines
	// RINEX routines
		// observation file header
char pony_gnss_io_rinex_v3_obs_file_header_read(pony_gnss *gnss, FILE *fp, char *buf) {

	enum system_id          {gps, glo, gal, bds, sys_count};

	const char   sys_id[] = {'G', 'R', 'E', 'C'}; 
	const size_t label_offset = 61, label_len = 20, type_offset = 20, sys_offset = 40;

	// required label - version and type
	const char  rinex_version_type_label[] = "RINEX VERSION / TYPE";
	const float rinex_version_range[] = {3.02f, 3.04f};
	const float rinex_version_prec = 0.001f;
	const char  rinex_type_token = 'O';

	// required label - system & observation types
	const char rinex_sys_obstypes_label[] = "SYS / # / OBS TYPES";
	// required label - time of 1st observation
	const char rinex_TOFO_label[] = "TIME OF FIRST OBS";
	// required label - end of header
	const char rinex_EOH_label[] = "END OF HEADER"  ;

	const size_t tokens_required = 4;

	// optional label - glonass slot frequencies
	const char rinex_glo_freq_slot_label[] = "GLONASS SLOT / FRQ #";
	// optional label - leap seconds
	const char rinex_leap_sec_label[] = "LEAP SECONDS";

	size_t tokens_found = 0;
	float version;
	size_t i, sys, maxsat, *obs_count = NULL;
	int scanned;
	char rinex_type_sys, current_sys = 0, sys_obstypes_found = 0, glo_freq_slot_found = 0, flag, (**obs_types)[4] = NULL; 
	pony_gnss_sat *sat = NULL;



	if (fp == NULL || feof(fp))
		return 0;

	// check starting label
	rinex_type_sys = 0;
	while ( !feof(fp) ) {
		// read a new string
		fgets(buf, PONY_GNSS_IO_RINEX_BUFFER_SIZE, fp);
		for (i = 0; buf[i] >= ' '; i++); buf[i] = '\0';		// set end of line on the first non-printable character
		if (i < label_offset) continue;						// check if buffer contains enough characters to proceed

		// check if a starting token is found
		if ( pony_gnss_io_rinex_find_token(rinex_version_type_label,buf+label_offset-1,label_len) ) {
			printf("\n\t\t%s",buf);
			if (sscanf(buf,"%f",&version) < 1) {
				printf("\n\t\terror: RINEX header line %s not parsed from %s", rinex_version_type_label, buf);
				return 0;
			}
			if (version+rinex_version_prec < rinex_version_range[0] || rinex_version_range[1] < version-rinex_version_prec) {
				printf("\n\t\terror: RINEX version out of range: header contains %f, must be from %f to %f", version, rinex_version_range[0], rinex_version_range[1]);
				return 0;
			}
			if (buf[type_offset] != rinex_type_token) {
				printf("\n\t\terror: RINEX observation file type incorrect: %s, must start with '%c'", buf + type_offset, rinex_type_token);
				return 0;
			}
			rinex_type_sys = buf[sys_offset];
			tokens_found++;
			break;
		}
	}
	// check other labels
	current_sys = 0;
	flag = 0;
	while ( !feof(fp) ) {

		fgets(buf, PONY_GNSS_IO_RINEX_BUFFER_SIZE, fp);
		for (i = 0; buf[i] >= ' '; i++); buf[i] = '\0';		// set end of line on the first non-printable character
		if (i < label_offset) continue;						// check if buffer contains enough characters to proceed

		// check if a token is found
		if ( pony_gnss_io_rinex_find_token(rinex_sys_obstypes_label,buf+label_offset-1,label_len) ) {
			printf("\n\t\t%s",buf);

			if (!sys_obstypes_found) {
				tokens_found++;
				sys_obstypes_found = 1;
			}

			// check for system id at line start
			for (i = 0, maxsat = 0, sys = 0; sys < sys_count; sys++)
				if (buf[i] == sys_id[sys]) { // start new system
					current_sys = sys_id[sys];
					flag = 0;
					break;
				}
			// if not found at line start, check if a system has already assigned
			if (sys >= sys_count && current_sys)
				for (sys = 0; sys < sys_count; sys++)
					if (current_sys == sys_id[sys]) { // continue with previously selected system
						flag = 1;
						break;
					}
					
			switch (sys) {
				case gps:  
					if (gnss->gps == NULL) 
						continue;
					obs_count = &(gnss->gps->obs_count);
					obs_types = &(gnss->gps->obs_types);
					sat       =   gnss->gps->sat;
					maxsat    =   gnss->gps->max_sat_count; 
					break;
				case glo:  
					if (gnss->glo == NULL) 
						continue;
					obs_count = &(gnss->glo->obs_count);
					obs_types = &(gnss->glo->obs_types);
					sat       =   gnss->glo->sat;
					maxsat    =   gnss->glo->max_sat_count; 
					break;
				case gal:  
					if (gnss->gal == NULL) 
						continue;
					obs_count = &(gnss->gal->obs_count);
					obs_types = &(gnss->gal->obs_types);
					sat       =   gnss->gal->sat;
					maxsat    =   gnss->gal->max_sat_count; 
					break;
				case bds:  
					if (gnss->bds == NULL) 
						continue;
					obs_count = &(gnss->bds->obs_count);
					obs_types = &(gnss->bds->obs_types);
					sat       =   gnss->bds->sat;
					maxsat    =   gnss->bds->max_sat_count; 
					break;
				default: continue;
			}

			if ( !pony_gnss_io_rinex_obs_file_header_parse_sys_obstypes(obs_count, obs_types, sat,  buf + i, label_offset-1-i, maxsat, flag) ) {
				printf("\n\t\terror: RINEX observation types not parsed in header from %s", buf);
				return 0;
			}

			continue;
		}

		// check if a token is found
		if ( pony_gnss_io_rinex_find_token(rinex_glo_freq_slot_label,buf+label_offset-1,label_len) && gnss->glo != NULL) {
			printf("\n\t\t%s",buf);
			if (!glo_freq_slot_found) {
				if ( !pony_gnss_io_rinex_obs_file_header_parse_glo_slot_frq(gnss->glo,  buf, label_offset-1, 0) )
					printf("\n\t\twarning: RINEX glonass frequency slots not parsed in header from %s\n", buf);
				glo_freq_slot_found = 1;
			}
			else
				if ( !pony_gnss_io_rinex_obs_file_header_parse_glo_slot_frq(gnss->glo,  buf, label_offset-1, 1) )
					printf("\n\t\twarning: RINEX glonass frequency slots not parsed in header from %s\n", buf);
			continue;
		}

		// check if a token is found
		if ( pony_gnss_io_rinex_find_token(rinex_TOFO_label,buf+label_offset-1,label_len) ) {
			printf("\n\t\t%s",buf);

			scanned = sscanf( buf,"%d %d %d %d %d %lf",
				&(gnss->epoch.Y),&(gnss->epoch.M),&(gnss->epoch.D),
				&(gnss->epoch.h),&(gnss->epoch.m),&(gnss->epoch.s) );
			if ( scanned < 6 || !pony_gnss_io_rinex_epoch_check( &(gnss->epoch) ) ) {
				printf("\n\t\terror: RINEX time of first observation not parsed in header from %s", buf);
				return 0;
			}
			gnss->epoch.Y = pony_gnss_io_rinex_resolve_2digit_year(gnss->epoch.Y);

			tokens_found++;
			continue;
		}

		// check if a token is found
		if ( pony_gnss_io_rinex_find_token(rinex_leap_sec_label,buf+label_offset-1,label_len) ) {
			printf("\n\t\t%s",buf);
			if ( !pony_gnss_io_rinex_file_header_parse_leap_sec(gnss, buf, label_offset-1, rinex_type_sys == sys_id[bds]) )
				printf("\n\t\twarning: leap seconds not parsed from '%s'",buf);
			continue;
		}

		// check if end of header token is found
		if ( pony_gnss_io_rinex_find_token(rinex_EOH_label,buf+label_offset-1,label_len) ) {
			printf("\n\t\t%s",buf);
			tokens_found++;
			break;
		}

	}

	return (tokens_found == tokens_required)? 1 : 0;
}

	// observation file header line sys/obstypes
char pony_gnss_io_rinex_obs_file_header_parse_sys_obstypes(size_t *obs_count, char (**obs_types)[4], pony_gnss_sat *sat,  char *buf, const size_t len, const size_t sat_count, char flag) {

	const size_t obstype_length = 3;

	static unsigned int current_sys_obs_count = 0, current_sys_obs_index = 0;

	size_t i, j, n;
	int scanned;

	for (n = 0; n < len && buf[n]; n++);
	i = 0;
	switch (flag) {

		case 0: // starting entries

			for (; i < n &&           buf[i] >  ' '; i++); // skip all     printable
			for (; i < n && buf[i] && buf[i] <= ' '; i++); // skip all non-printable

			// observation count
				// parse the number
			scanned = sscanf(buf + i, "%u", &current_sys_obs_count);
			if (scanned < 1 || current_sys_obs_count == 0) 
				return 0;
			*obs_count = current_sys_obs_count;
				// allocate memory for observations & validity flags
			for (j = 0; j < sat_count; j++) {
				sat[j].obs			= (double *)calloc( *obs_count, sizeof(double) );
				sat[j].obs_valid	= (char   *)calloc( *obs_count, sizeof(char  ) );
				if (sat[j].obs == NULL || sat[j].obs_valid == NULL)
					return 0;
			}
				// allocate memory for observation types
			*obs_types = (char (*)[4])calloc( *obs_count, sizeof(char [4]) );
			if (*obs_types == NULL)
				return 0;
			current_sys_obs_index = 0;

			for (; i < n &&           buf[i] >  ' '; i++); // skip all     printable
			for (; i < n && buf[i] && buf[i] <= ' '; i++); // skip all non-printable


		case 1: // continuation

			// parse observation types
			for (; i < n && buf[i] && current_sys_obs_index < *obs_count; current_sys_obs_index++) {
				for (j = 0; i < n && buf[i] > ' ' && j < obstype_length; i++, j++)
					(*obs_types)[current_sys_obs_index][j] = buf[i];

				for (; i < n &&           buf[i] >  ' '; i++); // skip all     printable
				for (; i < n && buf[i] && buf[i] <= ' '; i++); // skip all non-printable
			}

	}

	return 1;
}

	// observation file header line glonass slot/frq #
char pony_gnss_io_rinex_obs_file_header_parse_glo_slot_frq(pony_gnss_glo *glo,  char *buf, const size_t len, char flag) {

	const int max_frq_num = 24;

	static unsigned int entry_count = 0, current_entry_index = 0;

	size_t i, n;
	unsigned int sn;
	int scanned;

	for (n = 0; n < len && buf[n]; n++);
	for (i = 0; i < n && buf[i] && buf[i] <= ' '; i++); // skip all non-printable
	switch (flag) {

		case 0: // starting entries

			// entry count
				// parse the number
			scanned = sscanf(buf + i, "%u", &entry_count);
			if (scanned < 1 || entry_count == 0 || entry_count > glo->max_sat_count) 
				return 0;
			current_entry_index = 0;

			for (; i < n &&           buf[i] >  ' '; i++); // skip all     printable
			for (; i < n && buf[i] && buf[i] <= ' '; i++); // skip all non-printable


		case 1: // continuation

			// parse glonass frequency slots
			for (; i < n && buf[i] && current_entry_index < entry_count; current_entry_index++) {
				scanned = sscanf(buf+i, "%*c%u", &sn);
				if (scanned < 1 || sn == 0 || sn > glo->max_sat_count)
					return 0;

				for (; i < n &&           buf[i] >  ' '; i++); // skip all     printable
				for (; i < n && buf[i] && buf[i] <= ' '; i++); // skip all non-printable

				scanned = sscanf( buf+i, "%d", &(glo->freq_slot[sn-1]) );
				if (scanned < 1 || abs(glo->freq_slot[sn-1]) > max_frq_num )
					return 0;

				for (; i < n &&           buf[i] >  ' '; i++); // skip all     printable
				for (; i < n && buf[i] && buf[i] <= ' '; i++); // skip all non-printable
			}

	}

	return 1;

}

	// observation file records
char pony_gnss_io_rinex_v3_obs_file_record_read(pony_gnss *gnss, FILE *fp, char *buf) {

	const char 
		gps_sys_id[] = "G",
		glo_sys_id[] = "R",
		gal_sys_id[] = "E",
		bds_sys_id[] = "C",
		rec_id = '>';

	int epoch_flag = -1, scanned;
	unsigned int i, n = 0;

	if (gnss == NULL || fp == NULL || buf == NULL || gnss->cfg == NULL)
		return 0;

	// wait for the record identifier to occur
	do
		fgets(buf,PONY_GNSS_IO_RINEX_BUFFER_SIZE,fp);
	while (!feof(fp) && buf[0] != rec_id);

	if ( feof(fp) )
		return 0;

	scanned = sscanf(buf, "%*c %d %d %d  %d %d %lf  %d  %u", 
		&(gnss->epoch.Y), &(gnss->epoch.M), &(gnss->epoch.D),
		&(gnss->epoch.h), &(gnss->epoch.m), &(gnss->epoch.s),
		&epoch_flag, &n);

	if (scanned < 8 || !pony_gnss_io_rinex_epoch_check( &(gnss->epoch) ) || epoch_flag) {
		printf("\n\t\terror: RINEX observation record not parsed at %s", buf);
		return 0;
	}

	// if a new epoch has come, all observations and satellite data became outdated, drop their validity
	if (gnss->gps != NULL)
		pony_gnss_io_rinex_drop_flags_pony_sats(gnss->gps->sat, gnss->gps->max_sat_count, gnss->gps->obs_count); // GPS satellite flags
	if (gnss->glo != NULL)
		pony_gnss_io_rinex_drop_flags_pony_sats(gnss->glo->sat, gnss->glo->max_sat_count, gnss->glo->obs_count); // GLONASS satellite flags
	if (gnss->gal != NULL)
		pony_gnss_io_rinex_drop_flags_pony_sats(gnss->gal->sat, gnss->gal->max_sat_count, gnss->gal->obs_count); // Galileo satellite flags
	if (gnss->bds != NULL)
		pony_gnss_io_rinex_drop_flags_pony_sats(gnss->bds->sat, gnss->bds->max_sat_count, gnss->bds->obs_count); // BeiDou satellite flags
	pony_gnss_io_rinex_drop_flags_pony_sol( &(gnss->sol) ); // solution flags

	for (i = 0; i < n && !feof(fp); i++) {
		fgets(buf,PONY_GNSS_IO_RINEX_BUFFER_SIZE,fp);

		if      (buf[0] == gps_sys_id[0] && gnss->gps != NULL 
			&& !pony_gnss_io_rinex_obs_file_record_parse_obs_line(gnss->gps->sat,  buf, PONY_GNSS_IO_RINEX_BUFFER_SIZE, gnss->gps->max_sat_count, gnss->gps->obs_count) )
				break;
		else if (buf[0] == glo_sys_id[0] && gnss->glo != NULL
			&& !pony_gnss_io_rinex_obs_file_record_parse_obs_line(gnss->glo->sat,  buf, PONY_GNSS_IO_RINEX_BUFFER_SIZE, gnss->glo->max_sat_count, gnss->glo->obs_count) )
				break;
		else if (buf[0] == gal_sys_id[0] && gnss->gal != NULL 
			&& !pony_gnss_io_rinex_obs_file_record_parse_obs_line(gnss->gal->sat,  buf, PONY_GNSS_IO_RINEX_BUFFER_SIZE, gnss->gal->max_sat_count, gnss->gal->obs_count) )
				break;
		else if (buf[0] == bds_sys_id[0] && gnss->bds != NULL 
			&& !pony_gnss_io_rinex_obs_file_record_parse_obs_line(gnss->bds->sat,  buf, PONY_GNSS_IO_RINEX_BUFFER_SIZE, gnss->bds->max_sat_count, gnss->bds->obs_count) )
				break;
		else
			continue;

	}
	if (i < n) // not enough satellite data collected
		return 0;

	return 1;
}

char pony_gnss_io_rinex_obs_file_record_parse_obs_line(pony_gnss_sat *sat,  char *buf, const size_t len, const size_t sat_count, const size_t obs_count) {

	const size_t obs_width = 14,	LLI_width = 1,		SS_width = 1,		rec_width = obs_width+LLI_width+SS_width, sat_id_width = 3;
	const char	 obs_fmt[] = "%lf",	LLI_fmt[] = "%d",	SS_fmt[] = "%d";

	size_t i, k, n, n1, dk;
	unsigned int sn;
	int scanned;
	int LLI, SS;
	char c;
	
	for (n = 0; n < len && buf[n]; n++);
	if (n < sat_id_width)
		return 0;

	c = buf[sat_id_width];
	buf[sat_id_width] = 0;
	scanned = sscanf(buf, "%*c%2u", &sn);
	buf[sat_id_width] = c;
	if (scanned < 1 || sn == 0 || sn > sat_count)
		return 0;

	n1 = n - obs_width;
	for (i = 0, k = sat_id_width; i < obs_count && k < n1; i++, k += rec_width) {
		scanned = 0;
		// parse observation field
		scanned = pony_gnss_io_rinex_v3_obs_file_record_read_parse_entry( buf+k   , obs_width, obs_fmt, (void *)&(sat[sn-1].obs[i]) );
		dk = obs_width;
		if (scanned == 1)
			sat[sn-1].obs_valid[i] = 1;
		// parse LLI (loss of lock indicator) field
		if (k+dk >= n)
			break;
		LLI = 0;
		scanned = pony_gnss_io_rinex_v3_obs_file_record_read_parse_entry( buf+k+dk, LLI_width, LLI_fmt, (void *)&LLI );
		dk += LLI_width;
		if (k+dk >= n)
			break;
		if (scanned == 1 && LLI)
			sat[sn-1].obs_valid[i] = 0;
		// parse SS (signal strength) field, not analyzed yet
		SS = 0;
		scanned = pony_gnss_io_rinex_v3_obs_file_record_read_parse_entry( buf+k+dk, SS_width, SS_fmt, (void *)&SS );
	}

	return 1;

}

int pony_gnss_io_rinex_v3_obs_file_record_read_parse_entry(char *buf, const size_t width, const char *fmt, void *dest) {

	char c;
	int scanned;

	c = buf[width];
	buf[width] = '\0';
	scanned = sscanf(buf, fmt, dest);
	buf[width] = c;

	return scanned;
}


	// ephemeris file header
char pony_gnss_io_rinex_eph_file_header_read(pony_gnss *gnss, FILE *fp, char *buf) {

	enum system_id          {gps, glo, gal, bds, sys_count};

	const char   sys_id[] = {'G', 'R', 'E', 'C'}; 
	const size_t label_offset = 61, label_len = 20, type_offset = 20, sys_offset = 40;

	// mandatory label: RINEX version and type
	const char  rinex_version_type_label[] = "RINEX VERSION / TYPE";
	const float rinex_version_range[] = {2.10f, 3.04f};
	const float rinex_version_prec = 0.001f;
	const char  rinex_type_token = 'N';
	// mandatory label: end of header
	const char rinex_EOH_label [] = "END OF HEADER"  ;
	// mandatory labels count
	const size_t tokens_required = 2;
	// optional labels
	const char rinex_iono_corr_label[] = "IONOSPHERIC CORR";
	const char rinex_time_corr_label[] = "TIME SYSTEM CORR";
	const char rinex_leap_sec_label[] = "LEAP SECONDS";
	
	float version;
	char  rinex_type_sys;
	size_t i, tokens_found = 0;


	if ( feof(fp) )
		return 0;

	// check starting label
	while ( !feof(fp) ) {
		// read a new string
		fgets(buf, PONY_GNSS_IO_RINEX_BUFFER_SIZE, fp);
		for (i = 0; buf[i] >= ' '; i++); buf[i] = '\0';		// set end of line on the first non-printable character
		if (i < label_offset) continue;						// check if buffer contains enough characters to proceed

		// check if a starting token is found
		if ( pony_gnss_io_rinex_find_token(rinex_version_type_label,buf+label_offset-1,label_len) ) {
			printf("\n\t\t%s",buf);
			if (sscanf(buf,"%f",&version) < 1) {
				printf("\n\t\terror: RINEX header line %s not parsed from %s", rinex_version_type_label, buf);
				return 0;
			}
			if (version+rinex_version_prec < rinex_version_range[0] || rinex_version_range[1] < version-rinex_version_prec) {
				printf("\n\t\terror: RINEX version out of range: header contains %f, must be from %f to %f", version, rinex_version_range[0], rinex_version_range[1]);
				return 0;
			}
			if (buf[type_offset] != rinex_type_token) {
				printf("\n\t\terror: RINEX observation file type incorrect: %s, must start with '%c'", buf + type_offset, rinex_type_token);
				return 0;
			}
			rinex_type_sys = buf[sys_offset];
			tokens_found++;
			break;
		}
	}

	// check other labels
	while ( !feof(fp) ) {

		fgets(buf, PONY_GNSS_IO_RINEX_BUFFER_SIZE-1, fp);
		for (i = 0; buf[i] >= ' '; i++); buf[i] = '\0';		// set end of line on the first non-printable character
		if (i < label_offset) continue;						// check if buffer contains enough characters to proceed

		// check ionospheric correction
		if ( pony_gnss_io_rinex_find_token(rinex_iono_corr_label,buf+label_offset-1,label_len) ) {
			printf("\n\t%s",buf);
			if ( !pony_gnss_io_rinex_eph_file_header_parse_iono_corr(gnss, buf, label_offset-1) )
				printf("\n\t\twarning: ionospheric correction not parsed from '%s'",buf);
			continue;
		}

		// check time system correction
		if ( pony_gnss_io_rinex_find_token(rinex_time_corr_label,buf+label_offset-1,label_len) ) {
			printf("\n\t%s",buf);
			if ( !pony_gnss_io_rinex_eph_file_header_parse_time_corr(gnss, buf, label_offset-1) )
				printf("\n\t\twarning: time system correction not parsed from '%s'",buf);
			continue;
		}

		// check leap seconds
		if ( pony_gnss_io_rinex_find_token(rinex_leap_sec_label,buf+label_offset-1,label_len) ) {
			printf("\n\t%s",buf);
			if ( !pony_gnss_io_rinex_file_header_parse_leap_sec(gnss, buf, label_offset-1, rinex_type_sys == sys_id[bds]) )
				printf("\n\t\twarning: leap seconds not parsed from '%s'",buf);
			continue;
		}

		// check end of header
		if ( pony_gnss_io_rinex_find_token(rinex_EOH_label,buf+label_offset-1,label_len) ) {
			printf("\n\t%s",buf);
			tokens_found++;   
			break;
		}

	}

	return (tokens_found == tokens_required)? 1 : 0;

}

		// ephemeris file header ionospheric correction
char pony_gnss_io_rinex_eph_file_header_parse_iono_corr(pony_gnss *gnss,  char *buf, const size_t len) {
	
	const char gpsa_id[]		= "GPSA", gpsb_id[] =	"GPSB", gal_id[] =	"GAL", bdsa_id[] =	"BDSA", bdsb_id[] =	"BDSB";
	const size_t id_width[]		= {4,					4,					3,					4,					4};
	const size_t field_count[]	= {4,					4,					3,					4,					4};
	
	if (pony_gnss_io_rinex_find_token(gpsa_id, buf, id_width[0]) && gnss->gps != NULL)
		if ( !pony_gnss_io_rinex_eph_file_header_parse_iono_corr_line(gnss->gps->iono_a,  buf+id_width[0]+1, len-id_width[0], field_count[0]) )
			return 0;
	if (pony_gnss_io_rinex_find_token(gpsb_id, buf, id_width[1]) && gnss->gps != NULL)
		if ( pony_gnss_io_rinex_eph_file_header_parse_iono_corr_line(gnss->gps->iono_b,  buf+id_width[1]+1, len-id_width[1], field_count[1]) )
			gnss->gps->iono_valid = 1;
		else
			return 0;
	if (pony_gnss_io_rinex_find_token(gal_id, buf, id_width[2]) && gnss->gal != NULL)
		if ( pony_gnss_io_rinex_eph_file_header_parse_iono_corr_line(gnss->gal->iono,  buf+id_width[2]+1, len-id_width[2], field_count[2]) )
			gnss->gal->iono_valid = 1;
		else
			return 0;
	if (pony_gnss_io_rinex_find_token(bdsa_id, buf, id_width[0]) && gnss->bds != NULL)
		if ( !pony_gnss_io_rinex_eph_file_header_parse_iono_corr_line(gnss->bds->iono_a,  buf+id_width[0]+1, len-id_width[0], field_count[0]) )
			return 0;
	if (pony_gnss_io_rinex_find_token(bdsb_id, buf, id_width[1]) && gnss->bds != NULL)
		if ( pony_gnss_io_rinex_eph_file_header_parse_iono_corr_line(gnss->bds->iono_b,  buf+id_width[1]+1, len-id_width[1], field_count[1]) )
			gnss->bds->iono_valid = 1;
		else
			return 0;

	return 1;
}

		// ephemeris file header ionospheric correction - single line
char pony_gnss_io_rinex_eph_file_header_parse_iono_corr_line(double *iono,  char *buf, const size_t len, const size_t field_count) {

	const size_t field_width = 12;

	size_t i, j, n;
	int scanned;
	char c;

	for (n = 0; n < len && buf[n]; n++);
	for (i = 0, j = 0; i < field_count && buf[j] && j < n; i++) {
		if (j > n-field_width)
			break;
		c = buf[j+field_width];
		buf[j+field_width] = '\0';

		pony_gnss_io_rinex_charrep(buf+j, 'D', 'E'); pony_gnss_io_rinex_charrep(buf+j, 'd', 'e');
		scanned = sscanf( buf+j, "%lg", &(iono[i]) );
		if (scanned < 1)
			break;

		j += field_width;
		buf[j] = c;
	}

	return (i == field_count) ? 1 : 0;
}

		// ephemeris file header time system correction
char pony_gnss_io_rinex_eph_file_header_parse_time_corr(pony_gnss *gnss,  char *buf, const size_t len) {

	const char gps_time_id[] = "GP", glo_time_id[] = "GL", gal_time_id[] = "GA", bds_time_id[] = "BD";
	const size_t time_id_len = 2;

	size_t i, i1, n;

	for (n = 0; n < len && buf[n]; n++);
	i = 0;
	for (; buf[i] <= ' ' && i < n-time_id_len; i++); // skip non-printables
	if (i >= n)
		return 0;
	i1 = i + time_id_len;
	if (pony_gnss_io_rinex_find_token(gps_time_id, buf+i, n-i) && gnss->gps != NULL) // apply to gps
		pony_gnss_io_rinex_eph_file_header_parse_time_corr_line(gnss->gps->clock_corr, gnss->gps->clock_corr_to, &(gnss->gps->clock_corr_valid),  buf+i1, n-i1);
	if (pony_gnss_io_rinex_find_token(glo_time_id, buf+i, n-i) && gnss->glo != NULL) // apply to glonass
		pony_gnss_io_rinex_eph_file_header_parse_time_corr_line(gnss->glo->clock_corr, gnss->glo->clock_corr_to, &(gnss->glo->clock_corr_valid),  buf+i1, n-i1);
	if (pony_gnss_io_rinex_find_token(gal_time_id, buf+i, n-i) && gnss->gal != NULL) // apply to galileo
		pony_gnss_io_rinex_eph_file_header_parse_time_corr_line(gnss->gal->clock_corr, gnss->gal->clock_corr_to, &(gnss->gal->clock_corr_valid),  buf+i1, n-i1);
	if (pony_gnss_io_rinex_find_token(bds_time_id, buf+i, n-i) && gnss->bds != NULL) // apply to beidou
		pony_gnss_io_rinex_eph_file_header_parse_time_corr_line(gnss->bds->clock_corr, gnss->bds->clock_corr_to, &(gnss->bds->clock_corr_valid),  buf+i1, n-i1);

	return 1;
}

		// ephemeris file header time system correction - single line
void pony_gnss_io_rinex_eph_file_header_parse_time_corr_line(double *clock_corr, char *time_sys, char *valid,  char *buf, const size_t len) {

	const size_t time_id_len = 2, clock_corr_terms_count = 4, clock_corr_terms_len[] = {17, 16, 7, 5};

	size_t i, j, n;
	int scanned;
	char c;

	for (n = 0; n < len && buf[n]; n++);
	for (i = 0, j = 0; j < time_id_len && buf[i] && i < n; j++, i++)
		time_sys[j] = buf[i];
	if (i >= n-1)
		return;

	i++; // skip one position

	for (j = 0; j < clock_corr_terms_count && buf[i]; j++) {
		if (i > n-clock_corr_terms_len[j])
			break;
		c = buf[i+clock_corr_terms_len[j]];
		buf[i+clock_corr_terms_len[j]] = '\0';

		pony_gnss_io_rinex_charrep(buf+i, 'D', 'E'); pony_gnss_io_rinex_charrep(buf+i, 'd', 'e');
		scanned = sscanf(buf+i, "%lg", &(clock_corr[j]));
		if (scanned && j == 1)
			*valid = 1;

		i += clock_corr_terms_len[j];
		buf[i] = c;
	}
}

	// file header leap seconds
char pony_gnss_io_rinex_file_header_parse_leap_sec(pony_gnss *gnss,  char *buf, const size_t len, char bds_only) {

	enum format {cur_leap_sec = 0, future_past_leap_sec = 6, week = 12, day = 18, time_id = 24, end}; // format according to RINEX Version 3.03 Table A2 & Table A5 "LEAP SECONDS"

	const char gps_id[] = {'G','P','S'}, // GPS time system identifier according to RINEX Version 3.03 Table A2 & Table A5 "LEAP SECONDS"
	           bds_id[] = {'B','D','S'}; // BDS time system identifier according to RINEX Version 3.03 Table A2 & Table A5 "LEAP SECONDS"

	size_t i, n;
	int scanned;
	char c;


	// check string length
	for (n = 0; n < len && buf[n]; n++);
	if (n < future_past_leap_sec) // no room for current leap second field
		return 0;
	// parse current leap seconds
	c = buf[future_past_leap_sec];    // store character after the end of the field
	buf[future_past_leap_sec] = '\0'; // limit parsing to the beginning of the next field
	scanned = sscanf(buf+cur_leap_sec, "%d", &(gnss->leap_sec) );
	buf[future_past_leap_sec] = c;    // restore replaced character
	if (scanned == 1 && gnss->leap_sec > 0) // check if the value is parsed and valid
		gnss->leap_sec_valid = 1;
	else {
		gnss->leap_sec_valid = 0;
		return 0;
	}
	// check for GPS system id
	if (n >= time_id + sizeof(gps_id)) {
		for (i = 0; i < sizeof(gps_id); i++)
			if (buf[time_id+i] != gps_id[i])
				break; // no GPS time system id detected
		if (i >= sizeof(gps_id)) // GPS time system id detected
			return 1;
	}
	// check for BDS system id
	if (n >= time_id + sizeof(bds_id)) {
		for (i = 0; i < sizeof(bds_id); i++)
			if (buf[time_id+i] != bds_id[i])
				break; // no BDS time system id detected
		if (i >= sizeof(bds_id)) {// BDS time system id detected
			gnss->leap_sec += (int)(pony->gnss_const.bds.leap_sec);
			return 1;
		}
	}
	// system id is blank (see Note 2 for Table A2 & Table A5 "LEAP SECONDS" in RINEX Version 3.03)
	if (bds_only)
		gnss->leap_sec += (int)(pony->gnss_const.bds.leap_sec);

	return 1;
}

	// ephemeris file records
void pony_gnss_io_rinex_eph_file_records_read(
	pony_gnss_sat *sat, pony_time_epoch *gnss_epoch, 
	FILE *fp, char *buf, double *eph, unsigned int *sn, 
	const size_t sys, const size_t maxsat, const size_t maxeph) 
{

	enum sys_index {gps, glo, gal, bds, sys_count};
	const char sys_id[sys_count+1] = "GREC"; // GNSS system identifiers: GPS, GLONASS, Galileo, BeiDou, see RINEX documentation
	const size_t														// gps glo gal bds
				sys_record_lines							[sys_count] = { 7,  3,  7,  7}, 
				sys_total_eph								[sys_count] = {35, 21, 34, 35};
	const int	sys_update_older_ephemeris_if_less_than_min	[sys_count] = {61, 16,  6, 31};
	const size_t sat_id_width = 3, header_eph_count = 9;

	size_t i;
	int scanned = 0;
	char refepoch_flag;	// 1 if reference epoch exists, 0 otherwise 
	pony_time_epoch epoch, satepoch;

	// safety check
	if (sat == NULL || gnss_epoch == NULL || fp == NULL || buf == NULL || eph == NULL || sn == NULL)
		return;
	
	while ( !feof(fp) ) { // read until end of file occurs

		// check if eph buffers contain correct ephemeris, to use them for updating satellite data
		refepoch_flag = pony_gnss_io_rinex_epoch_check(gnss_epoch);
		if ( pony_gnss_io_rinex_epoch_from_array(&epoch, eph) )
			if ( *sn > 0 && *sn <= maxsat && ( 
				   !pony_gnss_io_rinex_epoch_from_array(&satepoch, sat[*sn-1].eph) 
				|| (refepoch_flag && pony_time_epochs_compare(&satepoch, gnss_epoch) < 0)

				// temporary measure(?) if there are e.g. two sets for the same GPS sat with less than 1 hour between them, say 11:59:44 and 12:00:00
				|| (abs( (satepoch.h*60+satepoch.m)-(epoch.h*60+epoch.m) ) <  sys_update_older_ephemeris_if_less_than_min[sys]                          )
				|| (abs( (satepoch.h*60+satepoch.m)-(epoch.h*60+epoch.m) ) > -sys_update_older_ephemeris_if_less_than_min[sys]+pony->gnss_const.sec_in_d)

				) ) {
				for (i = 0; i < maxeph; i++)
					sat[*sn-1].eph[i] = eph[i];
				sat[*sn-1].eph_valid = 1;
			}
			else if (refepoch_flag && pony_time_epochs_compare(&satepoch, gnss_epoch) >= 0 && pony_time_epochs_compare(&satepoch, &epoch) < 0)
				break;

		// wait for system identifier 
		for (buf[0] = '\0'; buf != NULL && buf[0] != sys_id[sys]; )
			buf = fgets(buf,PONY_GNSS_IO_RINEX_BUFFER_SIZE-1,fp);
		if (buf == NULL)
			break;
		
		// try to parse the sn next to the system identifier
		scanned = sscanf(buf,"%*c%2u", sn);
		if (scanned < 1 || *sn == 0 || *sn > maxsat)
			continue; // something is wrong

		for (i = 0; i < maxeph; i++) // reset ephemeris in the buffer
			eph[i] = 0.0;
		
		if ( !pony_gnss_io_rinex_eph_file_parse_record_header(eph, buf+sat_id_width, PONY_GNSS_IO_RINEX_BUFFER_SIZE-sat_id_width-1) // try to parse the first line
			 || !pony_gnss_io_rinex_epoch_from_array(&epoch, eph) 
			 || pony_gnss_io_rinex_eph_file_parse_record_lines(eph+header_eph_count,  fp, buf, sys_record_lines[sys]) < sys_total_eph[sys] // parse the rest of the lines (system-dependent, see RINEX documentation)
			 )
			continue;
	}

}

		// ephemeris file record header
char pony_gnss_io_rinex_eph_file_parse_record_header(double *eph, char *buf, const size_t len) {

	const size_t field_count = 9;
	const size_t field_width[] = {5, 3, 3, 3, 3, 3,  19, 19, 19};

	size_t i, k, n;
	int scanned;
	char c;

	for (n = 0; n < len && buf[n]; n++);
	pony_gnss_io_rinex_charrep(buf, 'D', 'E'); pony_gnss_io_rinex_charrep(buf, 'd', 'e');
	for (i = 0, k = 0; i < field_count; i++) {
		if (k >= n-field_width[i] || buf[k] == '\0')
			break;
		c = buf[k+field_width[i]];
		buf[k+field_width[i]] = '\0';

		scanned = sscanf( buf+k, "%lg", &(eph[i]) );
		if (scanned < 1)
			break;

		k += field_width[i];
		buf[k] = c;
	}

	return (i == field_count)? 1 : 0;
}

		// ephemeris file single record
size_t pony_gnss_io_rinex_eph_file_parse_record_lines(double *eph,  FILE *fp, char *buf, const size_t lines) {

	const size_t rec_per_line = 4, rec_width = 19, blank_width = 4;

	size_t i, k, line, n, total;
	int scanned;
	char c;

	for (line = 1, total = 0; line <= lines; line++) {
		fgets(buf,PONY_GNSS_IO_RINEX_BUFFER_SIZE-1,fp);
		for (n = 0; buf[n] != '\0'; n++); // determine total length
		pony_gnss_io_rinex_charrep(buf, 'D', 'E'); pony_gnss_io_rinex_charrep(buf, 'd', 'e');
		for (i = 0, k = blank_width; i < rec_per_line; i++) {
			if (k >= n-rec_width || buf[k] == '\0')
				break;
			c = buf[k+rec_width];
			buf[k+rec_width] = '\0';

			scanned = sscanf( buf+k, "%lg", &(eph[total]) );
			if (scanned < 1)
				break;

			total++;
			k += rec_width;
			buf[k] = c;
		}
		if (line < lines && i < rec_per_line)
			break;
	}
	return total;

}






// service routines
void pony_gnss_io_rinex_drop_flags_pony_sats(pony_gnss_sat *sat, const size_t sat_count, const size_t obs_count) {

	size_t i, s;

	for (s = 0; s < sat_count; s++) {
		sat[s].t_em_valid = 0;
		sat[s].x_valid = 0;
		sat[s].v_valid = 0;
		for (i = 0; i < obs_count; i++)
			sat[s].obs_valid[i] = 0;
	}

}

void pony_gnss_io_rinex_drop_flags_pony_sol(pony_sol *sol) {

	sol->  x_valid = 0;
	sol->llh_valid = 0;
	sol->  v_valid = 0;
	sol->  q_valid = 0;
	sol->  L_valid = 0;
	sol->rpy_valid = 0;
	sol-> dt_valid = 0;

}




		// free memory with NULL-check and NULL-assignment
void pony_gnss_io_rinex_free_null(void **ptr) {

	if (*ptr == NULL)
		return;
	
	free(*ptr);
	*ptr = NULL;

}





void pony_gnss_io_rinex_charrep(char *s, char oldc, char newc) {
	size_t i;

	for (i = 0; s[i]; i++)
		if (s[i] == oldc)
			s[i] = newc;
}

char pony_gnss_io_rinex_read_token_str(char *value,  const char *token, const char *src, const size_t len) {

	const char delim = '=', quote = '"', brace_open = '{', brace_close = '}';

	size_t i, j, k, n, len1;
	
	value[0] = 0;

	for (n = 0; token[n]; n++); // determine token length
	if (n == 0)
		return 0;
	len1 = len - n;

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
	for (i = k+1; i < len && src[i] && src[i] <= ' '; i++); // skip all non-printables
	if (i >= len || src[i] != delim) // no delimiter found
		return 0; 

	for (i++; i < len && src[i] && src[i] <= ' '; i++); // skip the delimiter and all non-printables
	if (i >= len || src[i] != quote) // no opening quote found
		return 0;

	for (i++, j = 0; i < len && src[i] && src[i] != quote; i++, j++) // copy until the closing quote appears
		value[j] = src[i];
	value[j] = 0;
	if (i >= len) // no closing quote found
		return 0;

	return 1;

}

char pony_gnss_io_rinex_read_token_double(double *dest, const char *token, const char *src, const size_t len) {

	const char delim = '=', quote = '"', brace_open = '{', brace_close = '}';
	size_t i, j, n;
	char token_found = 0;

	for (n = 0; n < len && src[n]; n++);
	for (i = 0; !token_found && src[i] && i < n; ) {
		if (src[i] == quote) // skip quoted values
			for (i++; src[i] && i < n && src[i] != quote; i++);
		else if (src[i] == brace_open) // skip groups
			for (i++; src[i] && i < n && src[i] != brace_close; i++);
		else {
			j = 0;
			if (src[i] == token[j]) {
				token_found = 1;
				for (i++,j++; src[i] && token[j] && i < n; i++,j++)
					if (src[i] != token[j]) {
						token_found = 0;
						break;
					}
				if (token[j])
					token_found = 0;
			}
			else
				i++;
		}
	}
	if (!token_found)
		return 0;

	for (i++; i < n && src[i] && src[i] <= ' '; i++); // skip all non-printables
	if (!src[i] || i == n || src[i] != delim)
		return 0; // no delimiter found

	for (i++; i < n && src[i] && src[i] <= ' '; i++); // skip the delimiter and all non-printables
	if (!src[i] || i == n)
		return 0; // no token value found

	if (sscanf(src+i,"%lg",dest) < 1)	// try to scan double value
		return 0;							// failed to scan
	else
		return 1;							// success

}

char pony_gnss_io_rinex_find_token(const char *token, const char *src, const size_t len) {

	size_t i, j = 0, k, n, len1;

	for (n = 0; token[n]; n++); // determine token length

	for (i = 0, len1 = len-n+1; src[i] && i < len1; i++) {
		for (j = 0, k = i; j < n && src[k] == token[j]; j++, k++);
		if (j == n) // all characters matched
			return 1;
	}

	return 0;

}

char pony_gnss_io_rinex_epoch_from_array(pony_time_epoch *epoch, double *YMDhms) {

	epoch->Y = pony_gnss_io_rinex_resolve_2digit_year( (int)(YMDhms[0]) );
	epoch->M = (int)(YMDhms[1]);
	epoch->D = (int)(YMDhms[2]);
	epoch->h = (int)(YMDhms[3]);
	epoch->m = (int)(YMDhms[4]);
	epoch->s = YMDhms[5];

	return pony_gnss_io_rinex_epoch_check(epoch);
}

char pony_gnss_io_rinex_epoch_check(pony_time_epoch *epoch) {
	const double time_precision = 1./(0x01<<5); // 1/32 sec, half precision step between 32 and 64
	return (
		   epoch->Y <  0					|| (100 <= epoch->Y && epoch->Y < 1980)
		|| epoch->M <= 0					|| epoch->M > 12
		|| epoch->D <= 0					|| epoch->D > 31
		|| epoch->h <  0					|| epoch->h > 23
		|| epoch->m <  0					|| epoch->m > 59
		|| epoch->s <  0.0-time_precision	|| epoch->s > 60.0+time_precision)? 0 : 1;
}

int  pony_gnss_io_rinex_resolve_2digit_year(int Y) {
	if (Y >= 100 || Y < 0)
		return Y;

	if (Y >= 80)
		return Y + 1900;
	else
		return Y + 2000;
}

double pony_gnss_io_rinex_dmod(double x, double y) {

    return x - (int)(x/y) * y;
}
