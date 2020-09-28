// Sep-2020
/*	pony_gnss_io_ublox 
	
	pony plugins for GNSS u-Blox receiver input/output:

	- pony_gnss_io_ublox_read_file 
		Reads raw observation and navigation data (measurements and ephemeris) from u-Blox binary files.
		Processes messages RXM-RAW/RXM-EPH (GPS L1-only) and RXM-RAWX/RXM-SFRBX
		Tested for UBX protocol versions 6, M8T, F9T/F9P.
		Supports GPS, GLONASS, Galileo and BeiDou systems.
		Multi-receiver/multi-antenna capable, with syncronization option.
	
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "../../pony.h"

// pony bus version check
#define PONY_GNSS_IO_UBLOX_BUS_VERSION_REQUIRED 8
#if PONY_BUS_VERSION < PONY_GNSS_IO_UBLOX_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif

// UBX signal ID conversion table
#define PONY_GNSS_IO_UBLOX_MAX_SIGNAL_TYPES_PER_SYS 4
typedef struct // gnss signal id conversion table entry
{
	unsigned char gnss_id;
	size_t        types;
	unsigned char ubx_sig_id[PONY_GNSS_IO_UBLOX_MAX_SIGNAL_TYPES_PER_SYS];
	char          rnx_sig_id[PONY_GNSS_IO_UBLOX_MAX_SIGNAL_TYPES_PER_SYS][2];
} pony_gnss_io_ublox_signal_id_converson_table_struct;

const pony_gnss_io_ublox_signal_id_converson_table_struct pony_gnss_io_ublox_signal_id_conversion_table[] = {
	{   // gps
		0,
		3, 
		{  0,        3,        4      },
		{{'1','C'},{'2','L'},{'2','M'}}
	},{ // glo
		6,
		2, 
		{  0,        2},
		{{'1','C'},{'2','C'}}
	},{ // gal
		2,
		4, 
		{  0,        1,        5,        6      },
		{{'1','C'},{'1','B'},{'7','I'},{'7','Q'}}
	},{ // bds
		3,
		4, 
		{  0,        1,        2,        3      },
		{{'1','I'},{'2','I'},{'7','I'},{'7','D'}}
	}
};

// message data structures
	// RXM-RAWX: multi-gnss raw measurement data, as in u-blox ZED-F9P Interface Description UBX-18010854-R07 p. 182 / u-blox ZED-F9T Interface Description UBX-18053584-R02 p. 174
typedef struct {
	double			rcvTow;
	unsigned short	week;
	  signed char	leapS;
	unsigned char	numMeas;
	unsigned char	recStat;
	unsigned char	version;
	unsigned char	reserved1[2];
} pony_gnss_io_ublox_RXM_RAWX_header;
typedef struct {
	         double	prMes;
	         double	cpMes;
	         float	doMes;
	unsigned char	gnssId;
	unsigned char	  svId;
	unsigned char	 sigId;
	unsigned char	freqId;
	unsigned short	locktime;
	unsigned char	cno;
	unsigned char	prStdev;
	unsigned char	cpStdev;
	unsigned char	doStdev;
	unsigned char	trkStat;
	unsigned char	reserved2;
} pony_gnss_io_ublox_RXM_RAWX_block;
	// RXM-RAW: GPS L1 raw measurement data, as in u-blox 6 Receiver Description GPS.G6-SW-10018-F p. 185
typedef struct {
	  signed long  int  iTOW;
	  signed short int  week;
	unsigned       char numSV;
	unsigned       char reserved1;
} pony_gnss_io_ublox_RXM_RAW_header;
typedef struct {
	         double cpMes;
	         double prMes;
	         float  doMes;
	unsigned char   sv;
	  signed char   mesQI;
	  signed char   cno;
	unsigned char   lli;
} pony_gnss_io_ublox_RXM_RAW_block;
	// RXM-SFRBX: multi-GNSS broadcast navigation data subframe, as in u-blox ZED-F9P Interface Description UBX-18010854-R07 p. 187 / u-blox ZED-F9T Interface Description UBX-18053584-R02 p. 182
typedef struct {
	unsigned char	gnssId;
	unsigned char	  svId;
	unsigned char	reserved1;
	unsigned char	freqId;
	unsigned char	numWords;
	unsigned char	chn;
	unsigned char	version;
	unsigned char	reserved2;
} pony_gnss_io_ublox_RXM_SFRBX_header;
typedef struct {
	unsigned long   dwrd[10];
} pony_gnss_io_ublox_RXM_SFRBX_block;
	// RXM-EPH: GPS aiding ephemeris output message, as in u-blox 6 Receiver Description GPS.G6-SW-10018-F p. 184
typedef struct {
	unsigned long	svid;
	unsigned long	how;
} pony_gnss_io_ublox_RXM_EPH_header;
typedef struct {
	unsigned long	sf[3][8];
} pony_gnss_io_ublox_RXM_EPH_block;
	

// navigation subframe parsing
typedef struct {
	size_t			wrd;	// word number (starting with 0)
	unsigned char	shift;	// bit shift within the word
	unsigned char	bits;	// bit count
	size_t			msb;	// index of most significant bits entry (scale and eph ignored there), or UINT_MAX if there are none
	double			scale;	// scale factor (LSB), negative for signed numbers
	size_t			eph;	// ephemeris array index (according to RINEX, starting with 0 for epoch year)
} pony_gnss_io_ublox_nav_subframe_conversion_entry;
char pony_gnss_io_ublox_nav_subframe_parse_data(double *eph, unsigned long *dwrd, pony_gnss_io_ublox_nav_subframe_conversion_entry *conversion_table, const size_t entries_count, const char complement);
char pony_gnss_io_ublox_nav_subframe_parser_gps(pony_gnss_sat *sat, pony_gnss_io_ublox_RXM_SFRBX_block *subframe);
char pony_gnss_io_ublox_nav_subframe_parser_glo(pony_gnss_sat *sat, pony_gnss_io_ublox_RXM_SFRBX_block *subframe);
char pony_gnss_io_ublox_nav_subframe_parser_gal(pony_gnss_sat *sat, pony_gnss_io_ublox_RXM_SFRBX_block *subframe);
char pony_gnss_io_ublox_nav_subframe_parser_bds(pony_gnss_sat *sat, pony_gnss_io_ublox_RXM_SFRBX_block *subframe);

// message parsers
size_t pony_gnss_io_ublox_file_read_message(pony_gnss *gnss, FILE *fp, void *hdr_buf, void *blk_buf); // read message from stream
char   pony_gnss_io_ublox_file_parse_RXM_RAWX (unsigned char *cs, pony_gnss *gnss, FILE *fp, void *hdr, void *blk); // RXM-RAWX : multi-gnss raw measurement data
char   pony_gnss_io_ublox_file_parse_RXM_RAW  (unsigned char *cs, pony_gnss *gnss, FILE *fp, void *hdr, void *blk); // RXM-RAW  : GPS L1 raw measurement data
char   pony_gnss_io_ublox_file_parse_RXM_SFRBX(unsigned char *cs, pony_gnss *gnss, FILE *fp, void *hdr, void *blk); // RXM-SFRBX: multi-GNSS broadcast navigation data subframe
char   pony_gnss_io_ublox_file_parse_RXM_EPH  (unsigned char *cs, pony_gnss *gnss, FILE *fp, void *hdr, void *blk); // RXM-EPH  : GPS aiding ephemeris output message

// internal routines
void   pony_gnss_io_ublox_glonass_day2date(pony_time_epoch *epoch, unsigned int day);								// GLONASS day into 4-year cycle to date closest to given year
void   pony_gnss_io_ublox_checksum_recurse(unsigned char *cs,  unsigned char byte);									// UBX recursive checksum
char   pony_gnss_io_ublox_obs_types_allocate(char (**obs_types)[4], size_t *obs_count, pony_gnss_sat *sat, const size_t sat_count, const pony_gnss_io_ublox_signal_id_converson_table_struct *sig); // observation types handling
void   pony_gnss_io_ublox_drop_flags_pony_sats(pony_gnss_sat *sat, const size_t sat_count, const size_t obs_count);	// drop satellite flags
void   pony_gnss_io_ublox_drop_flags_pony_sol(pony_sol *sol);														// drop solution flags
void   pony_gnss_io_ublox_free_null(void **ptr);																	// memory release with NULL-check and NULL-asignment 
int    pony_gnss_io_ublox_round(double x);																			// round to the nearest integer
double pony_gnss_io_ublox_dmod(double x, double y);																	// remainder after division for doubles














// plugin definitions

/* pony_gnss_io_ublox_read_file - pony plugin
	
	Reads raw observation and navigation data (measurements and ephemeris) from u-Blox binary files.
	Processes messages RXM-RAW/RXM-EPH (GPS L1-only) and RXM-RAWX/RXM-SFRBX
	Tested for UBX protocol versions 6, M8T, F9T/F9P.
	Supports GPS, GLONASS, Galileo and BeiDou systems.
	Multi-receiver/multi-antenna capable, with syncronization option.
	When new issue of navigation data starts, this plugin drops ephemeris validity flag, 
	so satellite stops being used (except for continuous integration in GLONASS).

	uses:
		pony->gnss_count
		pony->gnss[].gps/glo/gal/bds->max_sat_count
		pony->gnss[].gps/glo/gal/bds->max_eph_count
	changes:
		pony->mode
		pony->gnss[].epoch
		pony->gnss[].leap_sec
		pony->gnss[].leap_sec_valid
		pony->gnss[].gps/glo/gal/bds->obs_count
		pony->gnss[].gps/glo/gal/bds->obs_types
		pony->gnss[].gps/glo/gal/bds->sat[]. eph
		pony->gnss[].gps/glo/gal/bds->sat[]. eph_valid
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
		{gnss: ubx_in} - gnss u-blox data input source
			type   : string
			range  : valid file name
			default: none
			example: {gnss: ubx_in = "data/2019_10/191025_0255/base_1025.bin"}
		gnss_sync - multi-receiver observations synchronization threshold, sec
			type   : floating point
			range  : >0
			default: none
			example: gnss_sync = 0.09
			negative values result in sync turned off
*/
void pony_gnss_io_ublox_read_file(void) {

	enum  system_id {gps, glo, gal,	bds, sys_count}; // supported constellations
	// supported messages
	enum ubx_msg_id									  { RXM_RAWX,							RXM_SFRBX,								RXM_RAW,							RXM_EPH, msg_id_count};

	const char cfg_file_token[] = "ubx_in", gnss_sync_token[] = "gnss_sync", quote = '"';
	const size_t payload_hdr_size[msg_id_count]		= { sizeof(pony_gnss_io_ublox_RXM_RAWX_header),	sizeof(pony_gnss_io_ublox_RXM_SFRBX_header),	sizeof(pony_gnss_io_ublox_RXM_RAW_header),	sizeof(pony_gnss_io_ublox_RXM_EPH_header)};
	const size_t payload_blk_size[msg_id_count]		= { sizeof(pony_gnss_io_ublox_RXM_RAWX_block ),	sizeof(pony_gnss_io_ublox_RXM_SFRBX_block ),	sizeof(pony_gnss_io_ublox_RXM_RAW_block ),	sizeof(pony_gnss_io_ublox_RXM_EPH_block )};

	static FILE **fp = NULL;
	static double gnss_sync = -1;
	static void *payload_hdr, *payload_blk;

	size_t r, i, id, count;
	char *str;
	pony_time_epoch latest_epoch = {0,0,0,0,0,0};

	// init
	if (pony->mode == 0)	{

		// requires gnss data initialized
		if (pony->gnss == NULL) {
			pony->mode = -1;
			return;
		}

		// allocate memory
			// file pointers, memory buffers and observation format specifiers
		fp          = (FILE **)calloc( pony->gnss_count, sizeof(FILE *) );
		// memory buffers
		for (i = 0, count = 0; i < msg_id_count; i++)
			if (payload_hdr_size[i] > count)
				count = payload_hdr_size[i];
		payload_hdr = (void *)malloc(count);
		for (i = 0, count = 0; i < msg_id_count; i++)
			if (payload_blk_size[i] > count)
				count = payload_blk_size[i];
		payload_blk = (void *)malloc(count);
		if (
			   fp			== NULL
			|| payload_hdr	== NULL
			|| payload_blk	== NULL	
			) {
			printf("\nERROR: memory allocation failed for ublox data");
			pony->mode = -1;
			return;
		}

		for (r = 0; r < pony->gnss_count; r++) {
			// requires gnss data initialized
			if (pony->gnss[r].cfg == NULL)
				continue;
			// ubx file name from configuration
			str = pony_locate_token(cfg_file_token, pony->gnss[r].cfg_settings, pony->gnss[r].settings_length, '=');
			if ( str == NULL || str[0] == '\0' ) {
				printf("\n\terror: could not find GNSS ubx file name in the configuration for gnss[%d]:\n\t\t'%s'",
					r, pony->gnss[r].cfg_settings);
				pony->mode = -1;
				return;
			}
			for ( ; str < pony->gnss[r].cfg_settings+pony->gnss[r].settings_length && *str && *str <= ' '; str++);
			if (*str != quote || str >= pony->gnss[r].cfg_settings + pony->gnss[r].settings_length || !(*str)) {
				printf("\n\terror: could not parse GNSS ubx file name in the configuration for gnss[%d]:\n\t\t'%s'",
					r, pony->gnss[r].cfg_settings);
				pony->mode = -1;
				return;
			}
			for (str++, i = 0; str+i < pony->gnss[r].cfg_settings+pony->gnss[r].settings_length && str[i] && str[i] != quote; i++);
			if (str+i >= pony->gnss[r].cfg_settings + pony->gnss[r].settings_length || !(str[i])) {
				printf("\n\terror: could not parse GNSS ubx file name in the configuration for gnss[%d]:\n\t\t'%s'",
					r, pony->gnss[r].cfg_settings);
				pony->mode = -1;
				return;
			}
			// open the file
			str[i] = '\0';
			fp[r] = fopen(str,"rb");
			if (fp[r] == NULL) {
				printf("\n\terror: could not open GNSS ubx file '%s' for gnss[%d]",str,r);
				str[i] = quote;
				pony->mode = -1;
				return;
			}
			printf("\n\tubx binary file '%s' opened",str);
			str[i] = quote;
			// observation types
			if (   ( pony->gnss[r].gps !=NULL && !pony_gnss_io_ublox_obs_types_allocate(&(pony->gnss[r].gps->obs_types), &(pony->gnss[r].gps->obs_count), pony->gnss[r].gps->sat, pony->gnss[r].gps->max_sat_count, &(pony_gnss_io_ublox_signal_id_conversion_table[gps])) )
				|| ( pony->gnss[r].glo !=NULL && !pony_gnss_io_ublox_obs_types_allocate(&(pony->gnss[r].glo->obs_types), &(pony->gnss[r].glo->obs_count), pony->gnss[r].glo->sat, pony->gnss[r].glo->max_sat_count, &(pony_gnss_io_ublox_signal_id_conversion_table[glo])) )
				|| ( pony->gnss[r].gal !=NULL && !pony_gnss_io_ublox_obs_types_allocate(&(pony->gnss[r].gal->obs_types), &(pony->gnss[r].gal->obs_count), pony->gnss[r].gal->sat, pony->gnss[r].gal->max_sat_count, &(pony_gnss_io_ublox_signal_id_conversion_table[gal])) )
				|| ( pony->gnss[r].bds !=NULL && !pony_gnss_io_ublox_obs_types_allocate(&(pony->gnss[r].bds->obs_types), &(pony->gnss[r].bds->obs_count), pony->gnss[r].bds->sat, pony->gnss[r].bds->max_sat_count, &(pony_gnss_io_ublox_signal_id_conversion_table[bds])) )
				) {
				printf("\n\terror: could not allocate memory for observation types in gnss[%d]",r);
				pony->mode = -1;
				return;
			}
		}

		// check for gnss_sync option
		str = pony_locate_token(gnss_sync_token, pony->cfg_settings, pony->settings_length, '=');
		if ( str != NULL && sscanf(str,"%lf",&gnss_sync) && gnss_sync > 0) 
			printf("\n\n\t GNSS observations will be synchronized by time of day within %.3f msec to the latest one\n", gnss_sync*1e3);
		else
			gnss_sync = -1;

		// no return, try to read the first chunk from UBX stream
	}

	// terminate
	if (pony->mode < 0)	{

		// close files, if not already
		if (fp != NULL) {
			for (r = 0; r < pony->gnss_count; r++)
				if (pony->gnss[r].cfg == NULL)
					continue;
				else if (fp[r] != NULL)
					fclose(fp[r]);
			free(fp);
			fp = NULL;
		}
		// memory release
		pony_gnss_io_ublox_free_null(&(payload_hdr));
		pony_gnss_io_ublox_free_null(&(payload_blk));

		return;

	}

	// regular processing
	else {

		for (r = 0, count = 0; r < pony->gnss_count; r++) {
			if (fp[r] == NULL)
				continue;
			for (id = UINT_MAX; !feof(fp[r]) && !ferror(fp[r]) && id != RXM_RAWX && id != RXM_RAW; id = pony_gnss_io_ublox_file_read_message(&(pony->gnss[r]), fp[r], payload_hdr, payload_blk) );
			count += (id == RXM_RAWX || id == RXM_RAW) ? 1 : 0;
		}
		
		if (!count) // no single measurement parsed
			pony->mode = -1;
		
		if (gnss_sync <= 0) // no observation time skew check
			return;
		
		// try to synchronize observations
			// look for the latest epoch in observations
		for (r = 0; r < pony->gnss_count; r++)
			if (pony->gnss[r].cfg == NULL)
				continue;
			else if (pony_time_epochs_compare(&latest_epoch, &(pony->gnss[r].epoch)) == -1)
				latest_epoch = pony->gnss[r].epoch;
		for (r = 0; r < pony->gnss_count; r++)
			if (pony->gnss[r].cfg == NULL)
				continue;
			else
				while (
					pony_gnss_io_ublox_dmod(
						(       latest_epoch.h*3600.0 +        latest_epoch.m*60 +        latest_epoch.s) - // latest epoch
						(pony->gnss[r].epoch.h*3600.0 + pony->gnss[r].epoch.m*60 + pony->gnss[r].epoch.s),  // receiver epoch
						pony->gnss_const.sec_in_d)                                                          // modulo seconds in a day
					>	
					gnss_sync ) {
					for (id = UINT_MAX; !feof(fp[r]) && !ferror(fp[r]) && id != RXM_RAWX && id != RXM_RAW; id = pony_gnss_io_ublox_file_read_message(&(pony->gnss[r]), fp[r], payload_hdr, payload_blk) );
					if (id != RXM_RAWX && id != RXM_RAW) { // end-of-file or file error before syncronized epoch found
						pony->mode = -1;
						return;
					}
				}
		
	}

}










// message parsers
	// read message from stream
size_t pony_gnss_io_ublox_file_read_message(pony_gnss *gnss, FILE *fp, void *hdr_buf, void *blk_buf) {

	enum ubx_msg_id									  { RXM_RAWX,							 RXM_SFRBX,							RXM_RAW,							RXM_EPH, msg_id_count};

	const unsigned char	msg_id[msg_id_count][2]	=	{{0x02,0x15},							{0x02,0x13},						{0x02,0x10},						{0x02,0x31}	},
						sync_char[] = {0xB5, 0x62};

	static char (*msg_parser[])(unsigned char *, pony_gnss *, FILE *, void *, void *)
													= {pony_gnss_io_ublox_file_parse_RXM_RAWX,		pony_gnss_io_ublox_file_parse_RXM_SFRBX,	pony_gnss_io_ublox_file_parse_RXM_RAW,		pony_gnss_io_ublox_file_parse_RXM_EPH};

	unsigned char cs[2], buf[2];
	size_t id;
	short len;

	if ( feof(fp) || ferror(fp) )
		return UINT_MAX;
	// synchronize
	while( fread((void *)(buf),1,1,fp) && ((0xff & *buf) != (0xff & sync_char[0]))  );
	if ( !(fread((void *)(buf),1,1,fp) && ((0xff & *buf) == (0xff & sync_char[1]))) )
		return UINT_MAX;
	// try to parse frame header
	cs[0] = 0, cs[1] = 0;
	if (   !fread((void *)(buf+0),1,1,fp)
		|| !fread((void *)(buf+1),1,1,fp) ) // message class and id expected
		return UINT_MAX;
	for (id = 0; id < msg_id_count; id++) // check through message list
		if (   (0xff & buf[0]) == msg_id[id][0] 
			&& (0xff & buf[1]) == msg_id[id][1] ) {
			// renew checksum
			pony_gnss_io_ublox_checksum_recurse(cs, buf[0]);
			pony_gnss_io_ublox_checksum_recurse(cs, buf[1]);
			// message length expected
			if ( !fread((void *)(&len),2,1,fp) )
				return UINT_MAX;
			// renew checksum
			pony_gnss_io_ublox_checksum_recurse(cs, 0xff & (len>>0));
			pony_gnss_io_ublox_checksum_recurse(cs, 0xff & (len>>8));
			// try to parse the message & validate checksum
			if (   !(msg_parser[id])(cs, gnss, fp, hdr_buf, blk_buf)
				|| !fread((void *)(buf+0),1,1,fp)   // CK_A checksum expected
				|| !fread((void *)(buf+1),1,1,fp) ) // CK_B checksum expected
				return UINT_MAX;
			if (   (0xff & buf[0]) != cs[0]
				|| (0xff & buf[1]) != cs[1]) {
					printf("\n warning: uBlox checksum failed at %lu (epoch %02d/%02d/%02d_%02d:%02d:%06.3f)", ftell(fp),
						gnss->epoch.Y, gnss->epoch.M, gnss->epoch.D, gnss->epoch.h, gnss->epoch.m, gnss->epoch.s);
					return UINT_MAX;
			}
			return id;
		}
	//if (id >= msg_id_count)
	//	printf("\n warning: unknown ubx message 0x%02x 0x%02x", buf[0], buf[1]);

	return UINT_MAX;
}
	// RXM-RAWX: multi-gnss raw measurement data
char pony_gnss_io_ublox_file_parse_RXM_RAWX(unsigned char *cs, pony_gnss *gnss, FILE *fp, void *hdr_buf, void *blk_buf) {

	enum  system_id {gps, glo, gal,	bds, sys_count};

	const unsigned char message_version_expected = 0x01;
	const size_t 
		chars_8byte = (8*8)/CHAR_BIT, 
		chars_4byte = (4*8)/CHAR_BIT,
		freq_id_eph = 16;

	pony_gnss_io_ublox_RXM_RAWX_header *hdr	= NULL;
	pony_gnss_io_ublox_RXM_RAWX_block  *blk	= NULL;
	pony_gnss_sat *sat				= NULL;
	unsigned char *ptr				= NULL;
	size_t i, j, m, sys, s, maxsat;

	if (cs == NULL || gnss == NULL || fp == NULL || hdr_buf == NULL || blk_buf == NULL)
		return 0;

	hdr = (pony_gnss_io_ublox_RXM_RAWX_header *)hdr_buf;
	blk = (pony_gnss_io_ublox_RXM_RAWX_block  *)blk_buf;

	// get header data
	memset(hdr_buf,0,sizeof(*hdr)); // drop to all zeros
	if ( !fread((void *)(&(hdr->rcvTow      )),8,1,fp) ) return 0; // R8: time of week
	if ( !fread((void *)(&(hdr->week        )),2,1,fp) ) return 0; // U2: week
	if ( !fread((void *)(&(hdr->leapS       )),1,1,fp) ) return 0; // I1: leapS
	if ( !fread((void *)(&(hdr->numMeas     )),1,1,fp) ) return 0; // U1: numMeas
	if ( !fread((void *)(&(hdr->recStat     )),1,1,fp) ) return 0; // X1: recStat
	if ( !fread((void *)(&(hdr->version     )),1,1,fp) ) return 0; // U1: version
	if ( !fread((void *)(&(hdr->reserved1[0])),1,1,fp) ) return 0; // U1: reserved1[0]
	if ( !fread((void *)(&(hdr->reserved1[1])),1,1,fp) ) return 0; // U1: reserved1[1]
	// checksum
	ptr = (unsigned char *)(&(hdr->rcvTow)); // converting double to chars
	for (i = 0; i < chars_8byte; i++) for (j = 0; j < CHAR_BIT; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,ptr[i]>>j);
	for (j = 0; j < 16; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char )(hdr->week>>j));
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->leapS       );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->numMeas     );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->recStat     );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->version     );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->reserved1[0]);
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->reserved1[1]);
	// validate
	if (hdr->version > message_version_expected)	return 0;
	if (hdr->rcvTow   < 0)							return 0;
	if (hdr->numMeas == 0)							return 0;
	// process
	if ( !pony_time_gps2epoch(&(gnss->epoch), hdr->week, hdr->rcvTow) ) return 0;
	if (hdr->recStat & 0x01) {
		gnss->leap_sec			= hdr->leapS;
		gnss->leap_sec_valid	= 1;
	}
	// drop satellite and solution flags
	if (gnss->gps != NULL) pony_gnss_io_ublox_drop_flags_pony_sats(gnss->gps->sat, gnss->gps->max_sat_count, gnss->gps->obs_count);
	if (gnss->glo != NULL) pony_gnss_io_ublox_drop_flags_pony_sats(gnss->glo->sat, gnss->glo->max_sat_count, gnss->glo->obs_count);
	if (gnss->gal != NULL) pony_gnss_io_ublox_drop_flags_pony_sats(gnss->gal->sat, gnss->gal->max_sat_count, gnss->gal->obs_count);
	if (gnss->bds != NULL) pony_gnss_io_ublox_drop_flags_pony_sats(gnss->bds->sat, gnss->bds->max_sat_count, gnss->bds->obs_count);
	pony_gnss_io_ublox_drop_flags_pony_sol(&(gnss->sol));
	// measurement blocks
	for (m = 0; m < hdr->numMeas; m++) {
		// get data
		memset(blk_buf,0,sizeof(*blk)); // drop to all zeros
		if ( !fread((void *)(&(blk->prMes    )),8,1,fp) ) return 0; // R8: pseudorange measurement, [m]
		if ( !fread((void *)(&(blk->cpMes    )),8,1,fp) ) return 0; // R8: carrier phase measurement, [cycles]
		if ( !fread((void *)(&(blk->doMes    )),4,1,fp) ) return 0; // R4: doppler measurement, [Hz] 
		if ( !fread((void *)(&(blk->gnssId   )),1,1,fp) ) return 0; // U1: gnss identifier
		if ( !fread((void *)(&(blk->svId     )),1,1,fp) ) return 0; // U1: satellite identifier 
		if ( !fread((void *)(&(blk->sigId    )),1,1,fp) ) return 0; // U1: signal identifier 
		if ( !fread((void *)(&(blk->freqId   )),1,1,fp) ) return 0; // U1: GLONASS frequency slot + 7
		if ( !fread((void *)(&(blk->locktime )),2,1,fp) ) return 0; // U2: carrier phase locktime counter, [ms] 
		if ( !fread((void *)(&(blk->cno      )),1,1,fp) ) return 0; // U1: carrier-to-noise density ratio (signal strength), [dBHz]
		if ( !fread((void *)(&(blk->prStdev  )),1,1,fp) ) return 0; // X1: estimated pseudorange standard deviation, [0.01x2^n m]
		if ( !fread((void *)(&(blk->cpStdev  )),1,1,fp) ) return 0; // X1: estimated carrier phase standard deviation, [0.004 cycles]
		if ( !fread((void *)(&(blk->doStdev  )),1,1,fp) ) return 0; // X1: estimated doppler standard deviation, [0.002x2^n Hz] 
		if ( !fread((void *)(&(blk->trkStat  )),1,1,fp) ) return 0; // X1: tracking status 
		if ( !fread((void *)(&(blk->reserved2)),1,1,fp) ) return 0; // U1: reserved2
		// checksum
		ptr = (unsigned char *)(&(blk->prMes)); // converting double to chars
		for (i = 0; i < chars_8byte; i++) for (j = 0; j < CHAR_BIT; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,ptr[i]>>j);
		ptr = (unsigned char *)(&(blk->cpMes)); // converting double to chars
		for (i = 0; i < chars_8byte; i++) for (j = 0; j < CHAR_BIT; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,ptr[i]>>j);
		ptr = (unsigned char *)(&(blk->doMes)); // converting double to chars
		for (i = 0; i < chars_4byte; i++) for (j = 0; j < CHAR_BIT; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,ptr[i]>>j);
		pony_gnss_io_ublox_checksum_recurse(cs,blk->gnssId   );
		pony_gnss_io_ublox_checksum_recurse(cs,blk->svId     );
		pony_gnss_io_ublox_checksum_recurse(cs,blk->sigId    );
		pony_gnss_io_ublox_checksum_recurse(cs,blk->freqId   );
		for (j = 0; j < 16; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char )(blk->locktime>>j));
		pony_gnss_io_ublox_checksum_recurse(cs,blk->cno      );
		pony_gnss_io_ublox_checksum_recurse(cs,blk->prStdev  );
		pony_gnss_io_ublox_checksum_recurse(cs,blk->cpStdev  );
		pony_gnss_io_ublox_checksum_recurse(cs,blk->doStdev  );
		pony_gnss_io_ublox_checksum_recurse(cs,blk->trkStat  );
		pony_gnss_io_ublox_checksum_recurse(cs,blk->reserved2);
		// process
		for (sys = 0; sys < sys_count && blk->gnssId != pony_gnss_io_ublox_signal_id_conversion_table[sys].gnss_id; sys++);
		if (sys >= sys_count) continue; // unknown gnss constellation
		switch (sys) {
			case gps: if (gnss->gps == NULL) continue; else sat = gnss->gps->sat; maxsat = gnss->gps->max_sat_count; break;
			case glo: if (gnss->glo == NULL) continue; else sat = gnss->glo->sat; maxsat = gnss->glo->max_sat_count; break;
			case gal: if (gnss->gal == NULL) continue; else sat = gnss->gal->sat; maxsat = gnss->gal->max_sat_count; break;
			case bds: if (gnss->bds == NULL) continue; else sat = gnss->bds->sat; maxsat = gnss->bds->max_sat_count; break;
			default : sat = NULL; maxsat = 0;
		}
		if (sat == NULL) continue; // gnss constellation not initialized
		s = blk->svId-1;
		if (s >= maxsat) continue; // satellite id not supported
		for (j = 0; j < pony_gnss_io_ublox_signal_id_conversion_table[sys].types && blk->sigId != pony_gnss_io_ublox_signal_id_conversion_table[sys].ubx_sig_id[j]; j++);
		if (j >= pony_gnss_io_ublox_signal_id_conversion_table[sys].types) continue; // unknown signal id
		if (sys == glo)
			if (blk->freqId <= 13) // glonass frequency slot goes into ephemeris
				sat->eph[freq_id_eph] = blk->freqId - 7;
			else
				continue; // invalid data
		sat[s].obs[j*4+0] =         blk->prMes; // code pseudorange
		sat[s].obs[j*4+1] =         blk->cpMes; // carrier phase
		sat[s].obs[j*4+2] = (double)blk->doMes; // doppler shift
		sat[s].obs[j*4+3] = (double)blk->cno;   // carrier/noise
		sat[s].obs_valid[j*4+0] = (  blk->trkStat & 0x01                                  ) ? 1 : 0; // code pseudorange validity
		sat[s].obs_valid[j*4+1] = ( (blk->trkStat & 0x02) && (blk->cpStdev & 0x0f) != 0x0f) ? 1 : 0; // carrier phase validity
		sat[s].obs_valid[j*4+2] = 1; // doppler shift validity
		sat[s].obs_valid[j*4+3] = 1; // carrier/noise validity
		//std = 0.010*(0x01<<blk->prStdev);	if (std > gnss->settings.   code_sigma) gnss->settings.   code_sigma = std;
		//std = 0.004*blk->cpStdev;			if (std > gnss->settings.  phase_sigma) gnss->settings.  phase_sigma = std;
		//std = 0.002*(0x01<<blk->doStdev);	if (std > gnss->settings.doppler_sigma) gnss->settings.doppler_sigma = std;
	}

	return hdr->numMeas;
}
	// RXM-RAW: GPS L1 raw measurement data
char pony_gnss_io_ublox_file_parse_RXM_RAW(unsigned char *cs, pony_gnss *gnss, FILE *fp, void *hdr_buf, void *blk_buf) {

	const size_t 
		sigId = 0,
		chars_8byte = (8*8)/CHAR_BIT, 
		chars_4byte = (4*8)/CHAR_BIT;
	const unsigned char   lli_max = 0x07;      // maximum loss of lock indicator value
	const   signed char mesQI_min = 4;         // minimum measurement quality indicator value
	const double pr_min = +1e7, pr_max = +3e7, // valid pseudorange span
		         do_min = -1e4, do_max = +1e4; // valid doppler shift span

	pony_gnss_io_ublox_RXM_RAW_header *hdr	= NULL;
	pony_gnss_io_ublox_RXM_RAW_block  *blk	= NULL;
	unsigned char *ptr				= NULL, i, j, s, svid;

	if (   gnss           == NULL 
		|| gnss->gps      == NULL 
		|| gnss->gps->sat == NULL 
		|| cs      == NULL 
		|| fp      == NULL 
		|| hdr_buf == NULL 
		|| blk_buf == NULL)
		return 0;

	hdr = (pony_gnss_io_ublox_RXM_RAW_header *)hdr_buf;
	blk = (pony_gnss_io_ublox_RXM_RAW_block  *)blk_buf;

	// get header data
	memset(hdr_buf,0,sizeof(*hdr)); // drop to all zeros
	if ( !fread((void *)(&(hdr->iTOW     )),4,1,fp) ) return 0; // I4: measurement integer millisecond GPS time of week
	if ( !fread((void *)(&(hdr->week     )),2,1,fp) ) return 0; // I2: measurement GPS week number
	if ( !fread((void *)(&(hdr->numSV    )),1,1,fp) ) return 0; // U1: number of satellites following
	if ( !fread((void *)(&(hdr->reserved1)),1,1,fp) ) return 0; // U1: reserved
	// checksum
	for (j = 0; j < 32; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char )(hdr->iTOW>>j));
	for (j = 0; j < 16; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char )(hdr->week>>j));
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->numSV    );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->reserved1);
	// validate
	if (   hdr->iTOW  <  0
		|| hdr->week  <  0
		|| hdr->numSV == 0)							return 0;
	// process
	if ( !pony_time_gps2epoch(&(gnss->epoch), hdr->week, hdr->iTOW/1000.0) ) return 0;
	// drop satellite and solution flags
	pony_gnss_io_ublox_drop_flags_pony_sats(gnss->gps->sat, gnss->gps->max_sat_count, gnss->gps->obs_count);
	pony_gnss_io_ublox_drop_flags_pony_sol(&(gnss->sol));
	// measurement blocks
	for (s = 0; s < hdr->numSV; s++) {
		// get data
		memset(blk_buf,0,sizeof(*blk)); // drop to all zeros
		if ( !fread((void *)(&(blk->cpMes)),8,1,fp) ) return 0; // R8: carrier phase measurement, [cycles]
		if ( !fread((void *)(&(blk->prMes)),8,1,fp) ) return 0; // R8: pseudorange measurement, [m]
		if ( !fread((void *)(&(blk->doMes)),4,1,fp) ) return 0; // R4: doppler measurement, [Hz] 
		if ( !fread((void *)(&(blk->sv   )),1,1,fp) ) return 0; // U1: space vehicle number 
		if ( !fread((void *)(&(blk->mesQI)),1,1,fp) ) return 0; // I1: nav measurements quality indicator
		if ( !fread((void *)(&(blk->cno  )),1,1,fp) ) return 0; // I1: signal strength carrier-to-noise ratio, [dBHz]
		if ( !fread((void *)(&(blk->lli  )),1,1,fp) ) return 0; // U1: loss of lock indicator
		// checksum
		ptr = (unsigned char *)(&(blk->cpMes)); // converting double to chars
		for (i = 0; i < chars_8byte; i++) for (j = 0; j < CHAR_BIT; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,ptr[i]>>j);
		ptr = (unsigned char *)(&(blk->prMes)); // converting double to chars
		for (i = 0; i < chars_8byte; i++) for (j = 0; j < CHAR_BIT; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,ptr[i]>>j);
		ptr = (unsigned char *)(&(blk->doMes)); // converting double to chars
		for (i = 0; i < chars_4byte; i++) for (j = 0; j < CHAR_BIT; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,ptr[i]>>j);
		pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char)(blk->sv   ));
		pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char)(blk->mesQI));
		pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char)(blk->cno  ));
		pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char)(blk->lli  ));
		// process
		if (blk->sv == 0 || blk->sv > gnss->gps->max_sat_count) continue; // satellite id not supported
		if (   blk->mesQI < mesQI_min 
			|| blk->cno <= 0 
			|| blk->lli > lli_max
			|| blk->prMes < pr_min || blk->prMes > pr_max
			|| blk->doMes < do_min || blk->doMes > do_max) 
			return s; // corrupted data
		svid = blk->sv-1;
		gnss->gps->sat[svid].obs[sigId*4+0] =         blk->prMes; // code pseudorange
		gnss->gps->sat[svid].obs[sigId*4+1] =         blk->cpMes; // carrier phase
		gnss->gps->sat[svid].obs[sigId*4+2] = (double)blk->doMes; // doppler shift
		gnss->gps->sat[svid].obs[sigId*4+3] = (double)blk->cno;   // carrier/noise
		gnss->gps->sat[svid].obs_valid[sigId*4+0] = ( blk->mesQI >= mesQI_min        ) ? 1 : 0; // code pseudorange validity
		gnss->gps->sat[svid].obs_valid[sigId*4+1] = ( blk->mesQI >= 6 && !(blk->lli) ) ? 1 : 0; // carrier phase validity
		gnss->gps->sat[svid].obs_valid[sigId*4+2] = ( blk->mesQI >= mesQI_min        ) ? 1 : 0; // doppler shift validity
		gnss->gps->sat[svid].obs_valid[sigId*4+3] = 1;                                          // carrier/noise validity
	}

	return hdr->numSV;
}
	// RXM-SFRBX: multi-GNSS broadcast navigation data subframe
#define PONY_GNSS_IO_UBLOX_TWO_m05    ((double)1./(0x01<< 5)) // 2^-5
#define PONY_GNSS_IO_UBLOX_TWO_m11    ((double)1./(0x01<<11)) // 2^-11
#define PONY_GNSS_IO_UBLOX_TWO_m19    ((double)1./(0x01<<19)) // 2^-19
#define PONY_GNSS_IO_UBLOX_TWO_m29    ((double)1./(0x01<<29)) // 2^-29
#define PONY_GNSS_IO_UBLOX_TWO_m31    (PONY_GNSS_IO_UBLOX_TWO_m29/(0x01<< 2)) // 2^-31
#define PONY_GNSS_IO_UBLOX_TWO_m40    (PONY_GNSS_IO_UBLOX_TWO_m29/(0x01<<11)) // 2^-40
#define PONY_GNSS_IO_UBLOX_TWO_m43    (PONY_GNSS_IO_UBLOX_TWO_m29/(0x01<<14)) // 2^-43
#define PONY_GNSS_IO_UBLOX_TWO_m50    (PONY_GNSS_IO_UBLOX_TWO_m29/(0x01<<21)) // 2^-50
#define PONY_GNSS_IO_UBLOX_TWO_m55    (PONY_GNSS_IO_UBLOX_TWO_m29/(0x01<<26)) // 2^-55
#define PONY_GNSS_IO_UBLOX_TWO_m66    (PONY_GNSS_IO_UBLOX_TWO_m55/(0x01<<11)) // 2^-66
#define PONY_GNSS_IO_UBLOX_PI          3.1415926535898
#define PONY_GNSS_IO_UBLOX_TWO_m31xPI (PONY_GNSS_IO_UBLOX_TWO_m31*PONY_GNSS_IO_UBLOX_PI) // 2^-31*pi
#define PONY_GNSS_IO_UBLOX_TWO_m43xPI (PONY_GNSS_IO_UBLOX_TWO_m43*PONY_GNSS_IO_UBLOX_PI) // 2^-43*pi
char pony_gnss_io_ublox_file_parse_RXM_SFRBX(unsigned char *cs, pony_gnss *gnss, FILE *fp, void *hdr_buf, void *blk_buf) {
	
	enum system_id {gps, glo, gal, bds, sys_count};

	const unsigned char message_version_expected = 0x02;
	const size_t 
		max_words = 10, 
		epoch_eph = 0,
		freq_eph  = 16;

	static char (*subframe_parser[])(pony_gnss_sat *, pony_gnss_io_ublox_RXM_SFRBX_block *blk) = 
		{pony_gnss_io_ublox_nav_subframe_parser_gps, pony_gnss_io_ublox_nav_subframe_parser_glo, pony_gnss_io_ublox_nav_subframe_parser_gal, pony_gnss_io_ublox_nav_subframe_parser_bds};

	pony_gnss_io_ublox_RXM_SFRBX_header *hdr	= NULL;
	pony_gnss_io_ublox_RXM_SFRBX_block  *blk	= NULL;
	pony_gnss_sat *sat				= NULL;
	unsigned char i, j, sys, s, maxsat;

	if (cs == NULL || gnss == NULL || fp == NULL || hdr_buf == NULL || blk_buf == NULL)
		return 0;

	hdr = (pony_gnss_io_ublox_RXM_SFRBX_header *)hdr_buf;
	blk = (pony_gnss_io_ublox_RXM_SFRBX_block  *)blk_buf;

	// get header data
	memset(hdr_buf,0,sizeof(*hdr)); // drop to all zeros
	if ( !fread((void *)(&(hdr->gnssId   )),1,1,fp) ) return 0; // U1: gnss identifier
	if ( !fread((void *)(&(hdr->svId     )),1,1,fp) ) return 0; // U1: satellite identifier
	if ( !fread((void *)(&(hdr->reserved1)),1,1,fp) ) return 0; // U1: reserved
	if ( !fread((void *)(&(hdr->freqId   )),1,1,fp) ) return 0; // U1: GLONASS frequency slot + 7
	if ( !fread((void *)(&(hdr->numWords )),1,1,fp) ) return 0; // U1: number of data words
	if ( !fread((void *)(&(hdr->chn      )),1,1,fp) ) return 0; // U1: tracking channel number
	if ( !fread((void *)(&(hdr->version  )),1,1,fp) ) return 0; // U1: message version
	if ( !fread((void *)(&(hdr->reserved2)),1,1,fp) ) return 0; // U1: reserved
	// checksum
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->gnssId   );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->svId     );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->reserved1);
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->freqId   );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->numWords );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->chn      );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->version  );
	pony_gnss_io_ublox_checksum_recurse(cs,hdr->reserved2);
	// validate
	if (hdr->version > message_version_expected) return 0;
	if (hdr->numWords == 0 || hdr->numWords > max_words) return 0;
	// process
	for (sys = 0; sys < sys_count && hdr->gnssId != pony_gnss_io_ublox_signal_id_conversion_table[sys].gnss_id; sys++);
	if (sys >= sys_count) return 0; // unknown gnss constellation
	switch (sys) {
		case gps: if (gnss->gps == NULL) return 0; else sat = gnss->gps->sat; maxsat = (unsigned char)gnss->gps->max_sat_count; break;
		case glo: if (gnss->glo == NULL) return 0; else sat = gnss->glo->sat; maxsat = (unsigned char)gnss->glo->max_sat_count; break;
		case gal: if (gnss->gal == NULL) return 0; else sat = gnss->gal->sat; maxsat = (unsigned char)gnss->gal->max_sat_count; break;
		case bds: if (gnss->bds == NULL) return 0; else sat = gnss->bds->sat; maxsat = (unsigned char)gnss->bds->max_sat_count; break;
		default : sat = NULL; maxsat = 0;
	}
	if (sat == NULL) return 0; // gnss constellation not initialized
	s = hdr->svId-1;
	if (s >= maxsat) return 0; // satellite id not supported
	if (sys == glo)	{ // GLONASS frequency slot
		gnss->glo->freq_slot[s] = (signed int)hdr->freqId - 7;
		sat[s].eph[freq_eph] = hdr->freqId - 7;
	}
	// subframe data blocks
	memset(blk_buf,0,sizeof(*blk)); // drop to all zeros
	for (i = 0; i < hdr->numWords; i++) {
		// get data
		if ( !fread((void *)(&(blk->dwrd[i])),4,1,fp) ) return 0; // U4: data word
		// checksum
		for (j = 0; j < 32; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char )(blk->dwrd[i]>>j));
	}
	// process
	if (sat[s].eph[epoch_eph+1] < 1) { // approximate date to resolve gps week number mod 1024, glonass day number, etc.
		sat[s].eph[epoch_eph+0] = gnss->epoch.Y; 
		sat[s].eph[epoch_eph+1] = gnss->epoch.M;
		sat[s].eph[epoch_eph+2] = gnss->epoch.D;
	}
	return (subframe_parser[sys])(&(sat[s]), blk);

}
	// RXM-SFRB: GPS broadcast navigation data subframe
#define PONY_GNSS_IO_UBLOX_RXM_EPH_MAX_TOTAL_CONVERSION_ENTRIES 12
char pony_gnss_io_ublox_file_parse_RXM_EPH(unsigned char *cs, pony_gnss *gnss, FILE *fp, void *hdr_buf, void *blk_buf) {
	
	const unsigned char 
		alert_flag_bit = 12, alert_eph = 29,	// alert flag
		eph_counter_full = 0x07;
	const double 
		URA_sqrt2minus1 = 0.41,
		alert_value	= 8192;
	const int wk_rollover = 0x0400;
	const size_t 
		    max_words   = 8,
			max_frames  = 3,
		  epoch_eph     = 0,
		    toc_entry   = 7,
		   week_entry   = 1,
		    SVa_entry   = 3,
		entries_count[] = {11, 9, 9};  // least significant bits entries count (extra msb-s not included)
	// conversion tables as in Section 20.3 of GPS Interface Specifications IS-GPS-200J (22 May 2018), p. 77
	const pony_gnss_io_ublox_nav_subframe_conversion_entry ctable[][PONY_GNSS_IO_UBLOX_RXM_EPH_MAX_TOTAL_CONVERSION_ENTRIES] = { {
		// subframe 1 entries (Table 20-I, IS-GPS-200J (22 May 2018), p. 95
		// wrd shift bits msb_entry_index                    scale  eph
		{   0,   12,   2, UINT_MAX,                              1, 26      }, //  0 Code on L2
		{   0,   14,  10, UINT_MAX,                              1, 27      }, //  1 Week No. (mod 1024)
		{   1,   23,   1, UINT_MAX,                              1, 28      }, //  2 L2 P data flag
		{   0,    8,   4, UINT_MAX,                              1, 29      }, //  3 SV accuracy (URA index)
		{   0,    2,   6, UINT_MAX,                              1, 30      }, //  4 SV health
		{   4,    0,   8, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m31   , 31      }, //  5 T_GD, x 2^-31
		{   5,   16,   8,       11,                              1, 32      }, //  6 IODC LSB (issue of clock data least significant bits)
		{   5,    0,  16, UINT_MAX,                           0x10,  5      }, //  7 toc (time of clock data), x 2^4
		{   6,   16,   8, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m55   ,  8      }, //  8 af2, x 2^-55 
		{   6,    0,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43   ,  7      }, //  9 af1, x 2^-43
		{   7,    2,  22, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m31   ,  6      }, // 10 af0, x 2^-31
		// extra entries for MSB sections
		{   0,    0,   2, UINT_MAX,                              0, UINT_MAX}, // 11 IODC MSB (issue of clock data most significant bits)
	}, {
		// subframe 2 entries (Table 20-III, IS-GPS-200J (22 May 2018), p. 103
		// wrd shift bits msb_entry_index                    scale  eph
		{   0,   16,   8, UINT_MAX,                              1,  9      }, //  0 IODE (issue of ephemeris data)
		{   0,    0,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m05   , 10      }, //  1 Crs,        x 2^-29
		{   1,    8,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 11      }, //  2 Delta n,    x 2^-43 x pi
		{   2,    0,  24,        9, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 12      }, //  3 M0    LSB,  x 2^-31 x pi
		{   3,    8,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 13      }, //  4 Cuc,        x 2^-29
		{   4,    0,  24,       10,  PONY_GNSS_IO_UBLOX_TWO_m31/4 , 14      }, //  5 e     LSB,  x 2^-33
		{   5,    8,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 15      }, //  6 Cus,        x 2^-29
		{   6,    0,  24,       11,  PONY_GNSS_IO_UBLOX_TWO_m19   , 16      }, //  7 sqrtA LSB,  x 2^-33
		{   7,    8,  16, UINT_MAX,                           0x10, 17      }, //  8 toe (time of ephemeris data)
		// extra entries for MSB sections			          
		{   1,    0,   8, UINT_MAX,                              0, UINT_MAX}, //  9 M0    MSB
		{   3,    0,   8, UINT_MAX,                              0, UINT_MAX}, // 10 e     MSB
		{   5,    0,   8, UINT_MAX,                              0, UINT_MAX}, // 11 sqrtA MSB
	}, {
		// subframe 3 entries (Table 20-III, IS-GPS-200J (22 May 2018), p. 103
		// wrd shift bits msb_entry_index                    scale  eph
		{   0,    8,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 18      }, //  0 Cic,        x 2^-29
		{   1,    0,  24,        9, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 19      }, //  1 Omega0 LSB, x 2^-31 x pi
		{   2,    8,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 20      }, //  2 Cis,        x 2^-29
		{   3,    0,  24,       10, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 21      }, //  3 i0     LSB, x 2^-31 x pi
		{   4,    8,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m05   , 22      }, //  4 Crc,        x 2^-5
		{   5,    0,  24,       11, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 23      }, //  5 omega  LSB, x 2^-31 x pi
		{   6,    0,  24, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 24      }, //  6 Omega dot,  x 2^-43 x pi
		{   7,   16,   8, UINT_MAX,                              1,  9      }, //  7 IODE (issue of ephemeris data)
		{   7,    2,  14, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 25      }, //  8 idot,       x 2^-43 x pi
		// extra entries for MSB sections
		{   0,    0,   8, UINT_MAX,                              0, UINT_MAX}, //  9 Omega0 MSB
		{   2,    0,   8, UINT_MAX,                              0, UINT_MAX}, // 10 i0     MSB
		{   4,    0,   8, UINT_MAX,                              0, UINT_MAX}, // 11 omega  MSB
	} };

	pony_gnss_io_ublox_RXM_EPH_header *hdr	= NULL;
	pony_gnss_io_ublox_RXM_EPH_block  *blk	= NULL;
	pony_gnss_sat *sat = NULL;
	unsigned char sf, i, j;
	unsigned short N;
	int week0;
	pony_time_epoch epoch = {0,0,0,0,0,0};

	if (   gnss           == NULL
		|| gnss->gps      == NULL
		|| gnss->gps->sat == NULL
		|| cs      == NULL 
		|| fp      == NULL 
		|| hdr_buf == NULL 
		|| blk_buf == NULL)
		return 0;

	hdr = (pony_gnss_io_ublox_RXM_EPH_header *)hdr_buf;
	blk = (pony_gnss_io_ublox_RXM_EPH_block  *)blk_buf;

	// get header data
	memset(hdr_buf,0,sizeof(*hdr)); // drop to all zeros
	if ( !fread((void *)(&(hdr->svid)),4,1,fp) ) return 0; // U4: satellite vehicle identifier
	if ( !fread((void *)(&(hdr->how )),4,1,fp) ) return 0; // U4: handover word
	// checksum
	for (j = 0; j < 32; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char )(hdr->svid>>j));
	for (j = 0; j < 32; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char )(hdr->how >>j));
	// validate
	if (hdr->svid == 0 || hdr->svid > gnss->gps->max_sat_count) return 0; // satellite id not supported
	//if (hdr->how  == 0) return 0; // no ephemeris data is following
	// subframe data blocks
	memset(blk_buf,0,sizeof(*blk)); // drop to all zeros
	for (sf = 0; sf < max_frames; sf++)
		for (i = 0; i < max_words; i++) {
			// get data
			if ( !fread((void *)(&(blk->sf[sf][i])),4,1,fp) ) return 0; // U4: data word
			// checksum
			for (j = 0; j < 32; j += 8)	pony_gnss_io_ublox_checksum_recurse(cs,(unsigned char )(blk->sf[sf][i]>>j));
		}
	sat = &(gnss->gps->sat[hdr->svid-1]);
	if (sat->eph[epoch_eph+1] < 1) { // approximate date to resolve gps week number mod 1024, glonass day number, etc.
		sat->eph[epoch_eph+0] = gnss->epoch.Y; 
		sat->eph[epoch_eph+1] = gnss->epoch.M;
		sat->eph[epoch_eph+2] = gnss->epoch.D;
	}
	// store current satellite date, if present
	epoch.Y = pony_gnss_io_ublox_round(sat->eph[epoch_eph+0]);
	epoch.M = pony_gnss_io_ublox_round(sat->eph[epoch_eph+1]);
	epoch.D = pony_gnss_io_ublox_round(sat->eph[epoch_eph+2]);
	if (!pony_time_epoch2gps((unsigned int *)(&week0), NULL, &epoch))
		week0 = -1;
	// process
	for (sf = 0, sat->eph_counter = 0; sf < max_frames; sf++) {
		// regular data processing
		if ( !pony_gnss_io_ublox_nav_subframe_parse_data(sat->eph, blk->sf[sf], (pony_gnss_io_ublox_nav_subframe_conversion_entry *)(&(ctable[sf])), entries_count[sf], 1) )
			return 0;
		// special entries handling
		switch (sf+1) { 
			case 1:
				// URA index to SV accuracy: SVa = 2^(1+N/2) [N = 0..6], 2^(N-2) [N = 7..15]
				N = (unsigned short)(sat->eph[ ctable[0][SVa_entry].eph ]);
				sat->eph[ ctable[0][SVa_entry].eph ] = (N < 7) ? (0x01<<(1 + N/2))*(1 + URA_sqrt2minus1*(N%2)) : (0x01<<(N-2));
				// toc
				if (week0 >= 0) {
					// floor(.5+w0/wr)*wr + mod(w+wr/2,wr)-wr/2 -- closest to week0
					week0 = pony_gnss_io_ublox_round(((double)week0)/wk_rollover)*wk_rollover + (int)(sat->eph[ ctable[0][week_entry].eph ] + wk_rollover/2)%wk_rollover - wk_rollover/2;
					sat->eph[ ctable[0][week_entry].eph ] = (double)week0;
				}
				else
					week0 = pony_gnss_io_ublox_round(sat->eph[ ctable[0][week_entry].eph ]);
				// week number check
				if (week0 <= 0)
					return 0;
				pony_time_gps2epoch(&epoch, (unsigned int)week0, sat->eph[ ctable[0][toc_entry].eph ]);
				sat->eph[epoch_eph+0] = epoch.Y;
				sat->eph[epoch_eph+1] = epoch.M;
				sat->eph[epoch_eph+2] = epoch.D;
				sat->eph[epoch_eph+3] = epoch.h;
				sat->eph[epoch_eph+4] = epoch.m;
				sat->eph[epoch_eph+5] = epoch.s;
				break;
			case 2: break;
			case 3: break;
		}
		sat->eph_counter |= (0x01<<sf);
	}
	// check whether all subframes have been collected
	if ((sat->eph_counter & eph_counter_full) == eph_counter_full) {
		sat->eph_valid = 1;
		sat->eph_counter = 0;
	}
	else
		sat->eph_valid = 0;
	// alert flag
	if ( (hdr->how>>alert_flag_bit) & 0x01 ) {
		sat->eph[alert_eph] = alert_value; // use at own risk
		sat->eph_valid = 0; // force no-use
	}

	return 0;

}




// navigation subframe parsers
	// single subframe data parser
char pony_gnss_io_ublox_nav_subframe_parse_data(double *eph, unsigned long *dwrd, 
									  pony_gnss_io_ublox_nav_subframe_conversion_entry *conversion_table, const size_t entries_count,
									  const char complement) {

	unsigned long value, mask;
	size_t i, j;

	if (eph == NULL || dwrd == NULL || conversion_table == NULL || entries_count == 0)
		return 0; // invalid input

	for (i = 0; i < entries_count; i++) {
		mask  = ((unsigned long)(-1))>>(sizeof(long)*CHAR_BIT - conversion_table[i].bits);
		value = ((dwrd[conversion_table[i].wrd])>>(conversion_table[i].shift)) & mask;
		j = conversion_table[i].msb;
		if (j < UINT_MAX) {
			mask   = ((mask+1)<<(conversion_table[j].bits)) - 1;
			value += (( ((dwrd[conversion_table[j].wrd])>>(conversion_table[j].shift)) & ((0x01<<conversion_table[j].bits)-1) )<<conversion_table[i].bits);
			value &= mask;
		}
		eph[conversion_table[i].eph] = 
		//      treat as signed                 ? (  sign bit               ?                     two's complement or not      1 bit less :   negative value ) :  as is
			( ( conversion_table[i].scale < 0 ) ? ( (value & ((mask>>1)+1)) ? (double)(( complement ? (~value + 1) : value ) & (mask>>1)) : -((double)value) ) : (double)value )*conversion_table[i].scale;
	}
	return 1;

}

	// GPS subframes
#define PONY_GNSS_IO_UBLOX_GPS_NAV_SUBFRAME_MAX_TOTAL_CONVERSION_ENTRIES 12
char pony_gnss_io_ublox_nav_subframe_parser_gps(pony_gnss_sat *sat, pony_gnss_io_ublox_RXM_SFRBX_block *subframe) {

	const unsigned char 
		tlm_word = 0, tlm_preamble_shift = 22, tlm_preamble = 0x8b, tlm_preamble_mask = 0xff,				// telemetry word
		how_word = 1, how_alert_flag_bit = 12, how_alert_eph = 29, how_id_shift = 8, how_id_mask = 0x07,	// handover word
		eph_counter_full = 0x07;
	const double 
		URA_sqrt2minus1 = 0.41,
		how_alert_value	= 8192;
	const int wk_rollover = 0x0400;
	const size_t 
		  epoch_eph     = 0,
		    IOD_eph     = 9,
		    toc_entry   = 7,
		   week_entry   = 1,
		    SVa_entry   = 3,
		entries_count[] = {11, 9, 9};  // least significant bits entries count (extra msb-s not included)
	// conversion tables as in Section 20.3 of GPS Interface Specifications IS-GPS-200J (22 May 2018), p. 77
	const pony_gnss_io_ublox_nav_subframe_conversion_entry ctable[][PONY_GNSS_IO_UBLOX_GPS_NAV_SUBFRAME_MAX_TOTAL_CONVERSION_ENTRIES] = { {
		// subframe 1 entries (Table 20-I, IS-GPS-200J (22 May 2018), p. 95
		// wrd shift bits msb_entry_index                    scale  eph
		{   2,   18,   2, UINT_MAX,                              1, 26      }, //  0 Code on L2
		{   2,   20,  10, UINT_MAX,                              1, 27      }, //  1 Week No. (mod 1024)
		{   3,   29,   1, UINT_MAX,                              1, 28      }, //  2 L2 P data flag
		{   2,   14,   4, UINT_MAX,                              1, 29      }, //  3 SV accuracy (URA index)
		{   2,    8,   6, UINT_MAX,                              1, 30      }, //  4 SV health
		{   6,    6,   8, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m31   , 31      }, //  5 T_GD, x 2^-31
		{   7,   22,   8,       11,                              1, 32      }, //  6 IODC LSB (issue of clock data least significant bits)
		{   7,    6,  16, UINT_MAX,                           0x10,  5      }, //  7 toc (time of clock data), x 2^4
		{   8,   22,   8, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m55   ,  8      }, //  8 af2, x 2^-55 
		{   8,    6,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43   ,  7      }, //  9 af1, x 2^-43
		{   9,    8,  22, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m31   ,  6      }, // 10 af0, x 2^-31
		// extra entries for MSB sections
		{   2,    6,   2, UINT_MAX,                              0, UINT_MAX}, // 11 IODC MSB (issue of clock data most significant bits)
	}, {
		// subframe 2 entries (Table 20-III, IS-GPS-200J (22 May 2018), p. 103
		// wrd shift bits msb_entry_index                    scale  eph
		{   2,   22,   8, UINT_MAX,                              1,  9      }, //  0 IODE (issue of ephemeris data)
		{   2,    6,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m05   , 10      }, //  1 Crs,        x 2^-29
		{   3,   14,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 11      }, //  2 Delta n,    x 2^-43 x pi
		{   4,    6,  24,        9, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 12      }, //  3 M0    LSB,  x 2^-31 x pi
		{   5,   14,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 13      }, //  4 Cuc,        x 2^-29
		{   6,    6,  24,       10,  PONY_GNSS_IO_UBLOX_TWO_m31/4 , 14      }, //  5 e     LSB,  x 2^-33
		{   7,   14,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 15      }, //  6 Cus,        x 2^-29
		{   8,    6,  24,       11,  PONY_GNSS_IO_UBLOX_TWO_m19   , 16      }, //  7 sqrtA LSB,  x 2^-33
		{   9,   14,  16, UINT_MAX,                           0x10, 17      }, //  8 toe (time of ephemeris data)
		// extra entries for MSB sections			          
		{   3,    6,   8, UINT_MAX,                              0, UINT_MAX}, //  9 M0    MSB
		{   5,    6,   8, UINT_MAX,                              0, UINT_MAX}, // 10 e     MSB
		{   7,    6,   8, UINT_MAX,                              0, UINT_MAX}, // 11 sqrtA MSB
	}, {
		// subframe 3 entries (Table 20-III, IS-GPS-200J (22 May 2018), p. 103
		// wrd shift bits msb_entry_index                    scale  eph
		{   2,   14,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 18      }, //  0 Cic,        x 2^-29
		{   3,    6,  24,        9, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 19      }, //  1 Omega0 LSB, x 2^-31 x pi
		{   4,   14,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 20      }, //  2 Cis,        x 2^-29
		{   5,    6,  24,       10, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 21      }, //  3 i0     LSB, x 2^-31 x pi
		{   6,   14,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m05   , 22      }, //  4 Crc,        x 2^-5
		{   7,    6,  24,       11, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 23      }, //  5 omega  LSB, x 2^-31 x pi
		{   8,    6,  24, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 24      }, //  6 Omega dot,  x 2^-43 x pi
		{   9,   22,   8, UINT_MAX,                              1,  9      }, //  7 IODE (issue of ephemeris data)
		{   9,    8,  14, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 25      }, //  8 idot,       x 2^-43 x pi
		// extra entries for MSB sections
		{   2,    6,   8, UINT_MAX,                              0, UINT_MAX}, //  9 Omega0 MSB
		{   4,    6,   8, UINT_MAX,                              0, UINT_MAX}, // 10 i0     MSB
		{   6,    6,   8, UINT_MAX,                              0, UINT_MAX}, // 11 omega  MSB
	} };

	unsigned char id;
	unsigned short N;
	int week0, IODprev;
	pony_time_epoch epoch = {0,0,0,0,0,0};

	// store current satellite date, if present
	epoch.Y = pony_gnss_io_ublox_round(sat->eph[epoch_eph+0]);
	epoch.M = pony_gnss_io_ublox_round(sat->eph[epoch_eph+1]);
	epoch.D = pony_gnss_io_ublox_round(sat->eph[epoch_eph+2]);
	if (!pony_time_epoch2gps((unsigned int *)(&week0), NULL, &epoch))
		week0 = -1;
	// store IODE
	IODprev = pony_gnss_io_ublox_round(sat->eph[IOD_eph]);
	// tlm word - telemetry
		// preamble
	if ( ((subframe->dwrd[tlm_word]>>tlm_preamble_shift) & tlm_preamble_mask) != tlm_preamble ) 
		return 0;
	// how word - handover 
		// subframe id
	id = (subframe->dwrd[how_word]>>how_id_shift) & how_id_mask;
	// regular data processing
	if (id < 1 || id > 3) return 0; // subframe id invalid or not supported
	if ( !pony_gnss_io_ublox_nav_subframe_parse_data(sat->eph, subframe->dwrd, (pony_gnss_io_ublox_nav_subframe_conversion_entry *)(&(ctable[id-1])), entries_count[id-1], 1) )
		return 0;
	// special entries handling
	switch (id) { 
		case 1:
			// URA index to SV accuracy: SVa = 2^(1+N/2) [N = 0..6], 2^(N-2) [N = 7..15]
			N = (unsigned short)(sat->eph[ ctable[0][SVa_entry].eph ]);
			sat->eph[ ctable[0][SVa_entry].eph ] = (N < 7) ? (0x01<<(1 + N/2))*(1 + URA_sqrt2minus1*(N%2)) : (0x01<<(N-2));
			// toc
			if (week0 >= 0) {
				// floor(.5+w0/wr)*wr + mod(w+wr/2,wr)-wr/2 -- closest to week0
				week0 = pony_gnss_io_ublox_round(((double)week0)/wk_rollover)*wk_rollover + (int)(sat->eph[ ctable[0][week_entry].eph ] + wk_rollover/2)%wk_rollover - wk_rollover/2;
				sat->eph[ ctable[0][week_entry].eph ] = (double)week0;
			}
			else
				week0 = pony_gnss_io_ublox_round(sat->eph[ ctable[0][week_entry].eph ]);
			// week number check
			if (week0 <= 0)
				return 0;
			pony_time_gps2epoch(&epoch, (unsigned int)week0, sat->eph[ ctable[0][toc_entry].eph ]);
			sat->eph[epoch_eph+0] = epoch.Y;
			sat->eph[epoch_eph+1] = epoch.M;
			sat->eph[epoch_eph+2] = epoch.D;
			sat->eph[epoch_eph+3] = epoch.h;
			sat->eph[epoch_eph+4] = epoch.m;
			sat->eph[epoch_eph+5] = epoch.s;
			break;
		case 2: break;
		case 3: break;
	}
	// check whether all subframes have been collected
	if (IODprev != pony_gnss_io_ublox_round(sat->eph[IOD_eph])) {// new issue of data has started
		sat->eph_valid = 0;   // stop using the satellite
		sat->eph_counter = 0;
	}
	sat->eph_counter |= (0x01<<(id-1));
	if ((sat->eph_counter & eph_counter_full) == eph_counter_full) {
		sat->eph_valid = 1;
		sat->eph_counter = 0;
	}
	// alert flag
	if ( (subframe->dwrd[how_word]>>how_alert_flag_bit) & 0x01 ) {
		sat->eph[how_alert_eph] = how_alert_value; // use at own risk
		sat->eph_valid = 0; // force no-use
	}

	return 1;
}

	// GLONASS subframes
#define PONY_GNSS_IO_UBLOX_GLO_NAV_SUBFRAME_MAX_TOTAL_CONVERSION_ENTRIES 7
char pony_gnss_io_ublox_nav_subframe_parser_glo(pony_gnss_sat *sat, pony_gnss_io_ublox_RXM_SFRBX_block *subframe) {

	const double dt_utc_h = 3, dt_utc_s = dt_utc_h*3600;
	const unsigned char 
		msg_id_shift		= 27,
		tb_shift            = 14,
		NT_shift            = 21,
		eph_counter_full	= 0x0f;
	const unsigned long
		tb_mask = 0x007f,
		NT_mask = 0x07ff;
	const size_t        msg_id_word		= 0,
						epoch_eph		= 0,
						tau_entry		= 0,
						tb_entry		= 1,
						NT_entry		= 2,
						tk_s_entry		= 0,
						tk_h_entry		= 1,
						tk_eph			= 8,
						entries_count[]	= {5, 5, 4, 3};  // least significant bits entries count (extra msb-s not included)
	// conversion tables as in Section 4.3.2 of GLONASS Interface Control Document ICD L1,L2 GLONASS Edition 5.1 2008, p. 27
	const pony_gnss_io_ublox_nav_subframe_conversion_entry ctable[][PONY_GNSS_IO_UBLOX_GLO_NAV_SUBFRAME_MAX_TOTAL_CONVERSION_ENTRIES] = { {
		// string 1 entries (Table 4.5, ICD L1,L2 GLONASS Ed. 5.1 2008, p. 32)
		// wrd shift bits msb_entry_index                   scale  eph
		{   0,   11,   7, UINT_MAX,                            30,  5      }, //  0 tk sec, x 30
		{   0,   18,   5, UINT_MAX,                             1,  3      }, //  1 tk hours
		{   1,   19,  13,        5, -PONY_GNSS_IO_UBLOX_TWO_m19/2, 10      }, //  2 Vx LSB, x 2^-20
		{   1,   14,   5, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29/2, 11      }, //  3 Ax,     x 2^-30
		{   2,   19,  13,        6, -PONY_GNSS_IO_UBLOX_TWO_m11  ,  9      }, //  4 X  LSB, x 2^-11
		// extra entries for MSB sections
		{   0,    0,  11, UINT_MAX,                             0, UINT_MAX}, //  5 Vx MSB
		{   1,    0,  14, UINT_MAX,                             0, UINT_MAX}, //  6 X  MSB
	}, {
		// string 2 entries (Table 4.5, ICD L1,L2 GLONASS Ed. 5.1 2008, p. 32)
		// wrd shift bits msb_entry_index                   scale  eph
		{   0,   24,   3, UINT_MAX,                             1, 12      }, //  0 B
		{   0,   16,   7, UINT_MAX,                            15,  4      }, //  1 tb,     x 15
		{   1,   19,  13,        5, -PONY_GNSS_IO_UBLOX_TWO_m19/2, 14      }, //  2 Vy LSB, x 2^-20
		{   1,   14,   5, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29/2, 15      }, //  3 Ay,     x 2^-30
		{   2,   19,  13,        6, -PONY_GNSS_IO_UBLOX_TWO_m11  , 13      }, //  4 Y  LSB, x 2^-11
		// extra entries for MSB sections
		{   0,    0,  11, UINT_MAX,                           0  , UINT_MAX}, //  5 Vy MSB
		{   1,    0,  14, UINT_MAX,                           0  , UINT_MAX}, //  6 Y  MSB
	}, {
		// string 3 entries (Table 4.5, ICD L1,L2 GLONASS Ed. 5.1 2008, p. 32)
		// wrd shift bits msb_entry_index                   scale  eph
		{   0,   15,  11, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m40  ,  7      }, //  0 gamma,  x 2^-40
		{   1,   19,  13,        4, -PONY_GNSS_IO_UBLOX_TWO_m19/2, 18      }, //  1 Vz LSB, x 2^-20
		{   1,   14,   5, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29/2, 19      }, //  2 Az,     x 2^-30
		{   2,   19,  13,        5, -PONY_GNSS_IO_UBLOX_TWO_m11  , 17      }, //  3 Z  LSB, x 2^-11
		// extra entries for MSB sections				  
		{   0,    0,  11, UINT_MAX,                             0, UINT_MAX}, //  4 Vz MSB
		{   1,    0,  14, UINT_MAX,                             0, UINT_MAX}, //  5 Z  MSB
	}, {
		// string 4 entries (Table 4.5, ICD L1,L2 GLONASS Ed. 5.1 2008, p. 32)
		// wrd shift bits msb_entry_index                   scale  eph
		{   0,    5,  22, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29/2,  6      }, //  0 tau,    x 2^-30
		{   1,   27,   5, UINT_MAX,                             1, 20      }, //  1 E
		{   2,   26,   6,        3,                             1,  2      }, //  2 NT LSB
		// extra entries for MSB sections				        
		{   1,    0,   5, UINT_MAX,                             0, UINT_MAX}, //  3 NT MSB
	} };

	unsigned char id;
	long tk_prev;
	pony_time_epoch epoch;

	// store current satellite date, if present
	epoch.Y = pony_gnss_io_ublox_round(sat->eph[epoch_eph+0]);
	epoch.M = pony_gnss_io_ublox_round(sat->eph[epoch_eph+1]);
	epoch.D = pony_gnss_io_ublox_round(sat->eph[epoch_eph+2]);
	// store tb
	epoch.h = pony_gnss_io_ublox_round(sat->eph[epoch_eph+3]);
	epoch.m = pony_gnss_io_ublox_round(sat->eph[epoch_eph+4]);
	epoch.s = pony_gnss_io_ublox_round(sat->eph[epoch_eph+5]);
	// store tk
	tk_prev = pony_gnss_io_ublox_round(sat->eph[tk_eph]);
	// message string id
	id = (subframe->dwrd[msg_id_word]>>msg_id_shift) & 0x0f;
	// regular data processing
	if (id < 1 || id > 4) return 0; // message id invalid or not supported
	if ( !pony_gnss_io_ublox_nav_subframe_parse_data(sat->eph, subframe->dwrd, (pony_gnss_io_ublox_nav_subframe_conversion_entry *)(&(ctable[id-1])), entries_count[id-1], 0) )
		return 0;
	// special entries handling
	switch (id) {
		case 1:
			// tk + nd*86400 (seconds into utc week)
			pony_time_epoch2gps(NULL, &(sat->eph[tk_eph]), &epoch); // secondsi nto week
			sat->eph[tk_eph] -= epoch.h*3600.0 + epoch.m*60 + epoch.s; // from day beginning
			sat->eph[tk_eph] += sat->eph[ ctable[0][tk_h_entry].eph ]*3600 + sat->eph[ ctable[0][tk_s_entry].eph ];
			sat->eph[tk_eph] -= dt_utc_s;	// to UTC
			if (sat->eph[tk_eph] < 0)		// week rollover
				sat->eph[tk_eph] += pony->gnss_const.sec_in_w;
			// restore tb
			sat->eph[ ctable[0][tk_h_entry].eph ] = epoch.h, sat->eph[ ctable[0][tk_s_entry].eph ] = epoch.s;
			break;
		case 2:
			// copy tb to eph_counter bitfield and restore satellite time until full subframe set is collected
			sat->eph_counter &= ~(tb_mask<<tb_shift); // drop bits
			sat->eph_counter |= pony_gnss_io_ublox_round(sat->eph[ ctable[1][tb_entry].eph ]/ctable[1][tb_entry].scale)<<tb_shift;
			sat->eph[ ctable[1][tb_entry].eph ] = epoch.m;
			break;
		case 3: break;
		case 4: 
			// tauN x -1
			sat->eph[ ctable[3][tau_entry].eph ] *= -1; 
			// copy NT to eph_counter bitfield and restore satellite date until full subframe set is collected
			sat->eph_counter &= ~(NT_mask<<NT_shift); // drop bits
			sat->eph_counter |= (unsigned long)(pony_gnss_io_ublox_round(sat->eph[ ctable[3][NT_entry].eph ]/ctable[3][NT_entry].scale))<<NT_shift;
			sat->eph[ ctable[3][NT_entry].eph ] = epoch.D;
			break;
	}
	// check whether all subframes have been collected
	if (tk_prev != pony_gnss_io_ublox_round(sat->eph[tk_eph]))
		sat->eph_counter &= ~eph_counter_full; // new dataset has started, reset counting bits
	sat->eph_counter |= (0x01<<(id-1));
	if ((sat->eph_counter & eph_counter_full) == eph_counter_full) {
		// tb from ephemeris counter bitfield
		sat->eph[ ctable[1][tb_entry].eph ] = ((sat->eph_counter>>tb_shift) & tb_mask)*ctable[1][tb_entry].scale;
		// NT from ephemeris counter bitfield
		sat->eph[ ctable[3][NT_entry].eph ] = ((sat->eph_counter>>NT_shift) & NT_mask)*ctable[3][NT_entry].scale;
		// tb to time of day
		sat->eph[epoch_eph+3] = pony_gnss_io_ublox_round(sat->eph[ ctable[1][tb_entry].eph ])/60 - dt_utc_h;	// hours in UTC
		sat->eph[epoch_eph+4] = pony_gnss_io_ublox_round(sat->eph[ ctable[1][tb_entry].eph ])%60;				// minutes
		sat->eph[epoch_eph+5] = 0;																		// seconds
		// day rollover handling
		if (sat->eph[epoch_eph+3] < 0) {
			sat->eph[epoch_eph+3] += 24;
			sat->eph[ ctable[3][NT_entry].eph ] -= 1;
			if (sat->eph[ ctable[3][NT_entry].eph ] < 1) { // 4-year cycle rollover
				sat->eph[ ctable[3][NT_entry].eph ] += 1461;
				epoch.Y -= 4;
			}
		}
		// NT to date
		pony_gnss_io_ublox_glonass_day2date(&epoch, pony_gnss_io_ublox_round(sat->eph[ ctable[3][NT_entry].eph ]));
		sat->eph[epoch_eph+0] = epoch.Y, sat->eph[epoch_eph+1] = epoch.M, sat->eph[epoch_eph+2] = epoch.D;
		// ephemeris validity flag
		sat->eph_valid = 1;
		// drop the counter
		sat->eph_counter = 0;
	}

	return 1;
}

	// Galileo subframes I/NAV E1-B/E5b-I
#define PONY_GNSS_IO_UBLOX_GAL_NAV_SUBFRAME_MAX_TOTAL_CONVERSION_ENTRIES 12
char pony_gnss_io_ublox_nav_subframe_parser_gal(pony_gnss_sat *sat, pony_gnss_io_ublox_RXM_SFRBX_block *subframe) {

	const unsigned char 
		even_word         = 0,
		 odd_word         = 4,
		even_odd_shift    = 31,
		msg_type_word     = 0,
		msg_type_shift    = 24,
		msg_type_mask     = 0x3f,
		health_bits_count = 6, 
		eph_counter_full  = 0x1f;
	const unsigned short 
		wk_rollover = 0x1000, // GST/GAL week rollover interval
		wk_shift    = 0x0400, // GST week shift to align with GPS week
		DS_bitfield = 0x0201; // data source I/NAV E1-B, bit 0 and bit 9 set
	const size_t 
		  epoch_eph  = 0,
		     DS_eph  = 26,
		    IOD_eph  = 9,
		  SISA_entry = 7,
		   toc_entry = 3,
		  week_entry = 3,
		health_entry = 2,
		entries_count[] = {5, 5, 8, 7, 5};  // least significant bits entries count (extra msb-s not included)
	// conversion tables as in Sections 4.3.5 and 5.1 of Galileo Open Service Signal In Space Interface Control Document OS-SIS-ICD, Issue 1.2 (November 2015), pp. 37, 43
	const pony_gnss_io_ublox_nav_subframe_conversion_entry ctable[][PONY_GNSS_IO_UBLOX_GAL_NAV_SUBFRAME_MAX_TOTAL_CONVERSION_ENTRIES] = { {
		// I/NAV word type 1 entries (Table 39, OS-SIS-ICD, Issue 1.2 (November 2015), p. 37
		// wrd shift bits msb_entry_index                    scale  eph
		{   0,   14,  10, UINT_MAX,                              1,  9      }, //  0 IODnav
		{   0,    0,  14, UINT_MAX,                             60, 17      }, //  1 toe,       x 60
		{   1,    0,  32, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 12      }, //  2 M0,        x 2^-31 x pi
		{   2,    0,  32, UINT_MAX,  PONY_GNSS_IO_UBLOX_TWO_m31/4 , 14      }, //  3 e,         x 2^-33
		{   4,   16,  14,        5,  PONY_GNSS_IO_UBLOX_TWO_m19   , 16      }, //  4 sqrtA LSB, x 2^-19
		// extra entries for MSB sections										  
		{   3,   14,  18, UINT_MAX,                              0, UINT_MAX}, //  5 sqrtA MSB
	}, {
		// I/NAV word type 2 entries (Table 40, OS-SIS-ICD, Issue 1.2 (November 2015), p. 37
		// wrd shift bits msb                                scale  eph
		{   0,   14,  10, UINT_MAX,                              1,  9      }, //  0 IODnav
		{   1,   14,  18,        5, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 19      }, //  1 Om0   LSB, x 2^-31 x pi
		{   2,   14,  18,        6, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 21      }, //  2 i0    LSB, x 2^-31 x pi
		{   3,   14,  18,        7, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 23      }, //  3 omega LSB, x 2^-31 x pi
		{   4,   16,  14, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 25      }, //  4 idot,      x 2^-43 x pi
		// extra entries for MSB sections										  
		{   0,    0,  14, UINT_MAX,                              0, UINT_MAX}, //  5 Om0   MSB
		{   1,    0,  14, UINT_MAX,                              0, UINT_MAX}, //  6 i0    MSB
		{   2,    0,  14, UINT_MAX,                              0, UINT_MAX}, //  7 omega MSB
	}, {
		// I/NAV word type 3 entries (Table 41, OS-SIS-ICD, Issue 1.2 (November 2015), p. 38
		// wrd shift bits msb                                scale  eph
		{   0,   14,  10, UINT_MAX,                              1,  9      }, //  0 IODnav
		{   1,   22,  10,        8, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 24      }, //  1 Omdot LSB, x 2^-43 x pi
		{   1,    6,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 11      }, //  2 Delta n,   x 2^-43 x pi
		{   2,   22,  10,        9, -PONY_GNSS_IO_UBLOX_TWO_m29   , 13      }, //  3 Cuc   LSB, x 2^-29
		{   2,    6,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 15      }, //  4 Cus,       x 2^-29
		{   3,   22,  10,       10, -PONY_GNSS_IO_UBLOX_TWO_m05   , 22      }, //  5 Crc   LSB, x 2^-5
		{   4,   22,   8,       11, -PONY_GNSS_IO_UBLOX_TWO_m05   , 10      }, //  6 Crs   LSB, x 2^-5
		{   4,   14,   8, UINT_MAX,                              1, 29      }, //  7 SISA
		// extra entries for MSB sections										  
		{   0,    0,  14, UINT_MAX,                              0, UINT_MAX}, //  8 Omdot MSB
		{   1,    0,   6, UINT_MAX,                              0, UINT_MAX}, //  9 Cuc   MSB
		{   2,    0,   6, UINT_MAX,                              0, UINT_MAX}, // 10 Crc   MSB
		{   3,   14,   8, UINT_MAX,                              0, UINT_MAX}, // 11 Crs   MSB
	}, {
		// I/NAV word type 4 entries (Table 42, OS-SIS-ICD, Issue 1.2 (November 2015), p. 38
		// wrd shift bits msb                                scale  eph
		{   0,   14,  10, UINT_MAX,                              1,  9      }, //  0 IODnav
		{   1,   24,   8,        7, -PONY_GNSS_IO_UBLOX_TWO_m29   , 18      }, //  1 Cic   LSB, x 2^-29
		{   1,    8,  16, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m29   , 20      }, //  2 Cis,       x 2^-29
		{   2,   26,   6,        8,                             60,  5      }, //  3 toc   LSB, x 60
		{   3,   27,   5,        9, -PONY_GNSS_IO_UBLOX_TWO_m31/8 ,  6      }, //  4 af0   LSB, x 2^-34
		{   4,   22,   8,       10, -PONY_GNSS_IO_UBLOX_TWO_m43/8 ,  7      }, //  5 af1   LSB, x 2^-46
		{   4,   16,   6, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m55/16,  8      }, //  6 af2,       x 2^-59
		// extra entries for MSB sections										  
		{   0,    0,   8, UINT_MAX,                              0, UINT_MAX}, //  7 Cic   MSB
		{   1,    0,   8, UINT_MAX,                              0, UINT_MAX}, //  8 toc   MSB
		{   2,    0,  26, UINT_MAX,                              0, UINT_MAX}, //  9 af0   MSB
		{   3,   14,  13, UINT_MAX,                              0, UINT_MAX}, // 10 af1   MSB
	}, {
		// I/NAV word type 5 entries (Table 43, OS-SIS-ICD, Issue 1.2 (November 2015), p. 38
		// wrd shift bits msb                                scale  eph
		{   1,    5,  10, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m31/2 , 31      }, //  0 BGD(E1/E5a),     x 2^-32
		{   2,   27,   5,        5, -PONY_GNSS_IO_UBLOX_TWO_m31/2 , 32      }, //  1 BGD(E1,E5b) LSB, x 2^-32
		{   2,   21,   6, UINT_MAX,                              1, 30      }, //  2 health flags
		{   2,    9,  12, UINT_MAX,                              1, 27      }, //  3 WN
		{   3,   21,  11,        6,                              1, 33      }, //  4 TOW         LSB
		// extra entries for MSB sections				  	      					    
		{   1,    0,   5, UINT_MAX,                              0, UINT_MAX}, //  5 BGD(E1,E5b) MSB
		{   2,    0,   9, UINT_MAX,                              0, UINT_MAX}, //  6 TOW         MSB
	} };

	unsigned char id;
	unsigned short health_bits = 0;
	int week0, IODprev;
	double SISA;
	pony_time_epoch epoch = {0,0,0,0,0,0};

	// store current satellite date, if present
	epoch.Y = pony_gnss_io_ublox_round(sat->eph[epoch_eph+0]);
	epoch.M = pony_gnss_io_ublox_round(sat->eph[epoch_eph+1]);
	epoch.D = pony_gnss_io_ublox_round(sat->eph[epoch_eph+2]);
	epoch.h = pony_gnss_io_ublox_round(sat->eph[epoch_eph+3]);
	epoch.m = pony_gnss_io_ublox_round(sat->eph[epoch_eph+4]);
	epoch.s = pony_gnss_io_ublox_round(sat->eph[epoch_eph+5]);
	if (!pony_time_epoch2gps((unsigned int *)(&week0), NULL, &epoch))
		week0 = -1;
	// store IODnav
	IODprev = pony_gnss_io_ublox_round(sat->eph[IOD_eph]);
	// check even/odd bits
	if ( ((subframe->dwrd[even_word]>>even_odd_shift) & 0x01) != 0 || ((subframe->dwrd[odd_word]>>even_odd_shift) & 0x01) != 1 ) 
		return 0;
	// message type id
	id = (subframe->dwrd[msg_type_word]>>msg_type_shift) & msg_type_mask;
	// regular data processing
	if (id < 1 || id > 5) return 0; // subframe id invalid or not supported
	if ( !pony_gnss_io_ublox_nav_subframe_parse_data(sat->eph, subframe->dwrd, (pony_gnss_io_ublox_nav_subframe_conversion_entry *)(&(ctable[id-1])), entries_count[id-1], 1) )
		return 0;
	// special entries handling
	switch (id) { 
		case 1: break;
		case 2: break;
		case 3:  
			// SISA index to signal in space accuracy 
			SISA = sat->eph[ ctable[2][SISA_entry].eph ];
			sat->eph[ ctable[2][SISA_entry].eph ] = 
				  (SISA <=  50) ? (       SISA     *0.01) : 
				( (SISA <=  75) ? (0.5 + (SISA- 50)*0.02) : 
				( (SISA <= 100) ? (1.0 + (SISA- 75)*0.04) : 
				( (SISA <= 125) ? (2.0 + (SISA-100)*0.16) : -1) ) ); // -1 for NAPA state - no accuracy prediction available
			break;
		case 4: 
			// toc seconds to time of day and date, if available
			if (week0 < 0) { // no date available
				// remove whole days
				sat->eph[ ctable[3][toc_entry].eph ] -= ((int)(sat->eph[ ctable[3][toc_entry].eph ]/pony->gnss_const.sec_in_d))*pony->gnss_const.sec_in_d;
				// hours
				sat->eph[epoch_eph+3] = (int)(sat->eph[ ctable[3][toc_entry].eph ]/3600);
				// remove whole hours
				sat->eph[ ctable[3][toc_entry].eph ] -= sat->eph[epoch_eph+3]*3600;
				// minutes
				sat->eph[epoch_eph+4] = (int)(sat->eph[ ctable[3][toc_entry].eph ]/60);
				// seconds
				sat->eph[epoch_eph+5] = sat->eph[ ctable[3][toc_entry].eph ] - sat->eph[epoch_eph+4]*60;
			}
			else {
				pony_time_gps2epoch(&epoch, week0, sat->eph[ ctable[3][toc_entry].eph ]);
				sat->eph[epoch_eph+0] = epoch.Y;
				sat->eph[epoch_eph+1] = epoch.M;
				sat->eph[epoch_eph+2] = epoch.D;
				sat->eph[epoch_eph+3] = epoch.h;
				sat->eph[epoch_eph+4] = epoch.m;
				sat->eph[epoch_eph+5] = epoch.s;
			}
			break;
		case 5: 
			// GST week to GAL week (aligned to GPS week)
			sat->eph[ ctable[4][week_entry].eph ] += wk_shift;
			if (week0 >= 0) {
				// floor(.5+w0/wr)*wr + mod(w+wr/2,wr)-wr/2 -- closest to week0
				week0 = pony_gnss_io_ublox_round(((double)week0)/wk_rollover)*wk_rollover + ((int)(sat->eph[ ctable[4][week_entry].eph ]) + wk_rollover/2)%wk_rollover - wk_rollover/2;
				sat->eph[ ctable[4][week_entry].eph ] = (double)week0;
			}
			else
				week0 = pony_gnss_io_ublox_round(sat->eph[ ctable[4][week_entry].eph ]);
			// week number check
			if (week0 <= 0)
				return 0;
			// recalculate toc
			pony_time_epoch2gps(NULL, &(sat->eph[ ctable[3][toc_entry].eph ]), &epoch);
			pony_time_gps2epoch(&epoch, (unsigned int)week0, sat->eph[ ctable[3][toc_entry].eph ]);
			sat->eph[epoch_eph+0] = epoch.Y;
			sat->eph[epoch_eph+1] = epoch.M;
			sat->eph[epoch_eph+2] = epoch.D;
			sat->eph[epoch_eph+3] = epoch.h;
			sat->eph[epoch_eph+4] = epoch.m;
			sat->eph[epoch_eph+5] = epoch.s;
			// health bits
			health_bits = (unsigned short)(pony_gnss_io_ublox_round(sat->eph[ ctable[4][health_entry].eph ]));
			health_bits  |= (health_bits&0x01)<< health_bits_count;   // bit  0 remains in place, move to MSB
			health_bits  |= (health_bits&0x0c)<<(health_bits_count-1); // bits 2-3 to bits 1-2, move to MSB
			health_bits  |= (health_bits&0x02)<<(health_bits_count+5); // bit  1 to bit 6, move to MSB
			health_bits  |= (health_bits&0x30)<<(health_bits_count+3); // bits 4-5 to bits 7-8, move to MSB
			sat->eph[ ctable[4][health_entry].eph ] = (double)(health_bits>>health_bits_count); // move back to LSB
			break;
	}
	// check whether all subframes have been collected
	if (IODprev != pony_gnss_io_ublox_round(sat->eph[IOD_eph])) {// new issue of data has started
		sat->eph_valid = 0;   // stop using the satellite
		sat->eph_counter = 0;
	}
	sat->eph_counter |= (0x01<<(id-1));
	if ((sat->eph_counter & eph_counter_full) == eph_counter_full) {
		sat->eph_valid = 1;
		sat->eph_counter = 0; // drop the counter
	}
	// data source flags
	sat->eph[DS_eph] = (double)((short)(sat->eph[DS_eph]) | DS_bitfield); // set E1-B and bit 9

	return 1;

}

	// BeiDou subframes
#define PONY_GNSS_IO_UBLOX_BDS_NAV_SUBFRAME_MAX_TOTAL_CONVERSIION_ENTRIES 18
char pony_gnss_io_ublox_nav_subframe_parser_bds(pony_gnss_sat *sat, pony_gnss_io_ublox_RXM_SFRBX_block *subframe) {

	const unsigned char 
		eph_counter_full = 0x07,
		toe_MSB_shift    = 30,
		toe_LSB_shift    = 15;
	const unsigned long
		toe_MSB_mask = 0x0003,
		toe_LSB_mask = 0x7fff;
	const unsigned short
		pre_word = 0, pre_shift = 19, pre_mask = 0x07ff, preamble = 0x0712, // preamble
		fra_word = 0, fra_shift = 12, fra_mask = 0x0007,					// frame id
		wk_shift = 1356, wk_rollover = 0x2000;
	const double 
		URA_sqrt2minus1 = 0.41;
	const size_t 
		  epoch_eph     = 0,
		    toe_eph     = 17,
		   URAI_entry   = 3,
		   week_entry   = 4,
		    toc_entry   = 5,
		toe_MSB_entry   = 9,
		toe_LSB_entry   = 1,
		entries_count[] = {12, 10, 9};  // least significant bits entries count (extra msb-s not included)
	// conversion tables as in Section 5.2.4 of BeiDou Interface Control Document BDS-SIS-ICD-2.1 (November 2016), p. 23
	const pony_gnss_io_ublox_nav_subframe_conversion_entry ctable[][PONY_GNSS_IO_UBLOX_BDS_NAV_SUBFRAME_MAX_TOTAL_CONVERSIION_ENTRIES] = { {
		// subframe 1 entries (Tables 5-4, 5-6, 5-7, 5-8 of BDS-SIS-ICD-2.1 (November 2016), p. 23
		// wrd shift bits msb_entry_index                    scale  eph
		{   1,   18,  12,       12,                              1, 33      }, //  0 SOW     LSB  (seconds of BDS week least significant bits)
		{   1,   17,   1, UINT_MAX,                              1, 30      }, //  1 SatH1        (health flag)
		{   1,   12,   5, UINT_MAX,                              1, 34      }, //  2 AODC         (age of data - clock)
		{   1,    8,   4, UINT_MAX,                              1, 29      }, //  3 URAI         (user range accuracy indicator)
		{   2,   17,  13, UINT_MAX,                              1, 27      }, //  4 WN           (BDS week number)
		{   3,   22,   8,       13,                              8,  5      }, //  5 toc,         x 8
		{   3,   12,  10, UINT_MAX,                         -1e-10, 31      }, //  6 TGD1,        x 0.1 nsec
		{   4,   24,   6,       14,                         -1e-10, 32      }, //  7 TGD2,        x 0.1 nsec
		{   7,   15,  11, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m66   ,  8      }, //  8 a2,          x 2^-66
		{   8,   13,  17,       15, -PONY_GNSS_IO_UBLOX_TWO_m31/4 ,  6      }, //  9 a0      LSB, x 2^-33
		{   9,   13,  17,       16, -PONY_GNSS_IO_UBLOX_TWO_m50   ,  7      }, // 10 a1      LSB, x 2^-50
		{   9,    8,   5, UINT_MAX,                              1,  9      }, // 11 AODE         (age of data - ephemeris)
		// extra entries for MSB sections			  	      						 	    
		{   0,    4,   8, UINT_MAX,                              0, UINT_MAX}, // 12 SOW     MSB  (seconds of BDS week most significant bits)
		{   2,    8,   9, UINT_MAX,                              0, UINT_MAX}, // 13 toc     MSB
		{   3,    8,   4, UINT_MAX,                              0, UINT_MAX}, // 14 TGD2    MSB
		{   7,    8,   7, UINT_MAX,                              0, UINT_MAX}, // 15 a0      MSB
		{   8,    8,   5, UINT_MAX,                              0, UINT_MAX}, // 16 a1      MSB
	}, {
		// subframe 2 entries (Table 5-10 of BDS-SIS-ICD-2.1 (November 2016), p. 34
		// wrd shift bits msb_entry_index                    scale  eph
		{   1,   18,  12,       10,                              1, 33      }, //  0 SOW     LSB  (seconds of BDS week least significant bits)
		{   2,   24,   6,       11, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 11      }, //  1 Delta n LSB, x 2^-43 x pi
		{   3,   28,   2,       12, -PONY_GNSS_IO_UBLOX_TWO_m31   , 13      }, //  2 Cuc     LSB, x 2^-31
		{   4,   18,  12,       13, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 12      }, //  3 M0      LSB, x 2^-31 x pi
		{   5,    8,  22,       14, +PONY_GNSS_IO_UBLOX_TWO_m31/4 , 14      }, //  4 e       LSB, x 2^-33
		{   6,   12,  18, UINT_MAX, -PONY_GNSS_IO_UBLOX_TWO_m31   , 15      }, //  5 Cus     LSB, x 2^-31
		{   7,   16,  14,       15, -PONY_GNSS_IO_UBLOX_TWO_m05/2 , 22      }, //  6 Crc     LSB, x 2^-6
		{   8,   20,  10,       16, -PONY_GNSS_IO_UBLOX_TWO_m05/2 , 10      }, //  7 Crs     LSB, x 2^-6
		{   9,   10,  20,       17, +PONY_GNSS_IO_UBLOX_TWO_m19   , 16      }, //  8 sqrtA   LSB, x 2^-19
		{   9,    8,   2, UINT_MAX,                   (0x01<<15)*8, 17      }, //  9 toe     MSB, x 8*2^15
		// extra entries for MSB sections											   
		{   0,    4,   8, UINT_MAX,                              0, UINT_MAX}, // 10 SOW     MSB  (seconds of BDS week most significant bits)
		{   1,    8,  10, UINT_MAX,                              0, UINT_MAX}, // 11 Delta n MSB
		{   2,    8,  16, UINT_MAX,                              0, UINT_MAX}, // 12 Cuc     MSB
		{   3,    8,  20, UINT_MAX,                              0, UINT_MAX}, // 13 M0      MSB
		{   4,    8,  10, UINT_MAX,                              0, UINT_MAX}, // 14 e       MSB
		{   6,    8,   4, UINT_MAX,                              0, UINT_MAX}, // 15 Crc     MSB
		{   7,    8,   8, UINT_MAX,                              0, UINT_MAX}, // 16 Crs     MSB
		{   8,    8,  12, UINT_MAX,                              0, UINT_MAX}, // 17 sqrtA   MSB
	}, {
		// subframe 3 entries (Table 5-10 of BDS-SIS-ICD-2.1 (November 2016), p. 34
		// wrd shift bits msb_entry_index                    scale  eph
		{   1,   18,  12,        9,                              1, 33      }, //  0 SOW     LSB  (seconds of BDS week least significant bits)
		{   2,   25,   5,       10,                              8, 17      }, //  1 toe     LSB
		{   3,   15,  15,       11, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 21      }, //  2 i0      LSB, x 2^-31 x pi
		{   4,   19,  11,       12, -PONY_GNSS_IO_UBLOX_TWO_m31   , 18      }, //  3 Cic     LSB, x 2^-31
		{   5,   17,  13,       13, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 24      }, //  4 Om dot  LSB, x 2^-43 x pi
		{   6,   21,   9,       14, -PONY_GNSS_IO_UBLOX_TWO_m31   , 20      }, //  5 Cis     LSB, x 2^-31
		{   7,   29,   1,       15, -PONY_GNSS_IO_UBLOX_TWO_m43xPI, 25      }, //  6 i dot   LSB, x 2^-43 x pi
		{   8,   19,  11,       16, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 19      }, //  7 Om0     LSB, x 2^-31 x pi
		{   9,    9,  21,       17, -PONY_GNSS_IO_UBLOX_TWO_m31xPI, 23      }, //  8 omega   LSB, x 2^-31 x pi
		// extra entries for MSB sections										    
		{   0,    4,   8, UINT_MAX,                              0, UINT_MAX}, //  9 SOW     MSB  (seconds of BDS week most significant bits)
		{   1,    8,  10, UINT_MAX,                              0, UINT_MAX}, // 10 toe     ISB  (intermediate bits)
		{   2,    8,  17, UINT_MAX,                              0, UINT_MAX}, // 11 i0      MSB
		{   3,    8,   7, UINT_MAX,                              0, UINT_MAX}, // 12 Cic     MSB
		{   4,    8,  11, UINT_MAX,                              0, UINT_MAX}, // 13 Om dot  MSB
		{   5,    8,   9, UINT_MAX,                              0, UINT_MAX}, // 14 Cis     MSB
		{   6,    8,  13, UINT_MAX,                              0, UINT_MAX}, // 15 i dot   MSB
		{   7,    8,  21, UINT_MAX,                              0, UINT_MAX}, // 16 Om0     MSB
		{   8,    8,  11, UINT_MAX,                              0, UINT_MAX}, // 17 omega   MSB
	} };

	unsigned char id;
	unsigned short N;
	int week0;
	double toe;
	pony_time_epoch epoch = {0,0,0,0,0,0}, epoch0;

	// store current satellite toc, if present
	epoch.Y = pony_gnss_io_ublox_round(sat->eph[epoch_eph+0]);
	epoch.M = pony_gnss_io_ublox_round(sat->eph[epoch_eph+1]);
	epoch.D = pony_gnss_io_ublox_round(sat->eph[epoch_eph+2]);
	epoch.h = pony_gnss_io_ublox_round(sat->eph[epoch_eph+3]);
	epoch.m = pony_gnss_io_ublox_round(sat->eph[epoch_eph+4]);
	epoch0 = epoch;
	if (!pony_time_epoch2gps((unsigned int *)(&week0), NULL, &epoch))
		week0 = -1;
	else
		week0 -= wk_shift;
	// store toe
	toe = sat->eph[toe_eph];
	// preamble
	if ( ((subframe->dwrd[pre_word]>>pre_shift) & pre_mask) != preamble ) 
		return 0;
	// subframe id
	id = (unsigned char)((subframe->dwrd[fra_word]>>fra_shift) & fra_mask);
	// regular data processing
	if (id < 1 || id > 3) return 0; // subframe id invalid or not supported
	if ( !pony_gnss_io_ublox_nav_subframe_parse_data(sat->eph, subframe->dwrd, (pony_gnss_io_ublox_nav_subframe_conversion_entry *)(&(ctable[id-1])), entries_count[id-1], 1) )
		return 0;
	// special entries handling
	switch (id) { 
		case 1:
			// URA index to user range accuracy: URA = 2^(1+N/2) [N = 0..5], 2^(N-2) [N = 6..15]
			N = (unsigned short)(sat->eph[ ctable[0][URAI_entry].eph ]);
			sat->eph[ ctable[0][URAI_entry].eph ] = (N < 6) ? (0x01<<(1 + N/2))*(1 + URA_sqrt2minus1*(N%2)) : (0x01<<(N-2));
			// toc
			if (week0 >= 0) {
				// floor(.5+w0/wr)*wr + mod(w+wr/2,wr)-wr/2 -- closest to week0
				week0 = pony_gnss_io_ublox_round(((double)week0)/wk_rollover)*wk_rollover + ((int)(sat->eph[ ctable[0][week_entry].eph ]) + wk_rollover/2)%wk_rollover - wk_rollover/2;
				sat->eph[ ctable[0][week_entry].eph ] = (double)week0;
			}
			else
				week0 = pony_gnss_io_ublox_round(sat->eph[ ctable[0][week_entry].eph ]);
			// week number check
			if (week0 <= 0)
				return 0;
			pony_time_gps2epoch(&epoch, (unsigned int)week0 + wk_shift, sat->eph[ ctable[0][toc_entry].eph ]);
			sat->eph[epoch_eph+0] = epoch.Y;
			sat->eph[epoch_eph+1] = epoch.M;
			sat->eph[epoch_eph+2] = epoch.D;
			sat->eph[epoch_eph+3] = epoch.h;
			sat->eph[epoch_eph+4] = epoch.m;
			sat->eph[epoch_eph+5] = epoch.s;
			break;
		case 2: 
			// move toe most significant bits to eph_counter for further processing
			sat->eph_counter &= ~(toe_MSB_mask<<toe_MSB_shift); // reset toe MSB bits in counter
			sat->eph_counter |= ((unsigned long)(sat->eph[ ctable[1][toe_MSB_entry].eph ] / ctable[1][toe_MSB_entry].scale))<<toe_MSB_shift;
			sat->eph[toe_eph] = toe;
			break;
		case 3: 
			// move toe least significant bits to eph_counter for further processing
			sat->eph_counter &= ~(toe_LSB_mask<<toe_LSB_shift); // reset toe MSB bits in counter
			sat->eph_counter |= ((unsigned long)(sat->eph[ ctable[2][toe_LSB_entry].eph ] / ctable[2][toe_LSB_entry].scale))<<toe_LSB_shift;
			sat->eph[toe_eph] = toe;
			break;
	}
	// check whether all subframes have been collected
	if (   epoch0.Y != pony_gnss_io_ublox_round(sat->eph[epoch_eph+0])
		|| epoch0.M != pony_gnss_io_ublox_round(sat->eph[epoch_eph+1])
		|| epoch0.D != pony_gnss_io_ublox_round(sat->eph[epoch_eph+2])
		|| epoch0.h != pony_gnss_io_ublox_round(sat->eph[epoch_eph+3])
		|| epoch0.m != pony_gnss_io_ublox_round(sat->eph[epoch_eph+4])) {// new issue of data has started
		sat->eph_valid = 0;                    // stop using the satellite
		sat->eph_counter &= ~eph_counter_full; // reset frame counting bits
	}
	sat->eph_counter |= (0x01<<(id-1));
	if ((sat->eph_counter & eph_counter_full) == eph_counter_full) {
		// move toe from eph_counter to ephemeris
		sat->eph[toe_eph] = 
			((sat->eph_counter>>toe_MSB_shift)&toe_MSB_mask)*ctable[1][toe_MSB_entry].scale +
			((sat->eph_counter>>toe_LSB_shift)&toe_LSB_mask)*ctable[2][toe_LSB_entry].scale;
		// set flag, reset counter
		sat->eph_valid = 1;
		sat->eph_counter = 0;
	}

	return 1;
}




// internal routines
	// GLONASS day into 4-year cycle and tb to time epoch closest to given year, see A.3.1.3 in ICD L1,L2 GLONASS Ed. 5.1 2008, p. 57
void pony_gnss_io_ublox_glonass_day2date(pony_time_epoch *epoch, unsigned int day) {

                               // Mar Apr May Jun Jul Aug Sep Oct Nov Dec Jan
	const unsigned short dom[] = { 30, 60, 91,121,152,183,213,244,274,305,336}; // days of year since March 1 when months end
	
	// GLONASSS clock starts in 1996
	epoch->Y = (epoch->Y < 1996) ? 1996 : (epoch->Y/4)*4; // latest multiple of four not less than 1996

	// year
	if (day > 366)
		epoch->Y += (day - 2)/365;
	// day of year since March 1 starting from zero
	epoch->D =  (day < 61) ? (day + 305) : (day -  61)%365;
	// month
	epoch->M = epoch->D/31; // exact for 340 days out of 366, less by one for the rest 26
	if (epoch->M < 11 && epoch->D > dom[epoch->M]) // if underestimated, adjust by one
		epoch->M++;
	// day of year (from zero) to day of month (from one)
	epoch->D += (epoch->M > 0) ? -dom[epoch->M-1] : 1;
	// move starting day from March 1 to January 1
	epoch->M += (epoch->M > 9) ? -9 : 3;

}

	// observation types handling
char pony_gnss_io_ublox_obs_types_allocate(char (**obs_types)[4], size_t *obs_count, pony_gnss_sat *sat, const size_t sat_count, const pony_gnss_io_ublox_signal_id_converson_table_struct *sig) {

	enum obs_type_id {code, phase, doppler, SS, obs_type_count};

	const char obs_type[] = {'C','L','D','S'};

	size_t i;

	if (sig == NULL || obs_type == NULL || obs_count == NULL || sat == NULL)
		return 0;

	*obs_count = sig->types*obs_type_count;
	*obs_types = (char (*)[4])calloc(*obs_count, sizeof (char [4]));
	if (*obs_types == NULL)
		return 0;

	for (i = 0; i < *obs_count; i++) {
		(*obs_types)[i][0] = obs_type[i%obs_type_count];
		(*obs_types)[i][1] = sig->rnx_sig_id[i/obs_type_count][0];
		(*obs_types)[i][2] = sig->rnx_sig_id[i/obs_type_count][1];
	}

	for (i = 0; i < sat_count; i++) {
		sat[i].obs       = (double *)calloc(*obs_count, sizeof(double));
		sat[i].obs_valid = (char   *)calloc(*obs_count, sizeof(char  ));
		if (sat[i].obs == NULL || sat[i].obs_valid == NULL)
			return 0;
	}

	return 1;

}




	// drop satellite flags
void pony_gnss_io_ublox_drop_flags_pony_sats(pony_gnss_sat *sat, const size_t sat_count, const size_t obs_count) {

	size_t i, s;

	if (sat == NULL)
		return; // nothing to do

	for (s = 0; s < sat_count; s++) {
		sat[s].t_em_valid = 0;
		sat[s].x_valid = 0;
		sat[s].v_valid = 0;
		for (i = 0; i < obs_count; i++)
			sat[s].obs_valid[i] = 0;
	}

}




	// drop solution flags
void pony_gnss_io_ublox_drop_flags_pony_sol(pony_sol *sol) {

	sol->x_valid	= 0;
	sol->llh_valid	= 0;
	sol->v_valid	= 0;
	sol->q_valid	= 0;
	sol->L_valid	= 0;
	sol->rpy_valid	= 0;
	sol->dt_valid	= 0;

}



	// uBlox 8-bit Fletcher recursive checksum
void pony_gnss_io_ublox_checksum_recurse(unsigned char *cs,  unsigned char byte) {
		cs[0] = 0xff & ( (0xff & (cs[0])) + (0xff & byte ) );
		cs[1] = 0xff & ( (0xff & (cs[1])) + (0xff & cs[0]) );
}




	// free memory with pointer NULL-check and NULL-assignment
void pony_gnss_io_ublox_free_null(void **ptr) 
{
	if (*ptr == NULL)
		return;
	free(*ptr);
	*ptr = NULL;
}




	// round to the nearest integer
int pony_gnss_io_ublox_round(double x) {
	return ( (int)( x >= 0.0 ? (x + 0.5) : (x - 0.5) ) );
}

	// remainder after division for doubles
double pony_gnss_io_ublox_dmod(double x, double y) {
    return x - (int)(x/y) * y;
}