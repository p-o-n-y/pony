#include <stdlib.h>
#include "pony.h"





///
///     Internal pony functions and definitions
///





pony_struct pony = { pony_bus_version,0 };


// function for comparing strings up to a given length (like strncmp from string.h) but with a limited functionality (0 - equal, 1 - not equal)
// char* s1, char* s2 are the compared strings
// substrlen is the lenght up to which strings are compared, in pony is usually the lenght of a smaller string
char pony_strncmpeff(char* s1, char* s2, int substrlen)
{
	int i;
	for (i = 0; i < substrlen; i++)
	{
		if (s1[i] != s2[i])
		{
			return 1;
		}
	}
	return 0;
}

// function for locating the beginning of a substring in a string with a given length
// char* str is the general string
// int len is the lenght of the general string
// char* substr is the substring to be located
// int substrlen is the length of the substring
char* pony_locatesubstrn(char* str, int len, char* substr, int substrlen)
{
	int n = 0;
	while (n + substrlen <= len)
	{
		if (pony_strncmpeff(str + n, substr, substrlen) == 0)
		{
			return str + n;
		}
		n++;
	}
	return NULL;
}

// function for locating the end of a substring in a string with a given length
// char* str is the general string
// int len is the lenght of the general string
// char* substr is the substring to be located
char* pony_locatesubstrendn(char* str, int len, char* substr)
{
	int n;
	char* res;

	n = 0;
	while (substr[n] != '\0')
	{
		n++;
	}
	res = pony_locatesubstrn(str, len, substr, n);
	if (res == NULL)
	{
		return NULL;
	}

	return res + n;
}

// function spesificaly for locating the beginning of a substring in given configuration group
// char* str is a part of a configuration string
// char* substr is the substring to be located
// int substrlen is the pre-calculated substring length
char* pony_locatesubstreff(char* str, char* substr, int substrlen)
{
	char* res = str;

	int in = 0;

	while ((*res) != '\0')
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

// function spesificaly for locating the end of a substring in given configuration group
// char* str is a part of a configuration string
// char* substr is the substring to be located
// int substrlen is the pre-calculated substring length
char* pony_locatesubstrendeff(char* str, char* substr, int substrlen)
{
	int in = 0;
	char* res = str;

	while ((*res) != '\0')
	{
		if (in == 0 && pony_strncmpeff(res, substr, substrlen) == 0)
		{
			return res + substrlen;
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

// function specifically for measuring a configuration group length
// char* str is the beginning of the group
int pony_cfgpartlength(char* str)
{
	return (int)(pony_locatesubstreff(str, "}", 1) - str);
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
// return value:		1 if the requested group is found
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

// function for initialising pony_dataArrays depending on their sizes
// pony_dataArray *dataarr is the pony_dataArray to be initialised
// int size is the size of the array
char pony_setDASize(pony_dataArray *dataarr, int size)
{
	dataarr->arrsize = size;
	if ((dataarr->val = (double*)calloc(sizeof(double), size)) == NULL)
	{
		return 0;
	}
	return 1;
}

// function for freeing the memory allocated to pony, should be used only as the last step of terminating pony's activity
void pony_free()
{
	int i;

	if (pony.bus.imu != NULL)
	{
		free(pony.bus.imu->f.val);
		free(pony.bus.imu->q.val);
		free(pony.bus.imu->w.val);

		free(pony.bus.imu);
	}

	if (pony.bus.gnss != NULL)
	{
		if (pony.bus.gnss->gps != NULL)
		{
			if (pony.bus.gnss->gps->sat != NULL)
			{
				for (i = 0; i < pony.bus.gnss->gps->max_sat_num; i++)
					free(pony.bus.gnss->gps->sat[i].eph.val);

				free(pony.bus.gnss->gps->sat);
			}
			free(pony.bus.gnss->gps);
		}

		if (pony.bus.gnss->glo != NULL)
		{
			if (pony.bus.gnss->glo->sat != NULL)
			{
				for (i = 0; i < pony.bus.gnss->glo->max_sat_num; i++)
						free(pony.bus.gnss->glo->sat[i].eph.val);

				free(pony.bus.gnss->glo->sat);
			}
			free(pony.bus.gnss->glo);
		}

		free(pony.bus.gnss);
	}

	free(pony.cfg);
	free(pony.plugins);

}





///
///     Core pony functions for host application
///





// add external functions as pony plugins, plugins are then called in the order of being added
//
// void(*newplugin)(void) - a pointer to a new plugin function you wish to add
//
// return value - 1 if the plugin is successfully added to pony.plugins
//                0 otherwise
char pony_add_plugin(void(*newplugin)(void))
{
	if (newplugin == NULL)
	{
		return 0;
	}

	if (pony.plugins == NULL)
	{
		pony.plugins = (void(**)(void))malloc(sizeof(void(*)(void)));
	}
	else
	{
		pony.plugins = (void(**)(void))realloc(pony.plugins, (pony.pluginsNum + 1) * sizeof(void(*)(void)));
	}

	if (pony.plugins == NULL)
	{
		return 0;
	}

	pony.plugins[pony.pluginsNum] = newplugin;
	pony.pluginsNum++;

	return 1;
}


// function for initialising pony with a user-passed configuration string
// 
// char* cfg - pony configuration string (see documentation for syntax)
//
// return value - 1 if pony is successfully initialised
//                0 otherwise
char pony_init(char* cfg)
{
	// defaults
	const int gps_max_sat_number = 36;
	const int gps_max_eph_count = 32;
	const int glo_max_sat_number = 36;
	const int glo_max_eph_count = 16;

	int i;
	int grouplen;
	char* groupptr;

	for (pony.cfglength = 0; cfg[pony.cfglength]; pony.cfglength++);

	if ((pony.cfg = (char *)malloc(sizeof(char) * (pony.cfglength + 1))) == NULL)
	{
		pony_free();
		return 0;
	}
	for (i = 0; i < pony.cfglength; i++)
		pony.cfg[i] = cfg[i];
	pony.cfg[pony.cfglength] = '\0';


	pony_locatecfggroup("", pony.cfg, pony.cfglength, &pony.bus.cfg, &pony.bus.cfglength);


	if (pony_locatecfggroup("imu:", pony.cfg, pony.cfglength, &groupptr, &grouplen))
	{
		if ((pony.bus.imu = (pony_imu*)calloc(sizeof(pony_imu), 1)) == NULL)
		{
			pony_free();
			return 0;
		}
		pony.bus.imu->cfg = groupptr;
		pony.bus.imu->cfglength = grouplen;

		if (!(
			pony_setDASize(&(pony.bus.imu->f), 3)
			&& pony_setDASize(&(pony.bus.imu->q), 4)
			&& pony_setDASize(&(pony.bus.imu->w), 3)
			))
		{
			pony_free();
			return 0;
		}

		double fs;

		if (pony_extract_double(pony.bus.imu->cfg, pony.bus.imu->cfglength, "fs = ", &fs))
		{
			pony.bus.imu->dt.val = 1 / fs;
		}
		else
		{
			pony.bus.imu->dt.val = 1;
		}

	}

	if (pony_locatecfggroup("gnss:", pony.cfg, pony.cfglength, &groupptr, &grouplen))
	{
		if ((pony.bus.gnss = (pony_gnss*)calloc(sizeof(pony_gnss), 1)) == NULL)
		{
			pony_free();
			return 0;
		}
		pony.bus.gnss->cfg = groupptr;
		pony.bus.gnss->cfglength = grouplen;


		pony_locatecfggroup("", pony.bus.gnss->cfg, pony.bus.gnss->cfglength, &(pony.bus.gnss->wcfg), &(pony.bus.gnss->wcfglength));

		if (pony_locatecfggroup("gps:", pony.bus.gnss->cfg, pony.bus.gnss->cfglength, &groupptr, &grouplen))
		{
			if ((pony.bus.gnss->gps = (pony_gnss_gps*)calloc(sizeof(pony_gnss_gps), 1)) == NULL)
			{
				pony_free();
				return 0;
			}
			pony.bus.gnss->gps->cfg = groupptr;
			pony.bus.gnss->gps->cfglength = grouplen;

			pony.bus.gnss->gps->max_sat_num = gps_max_sat_number;
			if ((pony.bus.gnss->gps->sat = (pony_gnss_sat*)calloc(sizeof(pony_gnss_sat), pony.bus.gnss->gps->max_sat_num)) == NULL)
			{
				pony_free();
				return 0;
			}
			for (i = 0; i < pony.bus.gnss->gps->max_sat_num; i++)
				pony.bus.gnss->gps->sat[i].obs = NULL;
			pony.bus.gnss->gps->max_eph_count = gps_max_eph_count;
			for (i = 0; i < pony.bus.gnss->gps->max_sat_num; i++)
			{
				if (!pony_setDASize(&pony.bus.gnss->gps->sat[i].eph, gps_max_eph_count))
				{
					pony_free();
					return 0;
				}
			}

			if (!(
				pony_setDASize(&(pony.bus.gnss->gps->iono_a), 4)
				&& pony_setDASize(&(pony.bus.gnss->gps->iono_b), 4)
				&& pony_setDASize(&(pony.bus.gnss->gps->clock_corr), 3)
				))
			{
				pony_free();
				return 0;
			}
		}

		if (pony_locatecfggroup("glo:", pony.bus.gnss->cfg, pony.bus.gnss->cfglength, &groupptr, &grouplen))
		{
			if ((pony.bus.gnss->glo = (pony_gnss_glo*)calloc(sizeof(pony_gnss_glo), 1)) == NULL)
			{
				pony_free();
				return 0;
			}
			pony.bus.gnss->glo->cfg = groupptr;
			pony.bus.gnss->glo->cfglength = grouplen;

			pony.bus.gnss->glo->max_sat_num = glo_max_sat_number;
			if ((pony.bus.gnss->glo->sat = (pony_gnss_sat*)calloc(sizeof(pony_gnss_sat), pony.bus.gnss->glo->max_sat_num)) == NULL)
			{
				pony_free();
				return 0;
			}
			for (i = 0; i < pony.bus.gnss->glo->max_sat_num; i++)
				pony.bus.gnss->glo->sat[i].obs = NULL;
			pony.bus.gnss->glo->max_eph_count = glo_max_eph_count;
			for (i = 0; i < pony.bus.gnss->glo->max_sat_num; i++)
			{
				if (!pony_setDASize(&pony.bus.gnss->glo->sat[i].eph, glo_max_eph_count))
				{
					pony_free();
					return 0;
				}
			}

			if (!(
				pony_setDASize(&(pony.bus.gnss->glo->iono_a), 4)
				&& pony_setDASize(&(pony.bus.gnss->glo->iono_b), 4)
				&& pony_setDASize(&(pony.bus.gnss->glo->clock_corr), 3)
				))
			{
				pony_free();
				return 0;
			}
		}
	}


	pony.exitplnum = -1;

	return 1;
}

// function to be called by host application in a main loop
//
// return value - 1 if pony needs to continue functioning in next iteration
//                0 if pony's activity fully terminated
char pony_step(void)
{
	int i;

	for (i = 0; i < pony.pluginsNum; i++)
	{
		pony.plugins[i]();

		if (pony.exitplnum == i)
		{
			pony.exitplnum = -1;
			pony_free();
			break;
		}

		if (pony.bus.mode < 0 && pony.exitplnum == -1)
		{
			pony.exitplnum = i;
		}
	}

	if (pony.bus.mode == 0)
	{
		pony.bus.mode = 1;
	}
	return (pony.bus.mode >= 0) || (pony.exitplnum >= 0);
}

// function to be called by host application to force pony termination
//
// return value - TBD
char pony_terminate()
{
	int i;

	pony.bus.mode = -1;

	for (i = 0; i < pony.pluginsNum; i++)
	{
		pony.plugins[i]();
	}

	pony_free();

	return 1;
}





///
///     Standard extraction functions for variables defined by configurator
///





// function for getting the length of the string that would have been obtained using this identifier on pony_extract_string function, functions are not merged as memory is normally allocated in between them
// works only for standard strings, for symbol " that does not mean the end of the string use ""
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// int* res is the pointer to the variable the data should be written to
char pony_extract_string_length(char* cfgstr, int length, char* identifier, int* res)
{
	int i = 0;
	*res = 0;
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	while (cfgstr[i] != '\"' || cfgstr[i + 1] == '\"')
	{
		if (cfgstr[i] == '\"')
		{
			i++;
		}
		i++;
		(*res)++;
	}
	return 1;
}

// function for obtaining the string by identifier, as memory should be allocated in advance this function is not merged with pony_extract_string_length function
// works only for standard strings, for symbol " that does not mean the end of the string use ""
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// char** res is the pointer to the variable the data should be written to
char pony_extract_string(char* cfgstr, int length, char* identifier, char** res)
{
	int i = 0;
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	while (cfgstr[i] != '\"' || cfgstr[i + 1] == '\"')
	{
		(*res)[i] = cfgstr[i];
		if (cfgstr[i] == '\"' && cfgstr[i + 1] == '\"')
		{
			i++;
		}
		i++;
	}
	return 1;
}

// function for obtaining symbol by identifier
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// char* res is the pointer to the variable the data should be written to
char pony_extract_char_sym(char* cfgstr, int length, char* identifier, char* res)
{
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	*res = cfgstr[0];
	return 1;
}

// function for obtaining number of type char by identifier
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// char* res is the pointer to the variable the data should be written to
char pony_extract_char_num(char* cfgstr, int length, char* identifier, char* res)
{
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	if ((cfgstr[0] - '0' > 9 || cfgstr[0] - '0' < 0) && cfgstr[0] != '-' && cfgstr[0] != '+')
	{
		return 0;
	}
	*res = (char)atoi(cfgstr);
	return 1;
}

// function for obtaining number of type short by identifier
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// short* res is the pointer to the variable the data should be written to
char pony_extract_short(char* cfgstr, int length, char* identifier, short* res)
{
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	if ((cfgstr[0] - '0' > 9 || cfgstr[0] - '0' < 0) && cfgstr[0] != '-' && cfgstr[0] != '+')
	{
		return 0;
	}
	*res = (short)atoi(cfgstr);
	return 1;
}

// function for obtaining number of type int by identifier
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// int* res is the pointer to the variable the data should be written to
char pony_extract_int(char* cfgstr, int length, char* identifier, int* res)
{
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	if ((cfgstr[0] - '0' > 9 || cfgstr[0] - '0' < 0) && cfgstr[0] != '-' && cfgstr[0] != '+')
	{
		return 0;
	}
	*res = atoi(cfgstr);
	return 1;
}

// function for obtaining number of type long by identifier
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// long* res is the pointer to the variable the data should be written to
char pony_extract_long(char* cfgstr, int length, char* identifier, long* res)
{
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	if ((cfgstr[0] - '0' > 9 || cfgstr[0] - '0' < 0) && cfgstr[0] != '-' && cfgstr[0] != '+')
	{
		return 0;
	}
	*res = atol(cfgstr);
	return 1;
}

// function for obtaining number of type float by identifier
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// float* res is the pointer to the variable the data should be written to
char pony_extract_float(char* cfgstr, int length, char* identifier, float* res)
{
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	if ((cfgstr[0] - '0' > 9 || cfgstr[0] - '0' < 0) && cfgstr[0] != '.' && cfgstr[0] != 'e' && cfgstr[0] != 'E' && cfgstr[0] != '-' && cfgstr[0] != '+')
	{
		return 0;
	}
	*res = (float)atof(cfgstr);
	return 1;
}

// function for obtaining number of type double by identifier
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// double* res is the pointer to the variable the data should be written to
char pony_extract_double(char* cfgstr, int length, char* identifier, double* res)
{
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	if ((cfgstr[0] - '0' > 9 || cfgstr[0] - '0' < 0) && cfgstr[0] != '.' && cfgstr[0] != 'e' && cfgstr[0] != 'E' && cfgstr[0] != '-' && cfgstr[0] != '+')
	{
		return 0;
	}
	*res = atof(cfgstr);
	return 1;
}

// function for obtaining boolean by identifier (true - 1, false - 0)
// char* cfgstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// char* res is the pointer to the variable the data should be written to
char pony_extract_bool(char* cfgstr, int length, char* identifier, char* res)
{
	cfgstr = pony_locatesubstrendn(cfgstr, length, identifier);
	if (cfgstr == NULL)
	{
		return 0;
	}
	if (pony_strncmpeff(cfgstr, "true", 4) || pony_strncmpeff(cfgstr, "TRUE", 4))
	{
		*res = 1;
		return 1;
	}
	if (pony_strncmpeff(cfgstr, "false", 5) || pony_strncmpeff(cfgstr, "FALSE", 5))
	{
		*res = 0;
		return 1;
	}
	return 0;
}