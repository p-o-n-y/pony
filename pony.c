//#include "stdafx.h" //for Visual studio -- should remain removed in future versions
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

char pony_strcmptofixed(char* str, char* substr) //not used
{
	int n = 0;
	while (substr[n] != '\0')
	{
		n++;
	}
	return pony_strncmpeff(str, substr, n);
}

// function for locating the beginning of a substring in a string with a given length
// char* str is the general string
// int len is the lenght of the general string
// char* substr is the substring to be located
// int substrlen is the length of the substring
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
int pony_conpartlength(char* str)
{
	return (int)(pony_locatesubstreff(str, "}", 1) - str);
}

// function for "distributing" substrings from configuration to their intended destinations
// char* filter is the string used as an identifier of an inner configurator group (use "" for getting general settings substring)
// char* str is the string of configurator that directly contains the searched substring ( if "a {b: c {d: e} }" is passed only "a" or " c {d: e} " can be obtained, not "c" or " e")
// int len is the length of str
// char** substr is the pointer to be used for setting the pony pointer to the beginning of the searched substring
// int* substrlen is the pointer to be used for setting the length of (*substr) in pony
char pony_locateconfgroup(const char* groupname, char* confstr, const int conflen, char** groupptr, int* grouplen)
{
	int i, j;
	int group_layer;
	char group_found = 0;

	*groupptr = NULL;
	*grouplen = 0;

	// locate configuration substring that is outside of any group
	if (groupname[0] == '\0') {
		for (i = 0; confstr[i] && i < conflen; i++) {
			// skip all non-printable characters, blank spaces and commas between groups
			for (; confstr[i] && (confstr[i] <= ' ' || confstr[i] == ',') && i < conflen; i++); 
			// if no group started at this point
			if (confstr[i] != '{')
				break;
			// if a group started
			else {
				group_layer = 1;
				while (group_layer > 0 && confstr[i] && i < conflen) {
					i++;
					if (confstr[i] == '{')
						group_layer++;
					if (confstr[i] == '}')
						group_layer--;
				}
			}
		}

		// skip all non-printable characters, blank spaces and commas between groups
		for (i++; confstr[i] && (confstr[i] <= ' ' || confstr[i] == ',') && i < conflen; i++);
		// start from this point
		*groupptr = confstr + i;

		// determine the length, counting until the end of the string or when a group started
		for (; (*groupptr)[*grouplen] && (*groupptr)[*grouplen] != '{'; (*grouplen)++);
	}

	// locate configuration substring inside a requested group
	else {
		for (i = 0; confstr[i] && !group_found && i < conflen; i++) {
			// skip all non-printable characters, blank spaces and commas between groups
			for (; confstr[i] && (confstr[i] <= ' ' || confstr[i] == ',') && i < conflen; i++); 
			// if a group started
			if (confstr[i] == '{') {
				group_layer = 1;
				// skip all non-printable characters and blank spaces at the beginning of the group
				for (i++; confstr[i] && (confstr[i] <= ' ') && i < conflen; i++); 

				// check if the group is the one that has been requested
				group_found = 1;
				for (j = 0; confstr[i] && groupname[j] && i < conflen; i++, j++)
					if (confstr[i] != groupname[j]) {
						group_found = 0;
						break;
					}
				// if did not reach the last character of groupname
				if (groupname[j])
					group_found = 0;

				if (group_found)
					// start from this point
					*groupptr = confstr + i;

				// go through the rest of the group
				while (group_layer > 0 && confstr[i] && i < conflen) {
					if (confstr[i] == '{')
						group_layer++;
					if (confstr[i] == '}')
						group_layer--;
					i++;
					// count if inside the requested group, except for the last symbol
					if (group_found && group_layer > 0)
						(*grouplen)++;
				}
			}
		}
	}

	return (*grouplen)>0 ? 1 : 0;
}

// function that replaces all symbols with codes from 1 to 31 to whitespaces (' ')
// char* fromstr is the configuration initial string
// char** tostr is the pointer to the string to which the formatted configuration is copied
void pony_format(char* str)
{
	int i;

	for (i = 0; str[i]; i++)
		if (str[i] < 32)
			str[i] = ' ';
}

// function for initialising pony_dataArrays depending on their sizes
// pony_dataArray *dataarr is the pony_dataArray to be initialised
// int size is the size of the array
void pony_setDASize(pony_dataArray *dataarr, int size)
{
	(*dataarr).arrsize = size;
	(*dataarr).val = (double*)calloc(sizeof(double), size);
}

// function for freeing the memory allocated to pony, should be used only as the last step of terminating pony's activity
void pony_free()
{

	if (pony.bus.imu != NULL)
	{
		free((*pony.bus.imu).f.val);
		free((*pony.bus.imu).q.val);
		free((*pony.bus.imu).w.val);

		free(pony.bus.imu);
	}

	if (pony.bus.gnss != NULL)
	{
		if ((*pony.bus.gnss).gps != NULL)
		{
			free((*pony.bus.gnss).gps);
		}

		if ((*pony.bus.gnss).glo != NULL)
		{
			free((*pony.bus.gnss).glo);
		}

		free(pony.bus.gnss);
	}

	free(pony.conf);
	free(pony.plugins);

}





///
///     Core pony functions for host application
///





// add external functions as pony plugins, plugins are then called in the order of being added
//
// void(*newplugin)(void) - a pointer to a new plugin function you wish to add
//
// return value - TBD
char pony_add_plugin(void(*newplugin)(void))
{
	if (pony.plugins == NULL)
	{
		pony.plugins = (void(**)(void))malloc(sizeof(void(*)(void)));
	}
	else
	{
		pony.plugins = (void(**)(void))realloc(pony.plugins, (pony.pluginsNum + 1) * sizeof(void(*)(void)));
	}
	pony.plugins[pony.pluginsNum] = newplugin;
	pony.pluginsNum++;

	return 1; // пока единица - успешное завершение
}


// function for initialising pony with a user-passed configuration string
// 
// char* config - pony configuration string (see documentation for syntax)
//
// return value - TBD
char pony_init(char* config)
{
	int i;
	int grouplen;
	char* groupptr;

	for (pony.conflength = 0; config[pony.conflength]; pony.conflength++);

	pony.conf = (char *)malloc(sizeof(char) * (pony.conflength + 1));
	for (i = 0; i < pony.conflength; i++)
		pony.conf[i] = config[i];
	pony.conf[pony.conflength] = '\0';
	pony_format(pony.conf);


	pony_locateconfgroup("", pony.conf, pony.conflength, &pony.bus.conf, &pony.bus.conflength);
	

	if (pony_locateconfgroup("imu:", pony.conf, pony.conflength, &groupptr, &grouplen))
	{
		pony.bus.imu = (pony_imu*)calloc(sizeof(pony_imu), 1);
		pony.bus.imu->conf = groupptr;
		pony.bus.imu->conflength = grouplen;

		pony_setDASize(&(pony.bus.imu->f), 3);
		pony_setDASize(&(pony.bus.imu->q), 4);
		pony_setDASize(&(pony.bus.imu->w), 3);
	}

	if (pony_locateconfgroup("gnss:", pony.conf, pony.conflength, &groupptr, &grouplen))
	{
		pony.bus.gnss = (pony_gnss*)calloc(sizeof(pony_gnss), 1);
		pony.bus.gnss->conf = groupptr;
		pony.bus.gnss->conflength = grouplen;

		
		pony_locateconfgroup("", pony.bus.gnss->conf, pony.bus.gnss->conflength, &(pony.bus.gnss->wconf), &(pony.bus.gnss->wconflength));

		if (pony_locateconfgroup("gps:", pony.bus.gnss->conf, pony.bus.gnss->conflength, &groupptr, &grouplen))
		{
			pony.bus.gnss->gps = (pony_gnss_gps*)calloc(sizeof(pony_gnss_gps), 1);
			pony.bus.gnss->gps->conf = groupptr;
			pony.bus.gnss->gps->conflength = grouplen;
		}

		if (pony_locateconfgroup("glo:", pony.bus.gnss->conf, pony.bus.gnss->conflength, &groupptr, &grouplen))
		{
			pony.bus.gnss->glo = (pony_gnss_glo*)calloc(sizeof(pony_gnss_glo), 1);
			pony.bus.gnss->glo->conf = groupptr;
			pony.bus.gnss->glo->conflength = grouplen;
		}
	}
	

	pony.exitplnum = -1;
	
	return 1; // пока единица - успешное завершение
}

// function to be called by host application in a main loop
//
// return value - TBD
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

	return 1; // пока единица - успешное завершение
}





///
///     Standard extraction functions for variables defined by configurator
///





// function for getting the length of the string that would have been obtained using this identifier on pony_extract_string function, functions are not merged as memory is normally allocated in between them
// works only for standard strings, for symbol " that does not mean the end of the string use ""
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// int* res is the pointer to the variable the data should be written to
char pony_extract_string_length(char* confstr, int length, char* identifier, int* res)
{
	int i = 0;
	*res = 0;
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	while (confstr[i] != '\"' || confstr[i + 1] == '\"')
	{
		if (confstr[i] == '\"')
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
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// char** res is the pointer to the variable the data should be written to
char pony_extract_string(char* confstr, int length, char* identifier, char** res)
{
	int i = 0;
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	while (confstr[i] != '\"' || confstr[i + 1] == '\"')
	{
		(*res)[i] = confstr[i];
		if (confstr[i] == '\"' && confstr[i + 1] == '\"')
		{
			i++;
		}
		i++;
	}
	return 1;
}

// function for obtaining symbol by identifier
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// char* res is the pointer to the variable the data should be written to
char pony_extract_char_sym(char* confstr, int length, char* identifier, char* res)
{
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	*res = confstr[0];
	return 1;
}

// function for obtaining number of type char by identifier
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// char* res is the pointer to the variable the data should be written to
char pony_extract_char_num(char* confstr, int length, char* identifier, char* res)
{
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	if ((confstr[0] - '0' > 9 || confstr[0] - '0' < 0) && confstr[0] != '-')
	{
		return 0;
	}
	*res = (char)atoi(confstr);
	return 1;
}

// function for obtaining number of type short by identifier
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// short* res is the pointer to the variable the data should be written to
char pony_extract_short(char* confstr, int length, char* identifier, short* res)
{
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	if ((confstr[0] - '0' > 9 || confstr[0] - '0' < 0) && confstr[0] != '-')
	{
		return 0;
	}
	*res = (short)atoi(confstr);
	return 1;
}

// function for obtaining number of type int by identifier
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// int* res is the pointer to the variable the data should be written to
char pony_extract_int(char* confstr, int length, char* identifier, int* res)
{
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	if ((confstr[0] - '0' > 9 || confstr[0] - '0' < 0) && confstr[0] != '-')
	{
		return 0;
	}
	*res = atoi(confstr);
	return 1;
}

// function for obtaining number of type long by identifier
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// long* res is the pointer to the variable the data should be written to
char pony_extract_long(char* confstr, int length, char* identifier, long* res)
{
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	if ((confstr[0] - '0' > 9 || confstr[0] - '0' < 0) && confstr[0] != '-')
	{
		return 0;
	}
	*res = atol(confstr);
	return 1;
}

// function for obtaining number of type float by identifier
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// float* res is the pointer to the variable the data should be written to
char pony_extract_float(char* confstr, int length, char* identifier, float* res)
{
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	if ((confstr[0] - '0' > 9 || confstr[0] - '0' < 0) && confstr[0] != '.' && confstr[0] != 'e' && confstr[0] != 'E' && confstr[0] != '-')
	{
		return 0;
	}
	*res = (float)atof(confstr);
	return 1;
}

// function for obtaining number of type double by identifier
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// double* res is the pointer to the variable the data should be written to
char pony_extract_double(char* confstr, int length, char* identifier, double* res)
{
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	if ((confstr[0] - '0' > 9 || confstr[0] - '0' < 0) && confstr[0] != '.' && confstr[0] != 'e' && confstr[0] != 'E' && confstr[0] != '-')
	{
		return 0;
	}
	*res = atof(confstr);
	return 1;
}

// function for obtaining boolean by identifier (true - 1, false - 0)
// char* confstr is the configuration string containing the needed data
// int length is the length of the configuration string
// char* identifier is the string preceding the needed data
// char* res is the pointer to the variable the data should be written to
char pony_extract_bool(char* confstr, int length, char* identifier, char* res)
{
	confstr = pony_locatesubstrendn(confstr, length, identifier);
	if (confstr == NULL)
	{
		return 0;
	}
	if (pony_strncmpeff(confstr, "true", 4))
	{
		*res = 1;
		return 1;
	}
	if (pony_strncmpeff(confstr, "false", 5))
	{
		*res = 0;
		return 1;
	}
	return 0;
}