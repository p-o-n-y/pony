#include <stdlib.h>

#include "pony.h"

pony_struct pony;

char pony_strncmpeff(char* s1, char* s2, char n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		if (s1[i] != s2[i])
		{
			return 1;
		}
	}
	return 0;
}

char pony_strcmptofixed(char* str, char* substr)
{
	int n = 0;
	while (substr[n] != '\0')
	{
		n++;
	}
	return pony_strncmpeff(str, substr, n);
}

char* pony_locatesubstr(char* str, char* substr)
{
	char* res = str;
	while ((*res) != '\0')
	{
		if (pony_strcmptofixed(res, substr) == 0)
		{
			return res;
		}
		res++;
	}
	return NULL;
}

char* pony_locatesubstrend(char* str, char* substr)
{
	int n;

	char* res = pony_locatesubstr(str, substr);
	if (res == NULL)
	{
		return NULL;
	}
	n = 0;
	while (substr[n] != '\0')
	{
		n++;
	}
	return res + n;
}

char* pony_locatesubstreff(char* str, char* substr)
{
	char* res = str;
	int n = 0;

	while (substr[n] != '\0')
	{
		n++;
	}
	while ((*res) != '\0')
	{
		if (pony_strncmpeff(res, substr, n) == 0)
		{
			return res;
		}
		res++;
	}
	return NULL;
}

char* pony_locatesubstrendeff(char* str, char* substr)
{
	char* res = str;
	int n = 0;
	while (substr[n] != '\0')
	{
		n++;
	}
	while ((*res) != '\0')
	{
		if (pony_strncmpeff(res, substr, n) == 0)
		{
			return res + n;
		}
		res++;
	}
	return NULL;
}

unsigned short pony_conpartlength(char* str)
{
	return pony_locatesubstreff(str, "}") - str;
}

void add_plugin(void(***pluginarray)(), void(*newplugin)(), int* pluginnum)
{
	if (*pluginarray == NULL)
	{
		*pluginarray = (void**)malloc(sizeof(void(*)()));
	}
	else
	{
		*pluginarray = (void**)realloc(*pluginarray, (*pluginnum + 1) * sizeof(void(*)()));
	}
	(*pluginarray)[*pluginnum] = newplugin;
	(*pluginnum)++;
}

void pony_setDASize(pony_dataArray *dataarr, int size)  //чтобы было проще задавать размер массива в конфигураторе
{
	(*dataarr).arrsize = size;
	(*dataarr).val = (double*)calloc(sizeof(double), size);
}

void pony_free()
{
	int i;

	if (pony.bus.imu != NULL)
	{
		free((*pony.bus.imu).f.val);
		free((*pony.bus.imu).q.val);
		free((*pony.bus.imu).w.val);

		free((*pony.bus.imu).conf);

		free(pony.bus.imu);
	}

	if (pony.bus.gnss != NULL)
	{
		free((*pony.bus.gnss).conf);

		free(pony.bus.gnss);
	}

	free(pony.conf);

	for (i = 0; i < pony.pluginsNum; i++)
	{
		free(pony.plugins[i]);
	}

	free(pony.plugins);
}



void pony_add_plugin(void(*newplugin)())
{
	if (pony.plugins == NULL)
	{
		pony.plugins = (void**)malloc(sizeof(void(*)()));
	}
	else
	{
		pony.plugins = (void**)realloc(pony.plugins, (pony.pluginsNum + 1) * sizeof(void(*)()));
	}
	pony.plugins[pony.pluginsNum] = newplugin;
	pony.pluginsNum++;
}



void pony_init(char* config)
{
	pony.conf = config;

	while (pony.conf[pony.conflength] != '\0')
	{
		pony.conflength++;
	}

	{
		char* buffer;

		if ((buffer = pony_locatesubstrendeff(pony.conf, "imu:{")) != NULL)
		{
			pony.bus.imu = (pony_imu*)calloc(sizeof(pony_imu), 1);
			(*pony.bus.imu).conf = buffer;
			(*pony.bus.imu).conflength = pony_conpartlength((*pony.bus.imu).conf);

			pony_setDASize(&(*pony.bus.imu).f, 3);
			pony_setDASize(&(*pony.bus.imu).q, 4);
			pony_setDASize(&(*pony.bus.imu).w, 3);


		}

		if ((buffer = pony_locatesubstrendeff(pony.conf, "gnss:{")) != NULL)
		{
			pony.bus.gnss = (pony_gnss*)calloc(sizeof(pony_gnss), 1);
			(*pony.bus.gnss).conf = buffer;
			(*pony.bus.gnss).conflength = pony_conpartlength((*pony.bus.gnss).conf);
		}
	}

	pony.bus.conf = pony_locatesubstrendeff(pony.conf, "general:{");
	pony.bus.conflength = pony_conpartlength(pony.bus.conf);

	pony.exitplnum = -1;
}

char pony_step()
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
	return (pony.bus.mode >= 0) || (pony.exitplnum > 0);
}

void pony_terminate()
{
	int i;

	pony.bus.mode = -1;

	for (i = 0; i < pony.pluginsNum; i++)
	{
		pony.plugins[i]();
	}

	pony_free();
}