// Aug-2022
/*	pony_gnss_sol
	
	pony plugins that provide GNSS navigation solutions:
*/
void pony_gnss_sol_pos_code            (void); // compute standalone position and clock error from code pseudoranges for all available receivers/antennas
void pony_gnss_sol_check_elevation_mask(void); // checks if satellites are below elevation mask and drop validity flags if they are
void pony_gnss_sol_select_observables  (void); // selects observables according to templates in configuration: obs_use and obs_off
