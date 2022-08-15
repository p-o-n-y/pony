// Aug-2022
/*	pony_gnss_sat
	
	pony plugins for GNSS satellite-related calculations:
*/
void pony_gnss_sat_pos_vel_clock_gps(void); // position, velocity and clock correction for all GPS             satellites from ephemeris
void pony_gnss_sat_pos_vel_clock_glo(void); // position, velocity and clock correction for all GLONASS         satellites from ephemeris
void pony_gnss_sat_pos_vel_clock_gal(void); // position, velocity and clock correction for all Galileo         satellites from ephemeris
void pony_gnss_sat_pos_vel_clock_bds(void); // position, velocity and clock correction for all BeiDou MEO/IGSO satellites from ephemeris
