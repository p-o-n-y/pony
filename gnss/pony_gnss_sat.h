// Sep-2020
//
/*	pony_gnss_sat
	
	pony plugins for GNSS satellite-related calculations:
*/
void pony_gnss_sat_pos_vel_clock_gps(void); // position, velocity and clock correction for all gps     satellites from ephemeris
void pony_gnss_sat_pos_vel_clock_glo(void); // position, velocity and clock correction for all glonass satellites from ephemeris
void pony_gnss_sat_pos_vel_clock_gal(void); // position, velocity and clock correction for all galileo satellites from ephemeris
void pony_gnss_sat_pos_vel_clock_bds(void); // position, velocity and clock correction for all beidou  satellites from ephemeris
