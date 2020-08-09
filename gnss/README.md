# pony-gnss-plugin-set

## Overview

Plug-ins for [pony](https://github.com/p-o-n-y/pony) core library, that target GNSS aspects:

| Lib name              | Plugin list                                                                             | Short description |
| -------------------   | ----------------------------------------------------------------------                  | ----------------- |
| `pony_gnss_io_rinex`  |                                                                                         | GNSS input/output of RINEX data: |
|                       | [`pony_gnss_io_rinex_v3_read_obs_from_file`](#pony_gnss_io_rinex_v3_read_obs_from_file) | observation data (measurements) from RINEX v3.0x files |
|                       | [`pony_gnss_io_rinex_read_eph_from_file`   ](#pony_gnss_io_rinex_read_eph_from_file)    | navigation data (ephemeris) from RINEX v2.10-3.0x files |
| `pony_gnss_io_ublox`  |                                                                                         | GNSS input/output for u-Blox receiver data: |
|                       | [`pony_gnss_io_ublox_read_file`            ](#pony_gnss_io_ublox_read_file)             | observation and navigation data from u-Blox binary files |
| `pony_gnss_sat`       |                                                                                         | GNSS satellite-related calculations: |
|                       | [`pony_gnss_sat_pos_vel_clock_gps`         ](#pony_gnss_sat_pos_vel_clock_gps)          | position, velocity and clock correction for GPS     satellites |
|                       | [`pony_gnss_sat_pos_vel_clock_glo`         ](#pony_gnss_sat_pos_vel_clock_glo)          | position, velocity and clock correction for GLONASS satellites |
|                       | [`pony_gnss_sat_pos_vel_clock_gal`         ](#pony_gnss_sat_pos_vel_clock_gal)          | position, velocity and clock correction for Galileo satellites |
|                       | [`pony_gnss_sat_pos_vel_clock_bds`         ](#pony_gnss_sat_pos_vel_clock_bds)          | position, velocity and clock correction for BeiDou  satellites |
| `pony_gnss_sol`       |                                                                                         | GNSS navigation solutions: |
|                       | [`pony_gnss_sol_pos_code`                  ](#pony_gnss_sol_pos_code)                   | standalone position and clock error from code pseudoranges |
|                       | [`pony_gnss_sol_check_elevation_mask`      ](#pony_gnss_sol_check_elevation_mask)       | drop measurement validity for satellites below elevation mask |
|                       | [`pony_gnss_sol_select_observables`        ](#pony_gnss_sol_select_observables)         | selects specific observables according to templates |


## Detailed descriptions

### `pony_gnss_io_rinex_v3_read_obs_from_file`

Reads observation data (measurements) from RINEX v3.0x files, including header and data records.
Supports GPS, GLONASS, Galileo and BeiDou systems.
Multi-receiver/multi-antenna capable, with syncronization option.
Does not support header updates after END OF HEADER line.
Only limited number of header labels are processed. 

### `pony_gnss_io_rinex_read_eph_from_file`

Reads navigation data (ephemeris) from RINEX v2.10-3.0x files, including header and data records.
Supports GPS, GLONASS, Galileo and BeiDou systems.
Multi-receiver/multi-antenna capable.
Does not support header updates after END OF HEADER line.
Only limited number of header labels are processed.


### `pony_gnss_io_ublox_read_file`

Reads raw observation and navigation data (measurements and ephemeris) from u-Blox binary files.
Processes messages RXM-RAW/RXM-EPH (GPS L1-only) and RXM-RAWX/RXM-SFRBX
Tested for UBX protocol versions 6, M8T, F9T/F9P.
Supports GPS, GLONASS, Galileo and BeiDou systems.
Multi-receiver/multi-antenna capable, with syncronization option.


### `pony_gnss_sat_pos_vel_clock_gps`

Calculates position, velocity and clock correction for all GPS satellites with valid ephemeris.

### `pony_gnss_sat_pos_vel_clock_glo`

Calculates position, velocity and clock correction for all GLONASS satellites with valid ephemeris.

### `pony_gnss_sat_pos_vel_clock_gal`

Calculates position, velocity and clock correction for all Galileo satellites with valid ephemeris.

### `pony_gnss_sat_pos_vel_clock_bds`

Calculates position, velocity and clock correction for all BeiDou satellites with valid ephemeris.


### `pony_gnss_sol_pos_code`

Computes conventional least-squares standalone position and clock error from code pseudoranges 
for all available receivers/antennas.

### `pony_gnss_sol_check_elevation_mask`

Checks if satellites are below elevation mask and drop their measurement validity flags if the case.
Multi-receiver/multi-system capable. Stores separate value for each receiver in gnss->settings structure.

### `pony_gnss_sol_select_observables`

Sets validity flags for specific observables according to templates in configuration.
Template set 'obs_use' enumerates the exclusive selection of observable types to stay valid.
Template set 'obs_off' lists observable types subject to exclusion from further calculations.
