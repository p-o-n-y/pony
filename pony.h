// Aug-2019
//
// PONY core declarations

// TIME EPOCH
typedef struct 		// Julian-type time epoch
{
	int Y;			// Year
	int M;			// Month
	int D;			// Day
	int h;			// hour
	int m;			// minute
	double s;		// seconds
} pony_time_epoch;

// SOL
typedef struct			// navigation solution structure
{
	double x[3];		// cartesian coordinates, meters
	char x_valid;		// validity (0/1)
	
	double llh[3];		// geodetic coordinates: longitude (rad), latitude (rad), height (meters)
	char llh_valid;		// validity (0/1)

	double v[3];		// relative-to-Earth velocity vector coordinates in local-level geodetic or cartesian frame, meters per second
	char v_valid;		// validity (0/1)

	double q[4];		// attitude quaternion, relative to local-level or cartesian frame
	char q_valid;		// validity (0/1)

	double L[9];		// attitude matrix for the transition from local-level or cartesian frame, row-wise: L[0] = L_11, L[1] = L_12, ..., L[8] = L[33]
	char L_valid;		// validity (0/1)

	double rpy[3];		// attitude angles relative to local-level frame: roll (rad), pitch (rad), yaw = true heading (rad)
	char rpy_valid;		// validity (0/1)

	double dt;			// system clock error
	double dt_valid;	// validity (0/1)
} pony_sol;




// IMU
typedef struct			// inertial measurement unit
{
	char* cfg;			// pointer to IMU configuration string
	int cfglength;		// IMU configuration string length

	double w[3];		// up to 3 gyroscope measurements
	char w_valid;		// validity (0/1)

	double f[3];		// up to accelerometer measurements
	char f_valid;		// validity (0/1)

	pony_sol sol;		// inertial solution
} pony_imu;




// GNSS
	// SAT
typedef struct 				// GNSS satellite data
{
	double *eph;			// array of satellite ephemeris as defined by RINEX format
	char eph_valid;			// validity (0/1)

	double Deltatsv;		// SV PRN code phase time offset (seconds), SV slock correction term as in Section 20.3.3.3.3.1 of IS-GPS-200J (22 May 2018) p. 96

	double t_em;			// time of signal emission
	char t_em_valid;		// validity (0/1)
	double x[3];			// satellite coordinates
	char x_valid;			// validity (0/1)
	double v[3];			// satellite velocity vector
	char v_valid;			// validity (0/1)
	
	double sinEl;			// sine of satellite elevation angle
	char sinEl_valid;		// validity (0/1)

	double *obs;			// satellite observables array, defined at runtime
	char *obs_valid;		// satellite observables validity array (0/1)
} pony_gnss_sat;

	// GPS const
typedef struct		// GPS system constants
{
	double 
		pi,			// pi as in GPS interface specs
		c,			// speed of light, m/s, 
		mu,			// Earth grav constant as in GPS interface specs, m^3/s^2
		u,			// Earth rotation rate as in GPS interface specs, rad/s
		a,			// Earth ellipsoid semi-major axis, m
		e2,			// Earth ellipsoid first eccentricity squared
		F,			// relativistic correction constant
		sec_in_w,	// seconds in a week
		sec_in_d,	// seconds in a day
		F1, L1,		// nominal frequency and wavelength for L1 signal
		F2, L2;		// nominal frequency and wavelength for L2 signal
} pony_gps_const;

	// GPS
typedef struct				// GPS constellation data
{
	char* cfg;				// GPS configuration string
	int cfglength;			// configuration string length

	int max_sat_count;		// maximum supported number of satellites
	int max_eph_count;		// maximum supported number of ephemeris

	pony_gnss_sat *sat;		// GPS satellites
	char **obs_types;		// observation types according to RINEX: C1C, etc.; an array of 3-character null-terminated strings in the same order as satellites
	int obs_count;			// number of observation types

	double iono_a[4];		// ionospheric model parameters from GPS almanac
	double iono_b[4];		
	char iono_valid;		// validity (0/1)

	double clock_corr[3];	// clock correction parameters from GPS almanac
	char clock_corr_valid;	// validity (0/1)
} pony_gnss_gps;

	// GLONASS
typedef struct				// GLONASS constellation data (not supported yet)
{
	char* cfg;				// GLONASS configuration string
	int cfglength;			// configuration string length

	int max_sat_count;		// maximum supported number of satellites
	int max_eph_count;		// maximum supported number of ephemeris

	pony_gnss_sat *sat;		// GLONASS satellites
	char **obs_types;		// observation types according to RINEX: C1C, etc.; an array of 3-character null-terminated strings in the same order as satellites
	int obs_count;			// number of observation types

} pony_gnss_glo;

	// SETTINGS
typedef struct // GNSS operation settings
{
	double sinEl_mask;			// elevation angle mask, sine of
	double code_sigma;			// pseudorange measurement rmsdev (sigma), meters
	double phase_sigma;			// carrier phase measurement rmsdev (sigma), cycles
} pony_gnss_settings;

	// GNSS
typedef struct						// global navigation satellite systems data
{
	char* cfg;						// full GNSS configuration string pointer
	int cfglength;					// full GNSS configuration string length

	char* cfg_settings;				// pointer to a part of GNSS configuration string common to all systems
	int settings_length;			// length of the part of GNSS configuration string common to all systems

	pony_gps_const gps_const;		// GPS constants
	pony_gnss_settings settings;	// GNSS operation settings

	pony_gnss_gps* gps;				// GPS constellation data pointer
	pony_gnss_glo* glo;				// GLONASS constellation data pointer

	pony_time_epoch epoch;			// current GNSS time epoch

	pony_sol sol;					// current GNSS solution
	int obs_count;					// total observations used in solution
} pony_gnss;





// BUS
	// CORE
typedef struct					// main core structure and instance
{
	void(**plugins)(void);				// plugin array pointer
	int plugin_count;					// number of plugins
	int exit_plugin_id;					// index of a plugin that initiated termination
} pony_core;

#define pony_bus_version 1		// current bus version
typedef struct					// bus data to be used in host application
{
	int ver;							// bus version to be used at runtime	

	// main functions to be used in host app
	char(*add_plugin)( void(*)(void) );	// add plugin to the plugin execution list,		input: pointer to plugin function,				output: OK/not OK (1/0)
	char(*init)(char *);				// initialize the bus, except for core,			input: configuration string (see description),	output: OK/not OK (1/0)
	char(*step)(void);					// step through the plugin execution list,														output: OK/not OK (1/0)
	char(*terminate)(void);				// terminate operation,																			output: OK/not OK (1/0)
	pony_core core;						// core instances

	char* cfg;							// full configuration string
	int cfglength;						// full configuration string length

	char* cfg_settings;					// pointer to a part of the configuration string common to all subsystems
	int settings_length;				// length of the part of the configuration string common to all subsystems

	pony_imu* imu;						// inertial measurement unit data pointer

	pony_gnss* gnss;					// global navigation satellite system data pointer
	int gnss_count;						// number of gnss instances

	double t;							// system time
	int mode;							// operation mode: 0 - init, <0 termination, >0 normal operation
	pony_sol sol;						// navigation solution
} pony_bus;

extern pony_bus pony;






// linear algebra functions
	// conventional operations
double pony_linal_dot(double *u, double *v, const int m); // dot product

	// routines for m x m upper-triangular matrices U lined up in a single-dimension array u
void pony_linal_u_ij2k(int *k, const int i, const int j, const int m);	// index conversion: (i,j) -> k
void pony_linal_u_k2ij(int *i, int *j, const int k, const int m);		// index conversion: k -> (i,j)

void pony_linal_u_mul_v(double *res, double *u, double *v, const int m);	// matrix multiplication by vector: res = U*v
void pony_linal_uT_mul_v(double *res, double *u, double *v, const int m);	// transposed matrix multiplication by vector: res = U^T*v

void pony_linal_chol(double *S, double *P, const int m); // Cholesky upper-triangular factorization P = S*S^T, where P is symmetric positive-definite matrix

	// square root Kalman filtering
double pony_linal_kalman_update(double *x, double *S, double *K, double z, double *h, double sigma, const int m);