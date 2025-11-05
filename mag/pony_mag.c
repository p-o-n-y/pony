// Nov-2025
/*	pony_mag

	pony plugins for magnetic field calculations:

	- pony_mag_earth_igrf
		Calculates magnetic field of the Earth according to International Geomagnetic Reference Field
		model which is a semi-normalized spherical harmonic expansion.
		Currently no interpolation in time is supported, so only one set of coefficients is used.
		Produces Eastern, Northern and Upward components in those units, in which the model parameters
		are given in the configuration string (typically, but not limited to nano-Teslas).
		Allows for 2 update rates: fast, updating the field at each step, and slow (less computations at each step).

*/


#include <stdlib.h>
#include <math.h>

#include "../pony.h"


// pony bus version check
#define PONY_MAG_BUS_VERSION_REQUIRED 19
#if PONY_BUS_VERSION < PONY_MAG_BUS_VERSION_REQUIRED
	#error "pony bus version check failed, consider fetching the latest version"
#endif


// service functions
	// calculations
char   pony_mag_precalc_factors(float* sqr, float* f1, float* f2,  const size_t deg);
	// parsing
size_t pony_mag_parse_model(size_t* n,  float** G, float** H,  char* cfg, const size_t len,  const char* deg_token, const char* G_token, const char* H_token);
size_t pony_mag_parse_array_size(char* str, const size_t len);
size_t pony_mag_parse_array_values_igrf(float* M, const size_t deg, const size_t deg0,  char *cfg, const size_t len);
	// memory handling
void pony_mag_free_null(void** ptr);


/* pony_mag_earth_igrf - pony plugin

	Calculates magnetic field of the Earth according to International Geomagnetic Reference Field
	model which is a semi-normalized spherical harmonic expansion.
	For reference, see:
	- IAGA Division I Study Group. International Geomagnetic Reference Field 1975. 
	  in Journal of Geomagnetism Geoelectricity, vol. 27 (1975):437-439. DOI: 10.5636/jgg.27.437
	- Alken, P., Thebault, E., Beggan, C.D. et al. International Geomagnetic Reference Field: the 13th generation,
	  in Earth Planets Space 73, 49 (2021). DOI: 10.1186/s40623-020-01288-x
	- www.ncei.noaa.gov/products/international-geomagnetic-reference-field
	
	Currently no interpolation in time is supported, so only one set of coefficients is used.
	Produces Eastern, Northern and Upward components in those units, in which the model parameters
	are given in the configuration string (typically, but not limited to nano-Teslas).
	Allows for 2 update rates: fast, updating the field at each step, and slow (less computations at each step).
	
	Translated and adapted from SHTools project Fortran 95 subroutine PlmSchmidt_d1 (github.com/SHTOOLS)
	Mark A. Wieczorek and Matthias Meschede. SHTools: Tools for working with spherical harmonics,
	in Geochemistry, Geophysics, Geosystems, vol. 19 (2018), 2574-2592, DOI: 10.1029/2018GC007529
	- added slow update rate mode
	- added division by zero at poles check

	description:

		Takes current position from either pony->sol.llh (if valid),
		or from pony->mag->sol.llh array. Longitude and latitude in radians, height in meters
		above the Earth ellipsoid.


	uses:
		pony->sol->llh
		pony->sol->llh_valid
		pony->mag->sol.llh
		pony->mag->sol.llh_valid

	changes:
		pony->mag->e
		pony->mag->e_valid

	cfg parameters:
		{mag: igrf_degree} - expansion degree of spherical harmonics, so their maximum degree and order will be igrf_degree
			type :   integer
			range:   1 to 8192 (actual maximum value is defined by coefficient array sizes, see notes)
			default: defined by {mag: igrf_g}, {mag: igrf_h} parameters, or 4 if not specified
			example: {mag: igrf_degree = 8}
			notes:   - if sets of Gauss coefficients have more coefficients than specified by this parameter, extra values remain ignored
				     - zero degree inhibits magnetic field model calculation
		{mag: igrf_g} - set of spherical harmonic Gauss coefficients for cosine of longitude in the expansion of magnetic field potential
		{mag: igrf_h} - set of spherical harmonic Gauss coefficients for   sine of longitude in the expansion of magnetic field potential
			type :   floating point array
			range:   -1E+37 to +1E+37
			default: 4-th degree set of IGRF-14 at epoch 2025.0, in units of nano-Tesla, row-wise (degree), order in columns
			         (up to 400 nT terms cutoff, max magnetic deviation 15 deg at 15-deg geodetic grid)
			example: {mag: igrf_g = [     0
			                         -29404.8 -1450.9
			                          -2499.6  2982.0  1677.0
			                           1363.2 -2381.2  1236.2   525.7]
			               igrf_h = [     0
			                              0    4652.5
			                              0   -2991.6  -734.6
			                              0     -82.1   241.9  -543.4]}
			notes:   - both arrays must be present and have the same size, corresponding to the degree given in {mag: igrf_degree} parameter, or higher,
			         - the degree will be reduced appropriately to match array sizes, if need be
					 - failure to locate either of the two, or if their size mismatch, results in receding to the default model
		{mag: igrf_rate} - selects model update rate, at which the field components are updated,
		                   f for fast update at each time step, more computationally expensive, slower total calculation
		                   s for slow update, one order at each time step (resulting update rate depends on the degree used), faster calculation
			type :   single character
			range:   f or s (case-sensitive)
			default: f
			example: {mag: igrf_rate = s}
			notes:   self-explanatory words like "fast" or "slow" might be used, unless not interfering with other parameters in configuration
*/
void pony_mag_earth_igrf(void)
{
	enum {E, N, U, dim,  deg_def = 4, n_def = ((deg_def+1)*(deg_def+2))/2, llh_uses = 2};
	enum {
		slon,
		clon,
		trigs,
		slat,
		prefactors,
		Pk0,
		Pmm,
		harmonics,
		assign,
		stages };

	const char
		 deg_token[]  = "igrf_degree", // expansion degree parameter name in configuration
		   G_token[]  = "igrf_g",      // Gauss coefficients for cosine of longitude
		   H_token[]  = "igrf_h",      // Gauss coefficients for   sine of longitude
		rate_token[]  = "igrf_rate",   // calculation rate parameter name in configuration
		rate_chars[2] = {'f', 's'};    // allowed characters with 0-th element as default
	const size_t
		deg_max       = 8192;         // 8192, to prevent accidental excess memory allocation
	const float
		scalef       =  1.0e-30f,
		a            = 6371.2e3f,      // mean Earth's radius used in the model
		e2           = 6.69438e-3f;    // Earth's ellipsoid eccentricity

	static float // static pointer for type compatibility with actually used pointers (see p. 3.3.16.1 of the ANSI C standard, Simple assignment)
		// default Gauss coefficients for cosine of longitude in the magnetic field potential expansion over Schmidt semi-normalized spherical harmonics, IGRF-14(2025.0)
		G_def[n_def] = {     0   ,
		                -29350.0f,  -1410.3f,
						 -2556.2f,  +2950.9f,  +1648.7f,
						 +1360.9f,  -2404.2f,  +1243.8f,   +453.4f,
						  +894.7f,   +799.6f,    +55.8f,   -281.1f,    +12.0f},
		// default Gauss coefficients for   sine of longitude in the magnetic field potential expansion over Schmidt semi-normalized spherical harmonics, IGRF-14(2025.0)
		H_def[n_def] = {     0   ,
		                     0   ,  +4545.5f,
						     0   ,  -3133.6f,   -814.2f,
		                     0   ,    -56.9f,   +237.6f,   -549.6f,
		                     0   ,   +278.6f,   -134.0f,   +212.0f,   -375.4f};
	static float      // use single precision to save memory and speed on embedded systems/single-precision floating-point arithmetic processors
		*G    = NULL, // actual Gauss coefficients for cosine of longitude used in calculations
		*H    = NULL, // actual Gauss coefficients for   sine of longitude used in calculations
		*sqr  = NULL, // square roots for spherical harmonics
		*f1   = NULL, // multiplicative factors used in recursion
		*f2   = NULL,
		* P   = NULL, // spherical harmonics
		*dP   = NULL, // derivatives of spherical harmonics
		*sinm = NULL, // trig functions for longitude at different orders
		*cosm = NULL,
		*a_r  = NULL, // pre-factors for spherical harmonics
		 e  [dim] = {0,0,0}, // calculated field gradient
		 llh[dim] = {0,0,0}, // current coordinates
		 z, u, u2, inv_u2, r,
		 pm1, pm2, plm,
		 pmm, rescalem;
	static size_t
		 deg          = deg_def, // expansion degree and order
		   n          =   n_def, // coefficient array sizes
		llh_countdown = 0,       // validity countdown
		stage         = 0,       // calculation stage for slow mode
		m = 0, l = 0, k0 = 0;
	static char
		rate      = 0,  // model update rate symbol
		fast      = 1;

	char   *cfg_ptr;        // pointer to a substring in configuration
	size_t
		i, len,             // indices
		k, l1, m2;


	// validate magnetic subsystem
	if (pony->mag == NULL)
		return;

	if (pony->mode == 0) {		// init

		// parse update rate from configuration
		len = pony->mag->cfglength;
		cfg_ptr = pony_locate_token(rate_token, pony->mag->cfg, len, '=');
		if (cfg_ptr != NULL) {
			for (i = 0; i < len && cfg_ptr[i] != 0 && cfg_ptr[i] <= ' '; i++); // skip blank space
			rate = cfg_ptr[i];      // assign first character
			for (i = 0; i < 2; i++) // check among allowed values
				if (rate == rate_chars[i])
					break;
		}
		if (cfg_ptr == NULL || i >= 2)             // if not found, reset to default
			rate = rate_chars[0];
		fast = (rate == rate_chars[0]); // 1 for fast mode, 0 for slow
		// determine actual model degree and its coefficients
		if (!(deg = pony_mag_parse_model( // try parsing from configuration
			&n, &G, &H,
			pony->mag->cfg, pony->mag->cfglength,
			deg_token, G_token, H_token)) || deg > deg_max) {
			deg = deg_def;
			  n =   n_def;
			  G =   G_def;
			  H =   H_def;
		}
		// allocate memory
		sqr  = (float*)calloc(2*deg+1, sizeof(float));
		f1   = (float*)calloc(n      , sizeof(float));
		f2   = (float*)calloc(n      , sizeof(float));
		 P   = (float*)calloc(n      , sizeof(float));
		dP   = (float*)calloc(n      , sizeof(float));
		sinm = (float*)calloc(  deg+1, sizeof(float));
		cosm = (float*)calloc(  deg+1, sizeof(float));
		a_r  = (float*)calloc(  deg+1, sizeof(float));
		if (0
			|| sqr == NULL || f1 == NULL || f2 == NULL || P == NULL || dP == NULL || sinm == NULL || cosm == NULL || a_r == NULL
			|| !pony_mag_precalc_factors(sqr, f1, f2,  deg)) {
			pony->mode = -1; // error allocating memory, unable to proceed with calculations or recover
			return;
		}
		// constants
		sinm[0] = 0;
		cosm[0] = 1;
		// validity flags down
		pony->mag->e_valid = 0;
		llh_countdown      = 0;
		// calculation stage for slow mode
		if (!fast)
			stage = 0;
		m = 0;
		l = 0;

	}

	else if (pony->mode < 0) {	// terminate

		// free allocated memory
		if (G != G_def)
			pony_mag_free_null((void**)(&G));
		if (H != H_def)
			pony_mag_free_null((void**)(&H));
		pony_mag_free_null((void**)(&sqr ));
		pony_mag_free_null((void**)(&f1  ));
		pony_mag_free_null((void**)(&f2  ));
		pony_mag_free_null((void**)(& P  ));
		pony_mag_free_null((void**)(&dP  ));
		pony_mag_free_null((void**)(&sinm));
		pony_mag_free_null((void**)(&cosm));
		pony_mag_free_null((void**)(&a_r ));

	}

	else						// main cycle
	{
		if (deg == 0) // no model provided
			return;
		// drop validity
		if (fast || stage == assign)
			pony->mag->e_valid = 0;
		// renew coordinates
			if (0); // for code alignment, removed by compiler on optimization
		else if (pony->     sol.llh_valid) {for (i = 0; i < dim; i++) llh[i] = (float)pony->     sol.llh[i]; llh_countdown = llh_uses;}
		else if (pony->mag->sol.llh_valid) {for (i = 0; i < dim; i++) llh[i] = (float)pony->mag->sol.llh[i]; llh_countdown = llh_uses;}
		if (llh_countdown) {
			// pre-calculate trigs
			if (fast || stage == slon)
				sinm[1] = (float)sin(llh[E]);
			if (fast || stage == clon)
				cosm[1] = (float)cos(llh[E]);
			if (fast || stage == trigs)
				for (i = 2; i <= deg; i++) {
					sinm[i] = 2*sinm[i-1]*cosm[1] - sinm[i-2];
					cosm[i] = 2*cosm[i-1]*cosm[1] - cosm[i-2];
				}
			// calculate pre-factors
			if (fast || stage == slat)
				z = (float)sin(llh[N]);
			if (fast || stage == prefactors) {
				u2 = (1-z)*(1+z);    // cosine squared
				if (u2 > 0)
					inv_u2 = 1.0f/u2;
				else
					llh_countdown = 0; // at poles spherical expansion is undefined
				u = (float)sqrt(u2); // cosine absolute value
				r = a*(1 + e2*(0.16f - z*z/2)) + llh[U];  // approximate distance from the Earth's center for a = 6371.2 km
				a_r[0] = a/r;
				for (i = 1; i <= deg; i++)
					a_r[i] = a_r[i-1]*a/r;
			}
			if (fast && !llh_countdown) // fast mode requires valid coordinates at each time step to provide valid instantaneous magnetic model
				return;
			// calculate spherical harmonics
			if (fast || stage == Pk0) {
				// calculate P(1,0), not scaled
				pm2   = 1;
				P [0] = 1;
				dP[0] = 0;
				// calculate P(2,0), not scaled
				pm1   = z;
				P[1] = pm1;
				dP[1] = 1;
				for (k = 1, l1 = 2; l1 <= deg; l1++) {
					k = k + l1;
					plm   = f1[k]*z*pm1 - f2[k]*pm2;
					P[k] = plm;
					dP[k] = l1*(pm1 - z*plm)*inv_u2;
					pm2 = pm1;
					pm1 = plm;
				}
			}
			if (fast || stage == Pmm) {
				// calculate P(m,m), P(m+1,m), and P(l,m)
				if (m == 0) {
					pmm = sqr[1]*scalef;
					rescalem = 1/scalef;
					m = 1;
					k0 = 0;
				}
				do {
					rescalem = rescalem*u;
					// calculate P(m,m)
					k0 = k0 + m + 1;
					m2 = 2*m;
					pmm    = pmm*sqr[m2]/sqr[m2-1];
					P[k0] = pmm*rescalem/sqr[m2];
					dP[k0] = -1.0f*m*z*P[k0]*inv_u2;
					pm2 = pmm/sqr[m2];
					// calculate P(m+1,m)
					k = k0 + m + 1;
					pm1   = z*sqr[m2]*pm2;
					P[k] = pm1*rescalem;
					dP[k] = (P[k-m-1]*sqr[m2] - z*(m+1)*P[k])*inv_u2;
					// calculate P(l,m)
					for (l1 = m + 2; l1 <= deg; l1++) {
						k = k + l1;
						plm   = z*f1[k]*pm1 - f2[k]*pm2;
						P[k] = plm*rescalem;
						dP[k] = (sqr[l1+m-1]*sqr[l1-m-1]*P[k-l1] - l1*z*P[k])*inv_u2;
						pm2 = pm1;
						pm1 = plm;
					}
					m++;
				} while (fast && m < deg);
				if (m < deg)
					return; // no stage advancement, continue cycling over Pmm
				// calculate P(N,N)
				rescalem = rescalem*u;
				k0 = k0 + m + 1;
				pmm = pmm/sqr[2*deg-1];
				P[k0] = pmm*rescalem;
				dP[k0] = -1.0f*deg*z*P[k0]*inv_u2;
				m = 0; // reset cycling over Pmm
			}
			if (fast || stage == harmonics) {
				if (l == 0) {
					// multiply derivatives to use in gradient
					for (i = 0; i < n; i++)
						dP[i] *= u;
					// initialize field gradient
					for (i = 0; i < dim; i++)
						e[i] = (i == U) ? -G[0] : 0;
					l = 1;
				}
				// add all harmonics
				do {
					l1 = l + 1;
					k = ((l+1)*l)/2;
					e[U] += -a_r[l-1]*l1*G[k]* P[k];
					e[N] +=  a_r[l-1]   *G[k]*dP[k];
					for (m2 = 1; m2 <= l; m2++) {
						k++;
						e[U] += -a_r[l-1]*l1* P[k]*(         G[k]*cosm[m2] +    H[k]*sinm[m2]);
						e[N] +=  a_r[l-1]*   dP[k]*(         G[k]*cosm[m2] +    H[k]*sinm[m2]);
						e[E] +=  a_r[l-1]*    P[k]*(-1.0f*m2*G[k]*sinm[m2] + m2*H[k]*cosm[m2]);
					}
					l++;
				} while (fast && l <= deg);
				if (l <= deg)
					return; // no stage advancement, continue cycling over harmonics
				l = 0; // reset cycling over harmonics
			}
			if (fast || stage == assign) {
				// invert direction and scale
				e[E] *= (u > 0 ? 1/u : 0);
				for (i = 0; i < dim; i++)
					pony->mag->e[i] = -e[i]*a_r[0]*a_r[0];
				// validity
				pony->mag->e_valid = (llh_countdown && inv_u2 > 0);
				if (llh_countdown > 0)
					llh_countdown--;
			}
		}
		// advance to next stage
		if (!fast)
			stage = (stage + 1)%stages;
	}

}



// service functions
	// calculations
char pony_mag_precalc_factors(float* sqr, float* f1, float* f2,  const size_t deg)
{
	size_t k, l, m;


	// validate
	if (sqr == NULL || f1 == NULL || f2 == NULL)
		return 0;
	// pre-calculate square roots
	for (k = 0; k < 2*deg+1; k++)
		sqr[k] = (float)sqrt(k + 1.0f);
	// pre-calculate multiplicative factors used in recursion
	for (k = 0; k < 3; k++)
		f1[k] = 0, f2[k] = 0;
    for (l = 2; l <= deg; l++, k++) {
		f1[k] = (2*l - 1.0f)/l;
		f2[k] = (  l - 1.0f)/l;
		for (m = 1; m <= l-2; m++) {
			k++;
			f1[k] = (2*l - 1.0f)/sqr[l+m-1]/sqr[l-m-1];
			f2[k] = sqr[l-m-2]*sqr[l+m-2]/sqr[l+m-1]/sqr[l-m-1];
		}
		k += 2;
	}
	return 1;
}
	// parsing
size_t pony_mag_parse_model(
	size_t* n,  float** G, float** H,
	char* cfg, const size_t len,
	const char* deg_token, const char* G_token, const char* H_token)
{
	char *ptrG, *ptrH, *ptr;
	size_t n0, deg, deg0;


	// validate input
	if (0
		|| n == NULL || G == NULL || H == NULL
		|| cfg == NULL
		|| deg_token == NULL || G_token == NULL || H_token == NULL)
		return 0;

	// locate Gauss coefficients, and count them
	ptrG = pony_locate_token(G_token, cfg, len, '=');
	if (ptrG == NULL)
		return 0;
	else
		n0 = pony_mag_parse_array_size(ptrG, len-(ptrG-cfg));
	ptrH = pony_locate_token(H_token, cfg, len, '=');
	if (ptrH == NULL || n0 != pony_mag_parse_array_size(ptrH, len-(ptrH-cfg)))
		return 0;
	// determine the order, and check if number of coefficients fits any
	deg0 = (size_t)floor(sqrt(0.25 + 2*n0)); // follows from n = deg1*(deg1+1)/2, where deg1 = deg+1
	if (deg0*(deg0 + 1) != 2*n0)     // wrong number of coefficients
		return 0;
	else
		deg0--;
	// parse desired model degree, if present
	ptr = pony_locate_token(deg_token, cfg, len, '=');
	if (ptr == NULL || (deg = strtoul(ptr, NULL, 10)) == 0 || deg > deg0) // strtoul assigns zero on failure
		deg = deg0;
	// recalculate number of coefficients and (re-)allocate memory, with zero-assignment
	n0 = ((deg+1)*(deg+2))/2;
	*n = 0;
	if (*G != NULL)
		free(*G);
	if (*H != NULL)
		free(*H);
	*G = (float*)calloc(n0, sizeof(float));
	*H = (float*)calloc(n0, sizeof(float));
	if (*G == NULL || *H == NULL)
		return 0;
	// parse values with regards to difference in stored and desired model degree, if any
	if (0
		|| n0 != pony_mag_parse_array_values_igrf(*G, deg, deg0,  ptrG, len-(ptrG-cfg)) // wrong number of coefficients, leave array unchanged for debug
		|| n0 != pony_mag_parse_array_values_igrf(*H, deg, deg0,  ptrH, len-(ptrH-cfg)) // wrong number of coefficients, leave array unchanged for debug
		)
		return 0;
	*n = n0;

	return deg;

}

size_t pony_mag_parse_array_size(char* str, const size_t len)
{
	const char
		bro = '[',
		brc = ']',
		spc = ' ';

	char *ptr, *end_ptr, *max_ptr;
	size_t n;


	// go for opening bracket
	max_ptr = str + len;
	ptr = str;
	for (; ptr < max_ptr && *ptr != 0 && *ptr <= spc; ptr++); // skip blank space
	if (ptr >= max_ptr || *ptr != bro)                        // opening bracket not found
		return 0;
	// try parsing values, start after the opening bracket
	end_ptr = ptr + 1;
	n = 0;
	do
		strtod(ptr = end_ptr, &end_ptr);
	while (ptr < max_ptr && *ptr != 0 && end_ptr != ptr && ++n);
	// go for closing bracket
	for (; ptr < max_ptr && *ptr != 0 && *ptr <= spc; ptr++); // skip blank space
	if (ptr >= max_ptr || *ptr != brc)                        // closing bracket not found
		return 0;

	return n;
}

size_t pony_mag_parse_array_values_igrf(float* M, const size_t deg, const size_t deg0,  char *cfg, const size_t len)
{
	const char
		bro = '[',
		brc = ']',
		spc = ' ';

	char *ptr, *end_ptr, *max_ptr;
	size_t n, l, m;


	// validate
	if (M == NULL || cfg == NULL || deg > deg0)
		return 0;
	// go for opening bracket
	max_ptr = cfg + len;
	ptr = cfg;
	for (; ptr < max_ptr && *ptr != 0 && *ptr <= spc; ptr++); // skip blank space
	if (ptr >= max_ptr || *ptr != bro)                        // opening bracket not found
		return 0;
	// try parsing values, start after the opening bracket
	end_ptr = ptr + 1;
	for (n = 0, l = 0, m = 0; ptr < max_ptr && *ptr != 0 && end_ptr != ptr && l <= deg; n++) {
		M[n] = (float)strtod(ptr = end_ptr, &end_ptr);
		if (end_ptr == ptr)
			break;
		m++;
		if (m > l) {
			l++;
			m = 0;
		}
	}
	// go for closing bracket
	for (; ptr < max_ptr && *ptr != 0 && *ptr != brc; ptr++); // skip everything up to the bracket
	if (ptr >= max_ptr || *ptr == 0)                          // closing bracket not found
		return 0;

	return n;
}

	// memory handling
void pony_mag_free_null(void** ptr)
{
	if (ptr == NULL || *ptr == NULL)
		return;

	free(*ptr);
	*ptr = NULL;
}
