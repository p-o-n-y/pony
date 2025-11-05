# pony-mag-plugin-set

## Overview

Plug-ins for [pony](https://github.com/p-o-n-y/pony) core library, that target magnetic navigation aspects:

| Lib name              | Plugin list                                   | Short description                            |
| -------------------   | --------------------------------------------- | -------------------------------------------- |
| `pony_mag`            |                                               | Magnetic field calculations:                 |
|                       | [`pony_mag_earth_igrf`](#pony_mag_earth_igrf) | Earth magnetic field according to IGRF model |


## Detailed descriptions

### `pony_mag_earth_igrf`

Calculates magnetic field of the Earth according to International Geomagnetic Reference Field model
which is a semi-normalized spherical harmonic expansion.  
Currently no interpolation in time is supported, so only one set of coefficients is used.  
Produces Eastern, Northern and Upward components in those units, in which the model parameters are given
in the configuration string (typically, but not limited to nano-Teslas).  
Allows for 2 update rates: fast, updating at each step, and slow (less computations at each step).  
For reference, see:  
  - IAGA Division I Study Group. International Geomagnetic Reference Field 1975.  
    in Journal of Geomagnetism Geoelectricity, vol. 27 (1975):437–439. DOI: 10.5636/jgg.27.437  
  - Alken, P., Thébault, E., Beggan, C.D. et al. International Geomagnetic Reference Field: the 13th generation,  
    in Earth Planets Space 73, 49 (2021). DOI: 10.1186/s40623-020-01288-x  
  - www.ncei.noaa.gov/products/international-geomagnetic-reference-field  
