// Sep-2020
//
/*	pony_ins_alignment 
	
	pony plugins for ins initial alignment (initial attitude matrix determination):
*/
void pony_ins_alignment_static      (void); // conventional sensor output averaging on a static base
void pony_ins_alignment_rotating    (void); // gravity vector approximation in inertial reference
void pony_ins_alignment_rotating_rpy(void); // gravity vector approximation in inertial reference, but matrix is computed via roll, pitch and yaw=true heading, if defined
