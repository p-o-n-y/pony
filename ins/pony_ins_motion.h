// Feb-2025
/*	pony_ins_motion

	pony plugins for ins position and velocity algorithms
*/
void pony_ins_motion_euler           (void); // velocity and position using first-order Euler integration
void pony_ins_motion_sculling        (void); // planned for future development
void pony_ins_motion_vertical_damping(void); // vertical error buildup damping
void pony_ins_motion_size_effect     (void); // compensation for accelerometer cluster size effect (their spatial separation)
