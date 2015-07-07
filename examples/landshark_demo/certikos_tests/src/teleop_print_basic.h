
#include <iostream>

class StandalonePrint {
public:
	void step(const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags) {
		std::cout << "[Printer Joystick] " << "linear.x: " << in->base->linear.x << " flags: " << (int)in_flags->base << std::endl;
	}
};
