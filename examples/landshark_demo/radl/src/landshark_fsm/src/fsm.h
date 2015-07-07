

enum state_t {
	STATE_JOY = 0,
	STATE_JOY2CCC = 1,
	STATE_CCC = 2,
	STATE_JOY2PP = 3,
	STATE_PP = 4,
	STATE_ESTOP = 5,
	STATE_2ESTOP = 6,
	STATE_ESTOP2JOY = 7,
	STATE_CCC2JOY = 8,
	STATE_PP2JOY = 9
};

#include RADL_HEADER

class FSM {

	state_t state_;
	// max number of wait cycles N is determined by the formula:
	// N * minT(P) > D + maxT(S)
	uint8_t ccc_wait_cycle_;
	uint8_t pp_wait_cycle_;
	uint8_t estop_wait_cycle_;
	uint8_t estop2joy_wait_cycle_;
	uint8_t ccc2joy_wait_cycle_;
	uint8_t pp2joy_wait_cycle_;

public:
	FSM();
	int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );
	//inline void printState(cout << state_ << endl);

};
