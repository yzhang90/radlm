
enum state_t {
	STATE_JOY = 0,
	STATE_JOY2CCC = 1,
	STATE_CCC = 2,
	STATE_JOY2PP = 3,
	STATE_PP = 4,
	STATE_ESTOP = 5
};

#include RADL_HEADER

class FSM {

	state_t state_;
	uint8_t ccc_wait_cycle_;
	uint8_t pp_wait_cycle_;

public:
	FSM();
	int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );
	void printState();

};
