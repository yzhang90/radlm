#include RADL_HEADER
#include "a.h"

class AI {
 private:
  A _node;
 public:
  AI();
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
