#include RADL_HEADER
#include "c.h"

class TwoNodesC_Wrapper {
  private:
    C _node;
  public:
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
