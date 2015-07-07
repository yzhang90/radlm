#include "HokuyoDriver.h"

#include RADL_HEADER

class RADL_NODE_NAME {
  public:
    RADL_NODE_NAME();
    ~RADL_NODE_NAME();
    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );

  private: 
    HokuyoDriver driver_;
    size_t array_size_;
};
