#ifndef PP_CMU_H
#define PP_CMU_H

#include RADL_HEADER

class PP {

public:
  PP();
  int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );

};

#endif //PP_CMU_H
