#include RADL_HEADER

class timestamp {
  public:
    timestamp();
      
    ~timestamp();

    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );

};