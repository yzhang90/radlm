#include "profiler.h" 
#include <signal.h>
#include <stdlib.h>

static bool running = true;

void handler(int dummy=0) {
  running = false;
}

int main( int argc, char *argv[] ) {
  signal(SIGINT, handler);
  Profiler p;

  while ( running ) {
    p.init();
    usleep( 1e4 + rand() * 1e4 / RAND_MAX  );
    p.add( "first" );
    usleep( 2e4 + rand() * 1e4 / RAND_MAX );
    p.add( "second" );
    usleep( 3e4 + rand() * 1e4 / RAND_MAX );
    p.add( "third" );
    p.print_summary();
  }
}
