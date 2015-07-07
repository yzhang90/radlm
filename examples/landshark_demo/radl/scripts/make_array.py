#!/usr/bin/env python 

import sys 

def make_array( aname, atype, asize, aval ):
  filename = "arrays/%s.radl" % aname 
  with open( filename, "w" ) as fd:
    print( 'Writing to file: %s' % filename )
    fd.write( "%s : array {\n" % aname )
    fd.write( "  SIZE %d\n" % asize )
    fd.write( "  TYPE \"%s\"\n" % atype )
    fd.write( "  VALUES \n" )
    nrow = 20
    nleft = asize
    while nleft > 0:
      n = nrow if nleft > nrow else nleft
      #fd.write( "    %s\n" % ' '.join( "%s %s" % ( atype, aval ) for i in range( n ) ) )
      fd.write( "    %s\n" % ' '.join( "%s" % aval for i in range( n ) ) )
      nleft -= n 
    fd.write( "}\n\n" )


if __name__ == "__main__":
  if len( sys.argv ) < 5:
    print( "Usage: %s <array_name> <array_type> <array_size> <array_val>" % sys.argv[0] ) 
    sys.exit( 0 )
  aname = sys.argv[1]
  atype = sys.argv[2]
  asize = int( sys.argv[3] )
  aval = sys.argv[4]
  make_array( aname, atype, asize, aval )
    


  

