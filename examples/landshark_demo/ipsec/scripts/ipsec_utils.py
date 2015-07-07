#!/usr/bin/env python 

def load_ip( filename ):
  ip = dict()
  with open( filename, 'r' ) as f:
    for l in f.readlines():
      x = l.split()
      if len( x ) == 2:
        ip[x[0]] = x[1]
        print( 'Machine %s (%s)' % ( x[0], x[1] ) )
      else:
        print( 'Error reading line: %s' % l ) 
  return ip


