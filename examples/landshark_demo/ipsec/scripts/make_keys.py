#!/usr/bin/env python 

import os
import sys
import shutil 
import subprocess
from ipsec_utils import * 

def make_plainrsa_private( filename, bits=4096 ):
  cmd = "plainrsa-gen -b %d -f %s" % ( bits, filename )
  subprocess.call( cmd.split() )

def make_plainrsa_public( priv_filename, public_filename ):
  pub_found = False
  with open( priv_filename, 'r' ) as f:
    for l in f.readlines():
      words = l.split()
      if len( words ) == 4:
        if words[2] == "PUB":
          pub_found = True
          with open( public_filename, 'w' ) as f2:
            f2.write( ' '.join( '%s' % x for x in words[1:] ) )
          break
  return pub_found

if __name__ == "__main__":
  
  if len( sys.argv ) < 2:
    print( "Usage: %s <config>" % ( sys.argv[0] ) )

  certfolder = "_certificates"
  if not os.path.exists(certfolder):
    os.makedirs(certfolder)

  ip = load_ip( sys.argv[1] )
  for h in ip.keys():
    h_name = ip[h]
    private_file = "%s/%s" % (certfolder, h_name)
    public_file = "%s/%s.pub" % (certfolder, h_name)
    if os.path.exists( private_file ):
      print( "Will use existing private key %s" % private_file )
    else:
      print( "Writing private key to %s" % private_file )
      make_plainrsa_private( private_file )
    print( "Writing public key to %s" % public_file )
    make_plainrsa_public( private_file, public_file )
