#!/usr/bin/env python 

import os
import sys
import shutil 
from ipsec_utils import * 

def make_racoon_header( file_obj = sys.stdout ):
  file_obj.write( '# Racoon IKE daemon configuration file.\n#\n' )
  file_obj.write( 'log notify;\n' )
  file_obj.write( 'path certificate "/etc/racoon/certs";\n\n' )

def make_racoon_remote( host_ip, host_name, peer_ip, peer_name, 
    file_obj= sys.stdout ):
  file_obj.write( 'remote %s\n{\n' % peer_ip )
  file_obj.write( '  exchange_mode main;\n' )
  file_obj.write( '  lifetime time 12 hour;\n' )
  file_obj.write( '  certificate_type plain_rsa "%s";\n' % host_name )
  file_obj.write( '  peers_certfile plain_rsa "%s.pub";\n' % peer_name )
  file_obj.write( '  verify_cert off;\n' )
  file_obj.write( '  proposal {\n' )
  file_obj.write( '    encryption_algorithm 3des;\n' )
  file_obj.write( '    hash_algorithm sha256;\n' )
  file_obj.write( '    authentication_method rsasig;\n' )
  file_obj.write( '    dh_group modp1024;\n' )
  file_obj.write( '  }\n' )
  file_obj.write( '  generate_policy off;\n' )
  file_obj.write( '}\n\n' )

def make_racoon_sainfo( file_obj= sys.stdout):
  file_obj.write( 'sainfo anonymous\n' )
  file_obj.write( '{\n' )
  file_obj.write( '	pfs_group modp1024;\n' )
  file_obj.write( '	encryption_algorithm 3des;\n' )
  file_obj.write( '	authentication_algorithm hmac_sha256;\n' )
  file_obj.write( '	compression_algorithm deflate;\n' )
  file_obj.write( '}\n' )

def make_setkey_header( file_obj = sys.stdout ):
  file_obj.write( '#!/usr/sbin/setkey -f\n#\n\n' )
  file_obj.write( 'flush;\n' )
  file_obj.write( 'spdflush;\n\n' )

def make_setkey_spd( host_ip, peer_ip, val, file_obj= sys.stdout ):
  file_obj.write( 'spdadd %s %s any -P %s ipsec\n' % ( host_ip, peer_ip, val ) )
  file_obj.write( '\tesp/transport//require\n' )
  file_obj.write( '\tah/transport//require;\n\n' )

def mkdirp( filename ):
  d = os.path.dirname( filename )
  if not os.path.exists( d ):
    os.makedirs( d )

if __name__ == "__main__":
  if len( sys.argv ) < 2:
    print( "Usage: %s <config>" % ( sys.argv[0] ) )

  ip = load_ip( sys.argv[1] )
  for h in ip.keys():
    racoon_conf = '%s/etc/racoon/racoon.conf' % ip[h]
    print( 'Writing to %s' % racoon_conf )
    mkdirp( racoon_conf )
    with open( racoon_conf, "w" ) as f:
      make_racoon_header( f )
      peers = set( ip.keys() ) - set( [h] )
      host_ip = h
      host_name = ip[h]
      for p in peers:
        peer_ip = p
        peer_name = ip[p]
        make_racoon_remote( host_ip, host_name, peer_ip, peer_name, f )
      make_racoon_sainfo( f )
    print( '==' )
    setkey_conf = '%s/etc/ipsec-tools.conf' % ip[h]
    print( 'Writing to %s' % setkey_conf )
    with open( setkey_conf, "w" ) as f:
      make_setkey_header( f )
      peers = set( ip.keys() ) - set( [h] )
      host_ip = h
      for p in peers:
        peer_ip = p
        make_setkey_spd( host_ip, peer_ip, 'out', f )
        make_setkey_spd( peer_ip, host_ip, 'in', f )
    print( '==' )
    src = "_certificates"
    dst = "%s/etc/racoon/certs" % host_name
    mkdirp( '%s/sample' % dst )
    src_file = "%s/%s" % ( src, host_name)
    dst_file = "%s/%s" % ( dst, host_name )
    shutil.copyfile( src_file, dst_file )
    print( "Copying %s to %s" % ( src_file, dst_file ) )
    for p in ip.values():
      src_file = "%s/%s.pub" % ( src, p )
      dst_file = "%s/%s.pub" % ( dst, p )
      shutil.copyfile( src_file, dst_file )
      print( "Copying %s to %s" % ( src_file, dst_file ) )

    print( '==' )
    


