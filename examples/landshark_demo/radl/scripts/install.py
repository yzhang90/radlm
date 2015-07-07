#!/usr/bin/env python

import os 
import sys 
import subprocess 

def get_repository():
  thisfile = os.path.realpath( sys.argv[0] )
  thisdir = os.path.dirname( thisfile )
  repodir = os.path.realpath( thisdir + '/../../' ) 
  return repodir

def get_motd( src ):
  """
  """
  os.chdir( src )
  gitlog = subprocess.check_output( 'git log -n 1'.split() )
  build = 'installed %s' % subprocess.check_output('date') 
      

  motd = '\n'
  motd += 'This is the HACMS Landshark ('
  motd += ' '.join( x for x in build.split() ) + ')\n\n'
  motd += subprocess.check_output( 'uname -s -r -v -m'.split() ) + '\n'
  for l in gitlog.splitlines( True )[0:4]:
    motd += l
  return motd

def delete_install( ):
  for dirname, subdir, files in os.walk( '/opt/hacms', topdown=True ):
    for s in subdir:
      cmd = 'sudo rm -rf %s' % os.path.join( dirname, s )
      print cmd
      subprocess.call( cmd.split() )
    break

def install_sys_files( src ):
  print( 'You may need to type in the password' )
  base = '%s/radl/files' % src 
  # write motd
  motd = get_motd( src )
  print( motd )
  motd_file = '%s/etc/motd.tail' % base
  f = open( motd_file, 'w' )
  f.write( motd )
  f.close()

  ignore = list( [ "~", "swp", "swo" ] )
  # move files
  for dirname, subdirlist, filelist in os.walk( base ):
    for f in filelist:
      copy = True
      for i in ignore:
        if f.endswith( i ):
          copy = False
      fsrc = os.path.join( dirname, f )
      fdst = fsrc.replace( base, '' )
      if copy:
        print( '%s -> %s' % (fsrc, fdst) )
        mkdir = 'mkdir -p %s' % os.path.dirname( fdst )
        subprocess.call( mkdir.split() )
        subprocess.call( ['sudo', 'cp', fsrc, fdst] )
      else:
        print( "%s -> ()" % fsrc )


  

if __name__ == '__main__':
  src = get_repository()
  dst = "/opt/hacms"
  install_sys_files( src )

  
