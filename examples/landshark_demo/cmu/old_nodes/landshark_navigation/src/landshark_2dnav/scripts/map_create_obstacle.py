
import re
import numpy
import sys
import scipy.misc

from matplotlib import pyplot

import yaml

def read_pgm(filename, byteorder='>'):
  """Return image data from a raw PGM file as numpy array.

  Format specification: http://netpbm.sourceforge.net/doc/pgm.html

  """
  with open(filename, 'rb') as f:
    buffer = f.read()
  try:
    header, width, height, maxval = re.search(
        b"(^P5\s(?:\s*#.*[\r\n])*"
        b"(\d+)\s(?:\s*#.*[\r\n])*"
        b"(\d+)\s(?:\s*#.*[\r\n])*"
        b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
  except AttributeError:
    raise ValueError("Not a raw PGM file: '%s'" % filename)
  return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

def read_obs(filename):
  text_file = open(filename, "r")
  obstacles=[]
  for line in text_file:
    print "line = ", line
    line_comma = line.split(',')
    obstacles.append((float(line_comma[0]), float(line_comma[1]), float(line_comma[2].strip('\n')) ))
    print "obstacles = ", obstacles
  return obstacles
  

def map_convert_blank(image):
  image.setflags(write=True)
  image[image < 256] = 255
  print "image[1:10,1:10]", image[1:10,1:10]
  scipy.misc.imsave('../map/map_converted.pgm', image)

def map_meta_read(filename):
  f = open('../map/map.yaml')
  return yaml.safe_load(f)

def set_obstacles(image,obstacles,dataMap):
  image_obs=image
  image_obs.setflags(write=True)
  print "dataMap = ", dataMap
  origin = dataMap['origin']
  xmin = origin[0]
  ymin = origin[1]
  res = dataMap['resolution']
  print "int(xmin/res) = ", int(xmin/res)
  print "int(ymin/res) = ", int(ymin/res)
  shape = image.shape
  xmin_px = shape[0]
  ymin_px = shape[1]

  print "obstacles = ", obstacles

  for obstacle in obstacles:
    print "-int(obstacle[0]/res) - int(xmin/res) = ", -int(obstacle[0]/res) - int(xmin/res)
    print "-int(obstacle[1]/res) - int(ymin/res) = ", -int(obstacle[1]/res) - int(ymin/res)

    image_obs[int(obstacle[0]/res) - int(xmin/res) , int(obstacle[1]/res) - int(ymin/res) ] = 1
  
#    image_obs[int(obstacle[0]/res) - int(xmin/res) + 1, int(obstacle[1]/res) - int(ymin/res)] = 1
#    image_obs[int(obstacle[0]/res) - int(xmin/res), int(obstacle[1]/res) - int(ymin/res) + 1] = 1
#    image_obs[int(obstacle[0]/res) - int(xmin/res) + 1, int(obstacle[1]/res) - int(ymin/res) + 1] = 1

#    image_obs[int(obstacle[0]/res) - int(xmin/res) - 1, int(obstacle[1]/res) - int(ymin/res)] = 1
#    image_obs[int(obstacle[0]/res) - int(xmin/res), int(obstacle[1]/res) - int(ymin/res) - 1] = 1
#    image_obs[int(obstacle[0]/res) - int(xmin/res) - 1, int(obstacle[1]/res) - int(ymin/res) - 1] = 1 

#    image_obs[int(obstacle[0]/res) - int(xmin/res) + 1, int(obstacle[1]/res) - int(ymin/res) - 1] = 1  
#    image_obs[int(obstacle[0]/res) - int(xmin/res) - 1, int(obstacle[1]/res) - int(ymin/res) + 1] = 1  
  
    print "image_obs = "
    print image_obs      
  
  return image_obs
  

if __name__ == "__main__":
  map_yaml_file = sys.argv[1] 
  dataMap = map_meta_read(map_yaml_file)
  image_file = sys.argv[2]
  image = read_pgm(image_file, byteorder='<')
  obs_file = sys.argv[3]
  obstacles = read_obs(obs_file)
 
  image_obs = set_obstacles(image,obstacles,dataMap)

  scipy.misc.imsave('../map/map_obstacles.pgm', image_obs)

  print "image.shape = ", image.shape
  pyplot.imshow(image_obs[ :, : ], pyplot.cm.gray)

#  map_convert_blank(image)
#  print "image[550:565,425:440]", image[550:565,425:440]

  pyplot.show()


