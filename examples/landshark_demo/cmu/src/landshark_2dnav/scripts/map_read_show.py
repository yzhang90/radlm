
import re
import numpy
import sys
import scipy.misc

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

def map_convert_blank(image):
  image.setflags(write=True)
  image[image < 256] = 255
  print "image[1:10,1:10]", image[1:10,1:10]
  scipy.misc.imsave('../map/map_converted.pgm', image)

def map_convert_blank_smaller(image):
  image.setflags(write=True)
  image[image < 256] = 255
  print "image[1:10,1:10]", image[1:10,1:10]
  scipy.misc.imsave('../map/map_converted.pgm', image[0:101,0:101])

if __name__ == "__main__":
  from matplotlib import pyplot
  image_file = sys.argv[1]
  print "im here"
  image = read_pgm(image_file, byteorder='<')
  print "image.shape = ", image.shape
#  pyplot.imshow(image, pyplot.cm.gray)

#  map_convert_blank(image)
  map_convert_blank_smaller(image)
#  print "image[550:565,425:440]", image[550:565,425:440]

  pyplot.show()


