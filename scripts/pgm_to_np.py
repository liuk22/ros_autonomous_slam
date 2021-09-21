import re
import numpy as np

PGM_FILE_PATH = "../maps/my_map.pgm"

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
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

'''
arr = read_pgm(PGM_FILE_PATH)
print(arr.shape)
from matplotlib import pyplot
pyplot.imshow(arr, pyplot.cm.gray)
pyplot.show()

'''
from matplotlib import pyplot
arr = np.loadtxt("09-21-2021T10-17-25_stage9_thresh90.txt")
print(arr.shape)
print(np.sum(arr < 0) / arr.shape[0] / arr.shape[1])
pyplot.imshow(arr, pyplot.cm.gray)
pyplot.show()