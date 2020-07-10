def unsigned_to_signed(unsigned_array,N):
    signed = 0
    for i in range(N):
        signed = signed + int(unsigned_array[i])*(256**(N-1-i))
    signed = signed - (256**N)/2
    return signed

def unsigned_to_unsigned(unsigned_array,N):
    unsigned = 0
    for i in range(N):
        unsigned = unsigned + int(unsigned_array[i])*(256**(N-1-i))
    return unsigned

# Copied code

import sys, os

def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])

def css_file():
    fname = os.path.dirname(sys.modules['__main__'].__file__)+'/vent4us.css'
    if os.path.isfile(fname):
        return fname
    else:
        return os.path.dirname(sys.modules['__main__'].__file__)+'./vent4us.css'

def icon_file(fn):
    fname = os.path.dirname(sys.modules['__main__'].__file__)+'/'+fn
    if os.path.isfile(fname):
        return fname
    else:
        return os.path.dirname(sys.modules['__main__'].__file__)+'./'+fn
