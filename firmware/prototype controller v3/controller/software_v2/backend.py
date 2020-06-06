# coding=utf-8

# vent4us 15/04/20
# The MIT License (MIT)
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os, sys
import numpy           as     np
import constants as constants
import core

def plot_sine(size, counter):
    ''' returns a scalar value of data for counter range '''
    x = np.linspace(constants.__XRANGE__[0], constants.__XRANGE__[1], size)
    y = np.sin(x[counter])*(constants.__YRANGE__[1]-constants.__YRANGE__[0])/2
    return y, x[counter]
def plot_cosine(size, counter):
    ''' returns a scalar value of data for counter range '''
    x = np.linspace(constants.__XRANGE__[0], constants.__XRANGE__[1], size)
    y = np.cos(x[counter])/2*((constants.__YRANGE__[1]-constants.__YRANGE__[0])/2)
    return y, x[counter]

def plot_normal_dist(size, counter):
    ''' returns a scalar value of data for counter range '''
    data  = np.random.normal(loc=0.0, scale=1.0)*((constants.__YRANGE__[1]-constants.__YRANGE__[0])/2)
    x     = np.linspace(constants.__XRANGE__[0], constants.__XRANGE__[1], size)
    return data, x[counter]