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

from   PyQt5.QtWidgets import QApplication, QWidget
from   pyqtgraph.Qt    import QtGui, QtCore
from   pyqtgraph       import PlotWidget, PlotCurveItem
from   utils           import *

import pyqtgraph       as     pg
import numpy           as     np
import constants as constants
import os, sys

class plot_4vent():
    __PEN__ = {
        'blue': pg.mkPen(color=(51, 153, 255)),
        'green': pg.mkPen(color=(51, 153, 51)),
        'dblue': pg.mkPen(color=(0, 0, 153)),
        'lgreen': pg.mkPen(color=(204, 255, 204)),
        'black': pg.mkPen(color=(0, 0, 0)),
    }
    __BRUSH__ = {
        'none': pg.mkBrush(color=(0, 0, 255, 0)),
        'blue': pg.mkBrush(color=(0, 0, 131, 250)),
        'green': pg.mkBrush(color=(0, 255, 0, 200)),
        'lblue': pg.mkBrush(color=(0, 0, 153, 100)),
        'lgreen': pg.mkBrush(color=(25, 255, 25, 150)),
    }
    def __init__(self, label, xrange, yrange, **kwargs):
        self.label  = label
        self.counter= 0; self.tmo = 0; self.iter = 0
        self.widget = pg.PlotWidget(args={'left': 'Flow mBar', 'bottom': 'Time(seconds)'})
        self.xrange = xrange
        self.yrange = yrange
        ''' the registered function needs to provide the data as initialized
            in the size here
        '''
        self.size  = constants.__SIZE__ # TODO Make sure the run_series is called with size for now
        if kwargs is not None:
            for key, value in kwargs.items():
                if key == "size":
                    self.size = value
        self.widget.getPlotItem().setTitle('<p style="color:rgb(0, 0, 131);font-size:20px;font-family:Menlo;">' +
                                            self.label+'</p>')
        #self.widget.getPlotItem().showGrid(x=True, y=True, alpha=255)
        ''' Allow for multiple time series to be plotted on same plot
            the dictionary holds two values - data function and plot
            TODO Move the above to a separate function
        '''
        self.series = []
        curve = pg.PlotCurveItem()
        curve_prev = pg.PlotCurveItem()
        self.widget.addItem(curve)
        self.widget.addItem(curve_prev)
        _def_dict = { 'name': 'default', 'func': self.default_get_data, 'plot': curve, 'plot_prev': curve_prev,
                     'pen': self.__PEN__['black'], 'brush': self.__BRUSH__['lblue'], 'fillLevel' : 0,
                     'rolldata': [], 'rolldata_prev': [], 'rollcounts':[] }
        _def_dict['plot'].setBrush(self.__BRUSH__['green'])
        _def_dict['plot'].setPen(self.__PEN__['black'])
        _def_dict['plot'].setFillLevel(_def_dict['fillLevel'])
        self.init_series(_def_dict)
        #self.series.append(_def_dict)
        self.widget.setStyleSheet(open(css_file()).read())
        self.widget.setRange(xRange=self.xrange, yRange=self.yrange)

    def __str__(self):
        return '({0.label!s})'.format(self.label)

    def init_series(self, dict_item):
        for i in range(self.size):
            dict_item['rolldata'].append(0.0)
            dict_item['rolldata_prev'].append(0.0)
        dict_item['rollcounts'] = np.linspace(self.xrange[0], self.xrange[1], self.size)

    def get(self):
        return self.widget

    def data_cursor(self, size, counter):
        ''' returns an array of data with size n and the x value range '''
        data = self.yrange[1]
        x = np.linspace(self.xrange[0], self.xrange[1], size)
        print(data, x[counter])
        return data, x[counter]

    def default_get_data(self, size, counter):
        ''' returns an array of data with size n and the x value range '''
        data = np.random.random()
        #data  = np.random.normal(loc=0.0, scale=1.0, size=self.size)
        x = np.linspace(constants.__XRANGE__[0], constants.__XRANGE__[1], size)
        return data, x[counter]

    def add_series(self, name, func):
        ''' Note: the functions need to be globals and included in this file via backend.py
                  for now overwrite the time series function
        '''
        for dict_item in self.series:
            for key in dict_item:
                  if  key == 'name' and name == dict_item['name']:
                      dict_item['func'] = func
                      return
        curve      = pg.PlotCurveItem()
        curve_prev = pg.PlotCurveItem()
        self.widget.addItem(curve)
        self.widget.addItem(curve_prev)
        def_dict = {'name': name, 'func': func, 'plot': curve, 'plot_prev': curve_prev, 'pen': self.__PEN__['black'],
                    'brush': self.__BRUSH__['blue'], 'fillLevel' : 0, 'rolldata': [], 'rolldata_prev': [],
                    'rollcounts':[]}
        def_dict['plot'].setBrush(self.__BRUSH__['blue'])
        def_dict['plot'].setPen(self.__PEN__['black'])
        def_dict['plot'].setFillLevel(def_dict['fillLevel'])
        self.init_series(def_dict)
        self.series.append(def_dict)
        curve.setData(y=def_dict['rolldata'], x=def_dict['rollcounts'])
        curve_prev.setData(y=def_dict['rolldata_prev'], x=def_dict['rollcounts'],
                           fillLevel=0, pen=self.__PEN__['black'], brush=self.__BRUSH__['lblue'])

    def run_series(self, size):
        ''' Note: the functions need to be globals and included in this file via backend.py'''
        for dict_item in self.series:
            brush = dict_item['brush']
            yd, xd = dict_item['func'](size, self.counter)
            dict_item['rolldata_prev'][self.counter]  = 0
            dict_item['rolldata'][self.counter]       = yd
            dict_item['rollcounts'][self.counter]     = xd
            #if (self.iter == 0):
            #    brush = self.__BRUSH__['none']
            #if self.tmo:
            #    dict_item['rolldata'][(self.counter+1) % self.size] = self.yrange[0]
            #    dict_item['rolldata'][(self.counter+2) % self.size] = self.yrange[1]
            dict_item['plot'].setData(y=dict_item['rolldata'], x=dict_item['rollcounts'])
            dict_item['plot_prev'].setData(y=dict_item['rolldata_prev'], x=dict_item['rollcounts'],
                                           fillLevel=0, pen=self.__PEN__['black'], brush=self.__BRUSH__['lblue'])
        self.counter = (self.counter+1) % self.size
        self.tmo     = self.tmo +1
        if self.counter == 0:
            self.iter = self.iter +1
            dict_item['rolldata_prev'][:self.size] = dict_item['rolldata'][:self.size]
            dict_item['rolldata'][:self.size]      = [0.0]*self.size
