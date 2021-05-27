from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from chirpstack_api.as_pb import integration
from google.protobuf.json_format import Parse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import sys
import serial
# import pylab
import threading
import numpy
import numpy as np
import math
import tago
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import KNeighborsRegressor
from sklearn.metrics import confusion_matrix

class Kalman:
    def __init__(self, x_init, cov_init, meas_err, proc_err):
        self.ndim = len(x_init)
        self.A = numpy.eye(ndim)        #state transition model
        self.H = numpy.eye(ndim)        #observation model
        self.x_hat =  x_init
        self.cov = cov_init
        self.Q_k = numpy.eye(ndim)*proc_err #covariance matrix of process noise
        self.R = numpy.eye(len(self.H))*meas_err   #covariance matrix of observation noise
        
    def update(self, obs):

        # Make prediction
        self.x_hat_est = numpy.dot(self.A,self.x_hat)
        self.cov_est = numpy.dot(self.A,numpy.dot(self.cov,numpy.transpose(self.A))) + self.Q_k

        # Update estimate
        self.error_x = obs - numpy.dot(self.H,self.x_hat_est)
        self.error_cov = numpy.dot(self.H,numpy.dot(self.cov_est,numpy.transpose(self.H))) + self.R
        self.K = numpy.dot(numpy.dot(self.cov_est,numpy.transpose(self.H)),numpy.linalg.inv(self.error_cov))
        self.x_hat = self.x_hat_est + numpy.dot(self.K,self.error_x)
        if ndim>1:
            self.cov = numpy.dot((numpy.eye(self.ndim) - numpy.dot(self.K,self.H)),self.cov_est)
        else:
            self.cov = (1-self.K)*self.cov_est 

ser = serial.Serial(
    port='/dev/tty.usbmodem14201',
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)

ser.isOpen()
MY_DEVICE_TOKEN = '39ddd257-71cd-4ce9-9473-a272f72edfb6'
my_device = tago.Device(MY_DEVICE_TOKEN)

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ax1.set_xlim([0, 8])
ax1.set_ylim([0, 4])

ndim = 2
k1 = Kalman(numpy.array([0, 0]), numpy.eye(ndim),0.03, 3e-5)   
k2 = Kalman(numpy.array([0, 0]), numpy.eye(ndim),0.03, 3e-5)   
x_init=numpy.array([4, 2])
cov_init=0.1*numpy.ones((ndim))

def start_receiving(name):
    print('working')
    httpd = HTTPServer(('', 1700), Handler)
    httpd.serve_forever()

thread = threading.Thread(target=start_receiving, args=(1,), daemon=True)
thread.start()

def animate(i):
        

    if not ser.isOpen():
        ser.open()
        test = '0'
        while test != b'\n':
            test = ser.read(1)

    x = ''
    while (True):
        n = ser.read(1)
        if (n == b'#'):
            i = b'0'
            while i != b'#':
                i = ser.read(1)
                x = x + i.decode("utf-8")
            break

    x = x[:len(x) - 1].split(":")

    gps_long = int(x[0])
    gps_lat = int(x[1])
    node_1_rssi = int(x[2])
    node_2_rssi = int(x[3])
    node_3_rssi = int(x[4])
    node_4_rssi = int(x[5])
    
    ####################_DASHBOARD_####################
    

    # for n in range(8):
    #     data_to_insert = {
    #         "variable": "mod1_node%d_rssi" % n,
    #         "value": int(x[n])
    #     }
    #     # "value": "(%.1f, %.1f)" % (nx, ny)
    #     my_device.insert(data_to_insert)  # With response
    
    # ## after implementing ultasonic values 
    # for n in range(4):
    #     data_to_insert = {
    #         "variable": "mob1_node%d_ultrasonic" % (n+8),
    #         "value": int(x[n+8])
    #     }
    #     # "value": "(%.1f, %.1f)" % (nx, ny)
    #     my_device.insert(data_to_insert)  # With response
    
    ####################_DASHBOARD_####################
    
    k1.update(np.array([4, 4]))

    ax1.clear()
    ax1.set_xlim([0, 8])
    ax1.set_ylim([0, 4])
    ax1.plot(k1.x_hat_est[0], k1.x_hat_est[1], 'k+')

    ser.close()
    
ani = animation.FuncAnimation(fig, animate, interval=500)
plt.show()