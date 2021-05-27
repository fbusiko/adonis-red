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

ndim = 2
k1 = Kalman(numpy.array([0, 0]), numpy.eye(ndim),0.03, 3e-5)   
x_init=numpy.array([4, 2])
cov_init=0.1*numpy.ones((ndim))

node_1_lat = 0
node_1_lng = 0
node_2_lat = 0
node_2_lng = 0
node_3_lat = 0
node_3_lng = 0
node_4_lat = 0
node_4_lng = 0
gps_lat_use = 0
gps_lng_use = 0


while(True):
        
    if not ser.isOpen():
        ser.open()
        
    test = '0'    
    while test != b'#':
            test = ser.read(1)
    
    data = ser.read(47)
    d_data = data.decode("utf-8")
    x = d_data.split("-")

    gps_lng_32 = int(x[0])
    gps_lng_24 = int(x[1])
    gps_lng_16 = int(x[2])
    gps_lng_8 = int(x[3])
    gps_lat_32 = int(x[4])
    gps_lat_24 = int(x[5])
    gps_lat_16 = int(x[6])
    gps_lat_8 = int(x[7])
    direction = int(x[8])
    node_1_rssi = int(x[9])
    node_2_rssi = int(x[10])
    node_3_rssi = int(x[11])

    gps_lat_long = (gps_lat_32<<24) | (gps_lat_24<<16) | (gps_lat_16<<8) | gps_lat_8
    gps_lat = float(gps_lat_long / 1000000)
    gps_lng_long = (gps_lng_32<<24) | (gps_lng_24<<16) | (gps_lng_16<<8) | gps_lng_8
    gps_lng = float(gps_lng_long / 1000000)

    if (gps_lat != 0):
        gps_lat_use = gps_lat
    if (gps_lng != 0):
        gps_lng_use = gps_lng

    if (direction == 1):
        gps_lat_use *= -1
    if (direction == 16):
        gps_lng_use *= -1
    if (direction == 17):
        gps_lng_use *= -1
        gps_lat_use *= -1

    print(gps_lat_use)
    print(gps_lng_use)

    k1.update(np.array([gps_lat_use, gps_lng_use]))

    if node_1_rssi < 30:
        lat = node_1_lat
        lng = node_1_lng
    elif node_2_rssi < 30:
        lat = node_2_lat
        lng = node_2_lng
    elif node_3_rssi < 30:
        lat = node_3_lat
        lng = node_3_lng
    elif node_4_rssi < 30:
        lat = node_4_lat
        lng = node_4_lng
    else:
        lat = k1.x_hat_est[0]
        lng = k1.x_hat_est[1]
    
    # lat = 83.2
    # lng = 152.3
    
    ####################_DASHBOARD_####################
    
    data_to_insert = {
        "variable": "Location",
        "value": "My Location",
        "location": {
            "lat": gps_lng_use,
            "lng": gps_lat_use,
        },
    }
    result = my_device.insert(data_to_insert)  # With response
    
    ####################_DASHBOARD_####################

    ser.close()
