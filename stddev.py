import numpy as np

gps_x = np.loadtxt('config/log/Graph1.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
gps_x_stddev  = np.std(gps_x)
print("MeasuredStdDev_GPSPosXY = ", gps_x_stddev)

accelerotmeter_x = np.loadtxt('config/log/Graph2.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
accelerotmeter_x_stddev  = np.std(accelerotmeter_x)
print("MeasuredStdDev_AccelXY = ", accelerotmeter_x_stddev)