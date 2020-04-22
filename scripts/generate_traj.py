import airsim
import time
import utm
import sys
import pandas as pd
import numpy as np
import cv2
import os
# connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
# client.enableApiControl(True)
car_controls = airsim.CarControls()
gps_init = None
df_dict = {}
noisy_dict1 = {}
noisy_dict2 = {}
folder = "../data/loop/"
try:
    print("Collecting Trajectory Data")
    while True:
        # get state of the car
        car_state = client.getCarState()
        # print(car_state)
        # print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))
        gps_data = client.getGpsData()
        lat = float(gps_data.gnss.geo_point.latitude)
        lng = float(gps_data.gnss.geo_point.longitude)
        if gps_init == None:
            start_time = time.time()
            utmc = utm.from_latlon(lat, lng)
            x_o = utmc[0]
            y_o = utmc[1]
            gps_init = True
        utmc = utm.from_latlon(lat, lng)
        x = utmc[0]
        y = utmc[1]
        vx = gps_data.gnss.velocity.x_val
        vy = gps_data.gnss.velocity.y_val
        t = time.time() - start_time
        noise1 = np.random.normal(0,2,4)
        noise2 = np.random.normal(0,3,4)
        df_dict[t] = {
            "Time":t,
            "X":x-x_o,
            "Y":y-y_o,
            "VX":vx,
            "VY":vy
        }
        noisy_dict1[t] = {
            "Time":t,
            "X":x-x_o+noise1[0],
            "Y":y-y_o+noise1[1],
            "VX":vx+noise1[2],
            "VY":vy+noise1[3]
        }
        noisy_dict2[t] = {
            "Time":t,
            "X":x-x_o+noise2[0],
            "Y":y-y_o+noise2[1],
            "VX":vx+noise2[2],
            "VY":vy+noise2[3]
        }
        responses = client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.Scene)])
        filename = folder + 'raw_images/img_{:f}'.format(t)
        airsim.write_file(os.path.normpath(filename + '.png'), responses[0].image_data_uint8)
        time.sleep(0.1) # rate of collection

except KeyboardInterrupt:
    print('Interrupted')
    print('Storing as CSV')
    gdf = pd.DataFrame.from_dict(df_dict,"index")
    ndf1 = pd.DataFrame.from_dict(noisy_dict1,"index")
    ndf2 = pd.DataFrame.from_dict(noisy_dict2,"index")
    gdf.to_csv(folder+'ground_truth.csv')
    ndf1.to_csv(folder+'gauss_noise1.csv')
    ndf2.to_csv(folder+'gauss_noise2.csv')
    sys.exit(0)
