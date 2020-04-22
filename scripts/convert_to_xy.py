import pandas as pd
import numpy as np
import math

folder = "./"
#folder = "./"
file = folder + "ground_truth.csv"
df = pd.read_csv(file)
df_dict = {}
noisy_dict1 = {}
noisy_dict2 = {}
t = 0
for index, row in df.iterrows():
    sensX =  0 
    sensY =  0
    x = float(row[2]) - sensX
    y = float(row[3]) - sensY
    vx = float(row[4])
    vy = float(row[5])
    print(x)
    if x >= 0.0005 or x<= -0.0005:
        r = math.sqrt(x**2+y**2)
        th = np.arctan2(y, x)
        noise1x = np.random.normal(0,2,2)
        noise1y = np.random.normal(0,0.5,2)
        noise2x = np.random.normal(0,0.5,2)
        noise2y = np.random.normal(0,2, 2)
        noise1_vel = np.random.normal(0,0.8,2)
        noise2_vel = np.random.normal(0,0.4,2)
        noise1_t = np.random.normal(0,0.2,1)
        noise2_t = np.random.normal(0,0.02,1)
        df_dict[t] = {
            "Time":t,
            "r":r,
            "theta":th
        }
        noisy_dict1[t] = {
            "Time":t,
            "x": x + noise1x[0],
            "y": y + noise1y[0],
            "vx":vx + noise1x[1],
            "vy":vy + noise1y[1]
        }
        noisy_dict2[t] = {
            "Time":t,
            "x": x + noise2x[0],
            "y": y + noise2y[0],
            "vx":vx + noise2x[1],
            "vy":vy + noise2y[1]
        }
        t += 1

gdf = pd.DataFrame.from_dict(df_dict,"index")
ndf1 = pd.DataFrame.from_dict(noisy_dict1,"index")
ndf2 = pd.DataFrame.from_dict(noisy_dict2,"index")
gdf.to_csv(folder+'ground_truth_rt.csv')
ndf1.to_csv(folder+'gauss_noise1.csv')
ndf2.to_csv(folder+'gauss_noise2.csv')
