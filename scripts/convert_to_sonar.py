import pandas as pd
import numpy as np
import math

folder = "../data/loop/"
file = folder + "ground_truth.csv"
df = pd.read_csv(file)
df_dict = {}
noisy_dict1 = {}
noisy_dict2 = {}
t = 0
for index, row in df.iterrows():
    x = float(row[2])
    y = float(row[3])
    print(x)
    if x >= 0.0005 or x<= -0.0005:
        r = math.sqrt(x**2+y**2)
        th = math.atan(y/x)
        noise1 = np.random.normal(0,1,2)
        noise2 = np.random.normal(0,0.5,2)
        df_dict[t] = {
            "Time":t,
            "r":r,
            "theta":th
        }
        noisy_dict1[t] = {
            "Time":t,
            "r":r + noise1[0],
            "theta":th + noise1[1]
        }
        noisy_dict2[t] = {
            "Time":t,
            "r":r + noise2[0],
            "theta":th + noise2[1]
        }
        t += 1

gdf = pd.DataFrame.from_dict(df_dict,"index")
ndf1 = pd.DataFrame.from_dict(noisy_dict1,"index")
ndf2 = pd.DataFrame.from_dict(noisy_dict2,"index")
gdf.to_csv(folder+'ground_truth_rt.csv')
ndf1.to_csv(folder+'gauss_noise1_rt.csv')
ndf2.to_csv(folder+'gauss_noise2_rt.csv')
