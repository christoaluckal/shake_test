import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
import pickle
import time
from itertools import product
import os
from pprint import pprint

SEQ = 0
T = 1
OX = 2
OY = 3
OZ = 4
OW = 5
AX = 6
AY = 7
AZ = 8

def getData(bag_loc="",pkl_name="test",count=5):
    bag = rosbag.Bag(bag_loc)
    message_dict = {}
    for i in range(count):
        message_dict[f'imu{i}'] = []
        for topic, msg, t in bag.read_messages(topics=[f'/imu{i}']):
            imu_msg : Imu = msg
            t = imu_msg.header.stamp.secs + imu_msg.header.stamp.nsecs/10**9
            seq = imu_msg.header.seq
            ax = imu_msg.linear_acceleration.x
            ay = imu_msg.linear_acceleration.y
            az = imu_msg.linear_acceleration.z

            ox = imu_msg.orientation.x
            oy = imu_msg.orientation.y
            oz = imu_msg.orientation.z
            ow = imu_msg.orientation.w
            row = [seq,t,ox,oy,oz,ow,ax,ay,az]
            message_dict[f'imu{i}'].append(row)

    bag.close()

    for k,v in message_dict.items():
        print(pkl_name,k,len(v))

    with open(f"{pkl_name}.pkl","wb") as f:
        pickle.dump(message_dict,f)

def readData(pkl_loc="",skip=1,imu_num=0):
    print(pkl_loc)
    with open(pkl_loc,"rb") as f:
        data = pickle.load(f)

    key = f"imu{imu_num}"

    vals = np.array(data[key])

    seq_start = vals[0,0]
    
    seqs = vals[:,0]-seq_start
    ax = vals[:,6]
    ay = vals[:,7]
    az = vals[:,8]

    seqs = seqs[::skip]
    ax = ax[::skip]
    ay = ay[::skip]
    az = az[::skip]

    mu_x,mu_y,mu_z = np.mean(ax),np.mean(ay),np.mean(az)
    mu_sigma_x,mu_sigma_y,mu_sigma_z = np.std(ax),np.std(ay),np.std(az)


    

    return imu_num, [mu_x,mu_y,mu_z], [mu_sigma_x,mu_sigma_y,mu_sigma_z]


def processFolder(folder_loc="",prefix="test"):
    bags = os.listdir(folder_loc)
    bags = [f for f in bags if f.endswith(".bag")]
    names = []
    for b in bags:
        name = b.split("_")
        name = "_".join(name[:-1])
        name = prefix+"_"+name
        bag_loc = os.path.join(folder_loc,b)
        getData(bag_loc=bag_loc,pkl_name=name,count=10)
    
    
# white 1
data_dict = {}
white_noises = [
    "jun7_shake_imu_ambient.pkl",
    "jun7_shake_white.pkl",
    "jun7_shake_white_3.pkl",
    "jun7_shake_white_5.pkl",
    "jun7_shake_white_7.pkl",
]

# for w in white_noises:
#     name = w.split("_")[2:]
#     name = "_".join(name)
#     name = name.split(".")[0]
#     data_dict[name] = {}
#     for i in range(10):
#         imu_num, mu, sigma = readData(pkl_loc=w,imu_num=i)
#         data_dict[name][imu_num] = {
#             "mu": mu,
#             "sigma": sigma
#         }

with open("white_noise.pkl","rb") as f:
    # pickle.dump(data_dict,f)
    data_dict = pickle.load(f)



orientations = ["ax","ay","az"]
scale = 1

color_dict = {
    "imu_ambient": "tab:blue",
    "white": "tab:orange",
    "white_3": "tab:green",
    "white_5": "tab:red",
    "white_7": "tab:purple"
}

label_remap = {
    "imu_ambient": "Ambient",
    "white": "White Noise 1",
    "white_3": "White Noise 2",
    "white_5": "White Noise 3",
    "white_7": "White Noise after cracking"
}

fig,ax = plt.subplots(3,1,figsize=(18,21))

# plt.rcParams['font.size'] = 16

import pandas as pd

columns = ["type","imu","mu_ax","mu_ay","mu_az","sigma_ax","sigma_ay","sigma_az"]
df = pd.DataFrame(columns=columns)
df_rows = []
for k,v in data_dict.items():
    label = label_remap[k]
    imu_idx = []
    ax_vals = []
    ay_vals = []
    az_vals = []
    ax_sigma_l = []
    ay_sigma_l = []
    az_sigma_l = []
    for kk,vv in v.items():
        # data_row.append(vv["mu"])
        # imu_idx.append(kk)
        ax_mu = np.round(vv["mu"][0],5)
        ay_mu = np.round(vv["mu"][1],5)
        az_mu = np.round(vv["mu"][2],5)

        ax_sigma = np.round(vv["sigma"][0],5)
        ay_sigma = np.round(vv["sigma"][1],5)
        az_sigma = np.round(vv["sigma"][2],5)
        
        ax_vals.append(ax_mu*scale)
        ay_vals.append(ay_mu*scale)
        az_vals.append(az_mu*scale)

        ax_sigma_l.append(ax_sigma*scale)
        ay_sigma_l.append(ay_sigma*scale)
        az_sigma_l.append(az_sigma*scale)


        imu_idx.append(kk)

        data_row = [label,kk,ax_mu,ay_mu,az_mu,ax_sigma,ay_sigma,az_sigma]
        df_rows.append(data_row)

    # MU
    # ax[0].plot(imu_idx,ax_vals,label=f"AX mu {label}",c=color_dict[k])
    # ax[0].scatter(imu_idx,ax_vals,c=color_dict[k],s=10)
    # ax[0].set_xticks(imu_idx)
    # ax[0].set_xlabel("IMU Index")
    # ax[0].legend()

    # ax[1].plot(imu_idx,ay_vals,label=f"AY mu {label}",c=color_dict[k])
    # ax[1].scatter(imu_idx,ay_vals,c=color_dict[k],s=10)
    # ax[1].set_xticks(imu_idx)
    # ax[1].set_xlabel("IMU Index")
    # ax[1].legend()

    # ax[2].plot(imu_idx,az_vals,label=f"AZ mu {label}",c=color_dict[k])
    # ax[2].scatter(imu_idx,az_vals,c=color_dict[k],s=10)
    # ax[2].set_xticks(imu_idx)
    # ax[2].set_xlabel("IMU Index")
    # ax[2].legend()


    # STDEV
    ax[0].plot(imu_idx,ax_sigma_l,label=f"AX sigma {label}",c=color_dict[k])
    ax[0].scatter(imu_idx,ax_sigma_l,c=color_dict[k],s=10)
    ax[0].set_xticks(imu_idx)
    ax[0].set_xlabel("IMU Index")
    ax[0].legend()

    ax[1].plot(imu_idx,ay_sigma_l,label=f"AY sigma {label}",c=color_dict[k])
    ax[1].scatter(imu_idx,ay_sigma_l,c=color_dict[k],s=10)
    ax[1].set_xticks(imu_idx)
    ax[1].set_xlabel("IMU Index")
    ax[1].legend()

    ax[2].plot(imu_idx,az_sigma_l,label=f"AZ sigma {label}",c=color_dict[k])
    ax[2].scatter(imu_idx,az_sigma_l,c=color_dict[k],s=10)
    ax[2].set_xticks(imu_idx)
    ax[2].set_xlabel("IMU Index")
    ax[2].legend()

df = pd.DataFrame(df_rows,columns=columns)
df.to_csv("white_noise.csv",index=False)

# plt.suptitle("White Noise Mean Comparison")
plt.suptitle("White Noise SD Comparison")

plt.legend()
# plt.savefig("white_noise_mu.png")
plt.savefig("white_noise_sigma.png")
plt.show()