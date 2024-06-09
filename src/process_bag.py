import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
import pickle
import time
from itertools import product
import os

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

def readData(pkl_loc="",skip=20):
    print(pkl_loc)
    with open(pkl_loc,"rb") as f:
        data = pickle.load(f)

    key = "imu0"

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


    # fig,axs = plt.subplots(3,1,figsize=(15,10))

    # axs[0].plot(seqs,ax,label="AX")
    # axs[1].plot(seqs,ay,label="AY")
    # axs[2].plot(seqs,az,label="AZ")

    # axs[0].legend()
    # axs[1].legend()
    # axs[2].legend()
    # plt.show()

    return seqs,ax,ay,az

def rotateVector(vec,roll,pitch,yaw):
    cos_gamma = np.cos(roll)
    sin_gamma = np.sin(roll)

    cos_beta = np.cos(pitch)
    sin_beta = np.sin(pitch)

    cos_alpha = np.cos(yaw)
    sin_alpha = np.sin(yaw)

    R_1 = np.array([
        [cos_gamma,-sin_gamma,0],
        [sin_gamma,cos_gamma,0],
        [0,0,1]
    ])

    R_2 = np.array([
        [cos_beta,0,sin_beta],
        [0,1,0],
        [-sin_beta,0,cos_beta]
    ])

    R_3 = np.array([
        [1,0,0],
        [0,cos_alpha,-sin_alpha],
        [0,sin_alpha,cos_alpha]
    ])

    R = np.dot(R_2,R_3)
    R = np.dot(R_1,R)

    R_ = np.dot(R,vec)

    return R_

def computeRotationOffset(pkl_loc=""):
    with open(pkl_loc,"rb") as f:
        data = pickle.load(f)

    # imus = data.keys()
    imus = ['imu0','imu1','imu2','imu3','imu4']
    for im in imus:
        vals = np.array(data[im])

        ox = vals[:,OX]
        oy = vals[:,OY]
        oz = vals[:,OZ]
        ow = vals[:,OW]
        ax = vals[:,AX]
        ay = vals[:,AY]
        az = vals[:,AZ]

        mean_ax = np.mean(ax)
        mean_ay = np.mean(ay)
        mean_az = np.mean(az)

        mean_v = [mean_ax,mean_ay,mean_az]

        ref_z = -9.80665
        ref_x = 0
        ref_y = 0

        alphas = np.linspace(-np.pi/1,np.pi/1,150)
        betas = np.linspace(-np.pi/1,np.pi/1,150)
        gammas = np.linspace(-np.pi/1,np.pi/1,150)

        l = [alphas,betas,gammas]

        angles = []
        vec_R = []

        for item in product(*l):
            angles.append(item)
            vec_R.append(rotateVector(mean_v,item[0],item[1],item[2]))

        vec_R = np.array(vec_R)

        deviations = []

        for i in range(len(angles)):
            deviations.append((vec_R[i][0]-ref_x)**2+(vec_R[i][1]-ref_y)**2+(vec_R[i][2]-ref_z)**2)

        deviations = np.array(deviations)
        min_d = np.argmin(deviations)

        print(im,angles[min_d])

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
    
# processFolder("/home/caluckal/Developer/summer2024/shake_data/shake_Jun_4","jun4")
# processFolder("/home/caluckal/Developer/summer2024/shake_data/shake_Jun_6","jun6")
# processFolder("/home/caluckal/Developer/summer2024/shake_data/shake_Jun_7/bags","jun7")
# computeRotationOffset('ambient.pkl')

# getData(bag_loc="/home/caluckal/Developer/summer2024/shake_data/shake_imu_ambient_2024-06-04-12-29-31.bag",pkl_name="ambient")
# readData("ambient.pkl")

# getData(bag_loc="/home/caluckal/Developer/summer2024/shake_data/shake_pre_white_2024-06-04-12-48-43.bag",pkl_name="white_1")
# readData("white_1.pkl")


# getData(bag_loc="/home/caluckal/Developer/summer2024/shake_data/shake_pre_white_2_2024-06-04-12-49-31.bag",pkl_name="white_2")
# readData("white_2.pkl")

# getData(bag_loc="/home/caluckal/Developer/summer2024/shake_data/shake_pre_white_3_2024-06-04-12-57-56.bag",pkl_name="white_3")
# readData("white_3.pkl")

# getData(bag_loc="/home/caluckal/Developer/summer2024/shake_data/shake_pre_white_4_2024-06-04-12-59-59.bag",pkl_name="white_4")
# readData("white_4.pkl")

# fig,axs = plt.subplots(3,1,figsize=(15,10))

# s,x,y,z = readData("ambient.pkl")
# axs[0].plot(s,x,label="Ambient AX",alpha=0.5,c="tab:blue")
# axs[1].plot(s,y,label="Ambient AY",alpha=0.5,c="tab:blue")
# axs[2].plot(s,z,label="Ambient AZ",alpha=0.5,c="tab:blue")

# s,x,y,z = readData("white_1.pkl")
# axs[0].plot(s,x,label="White Noise Short AX",alpha=0.5)
# axs[1].plot(s,y,label="White Noise Short AY",alpha=0.5)
# axs[2].plot(s,z,label="White Noise Short AZ",alpha=0.5)

# s,x,y,z = readData("white_2.pkl")
# axs[0].plot(s,x,label="White Noise Long AX",alpha=0.5,c="tab:blue")
# axs[1].plot(s,y,label="White Noise Long AY",alpha=0.5,c="tab:blue")
# axs[2].plot(s,z,label="White Noise Long AZ",alpha=0.5,c="tab:blue")

# s,x,y,z = readData("white_3.pkl")
# axs[0].plot(s,x,label="Real Shake AX",alpha=0.5,c="tab:red")
# axs[1].plot(s,y,label="Real Shake AY",alpha=0.5,c="tab:red")
# axs[2].plot(s,z,label="Real Shake AZ",alpha=0.5,c="tab:red")

# s,x,y,z = readData("white_4.pkl")
# axs[0].plot(s,x,label="White Noise 2 Long AX",alpha=0.5,c="tab:purple")
# axs[1].plot(s,y,label="White Noise 2 Long AY",alpha=0.5,c="tab:purple")
# axs[2].plot(s,z,label="White Noise 2 Long AZ",alpha=0.5,c="tab:purple")



# axs[0].legend()
# axs[1].legend()
# axs[2].legend()

# plt.show()


# imu0 (1.28519699465037, 0.015866629563584977, -1.5073298085405573)
# imu1 (-0.9678644033786736, 0.015866629563584977, -1.5390630676677268)
# imu2 (-0.17453292519943298, 0.015866629563584977, -1.5390630676677268)
# imu3 (-1.5707963267948966, 0.015866629563584977, -1.5707963267948966)
# imu4 (-1.1582639581416914, 0.015866629563584977, -1.5707963267948966)