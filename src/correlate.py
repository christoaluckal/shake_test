import numpy as np
import pickle
import time
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from tqdm import tqdm
import cv2
import os

def readIMUPKL(pkl_loc="",skip=1,imu_num=0):
    print(pkl_loc)
    with open(pkl_loc,"rb") as f:
        data = pickle.load(f)

    key = f"imu{imu_num}"

    vals = np.array(data[key])

    seq_start = vals[0,0]
    
    seqs = vals[:,0]-seq_start
    time_stamps = vals[:,1]
    ax = vals[:,6]
    ay = vals[:,7]
    az = vals[:,8]

    seqs = seqs[::skip]
    ax = ax[::skip]
    ay = ay[::skip]
    az = az[::skip]

    return time_stamps,ax,ay,az

def readRADARPKL(pkl_loc="",skip=1):

    images = []
    time_stamps = []
    
    with open(pkl_loc,"rb") as f:
        data = pickle.load(f)

    for i,t in data:
        images.append(i)
        time_stamps.append(t)

    images = np.array(images)
    time_stamps = np.array(time_stamps)

    return time_stamps,images

def pairData(
    imu_t=None,
    imu_ax=None,
    imu_ay=None,
    imu_az=None,
    radar_t=None,
    radar_i=None
):
    pairs = []
    t_start = radar_t[0]
    for i in range(len(radar_t)):
        t = radar_t[i]
        idx = np.argmin(np.abs(imu_t-t))
        pairs.append([t-t_start,radar_i[i],imu_ax[idx],imu_ay[idx],imu_az[idx]])

    return pairs

def createVideo(
    image_array=None,
    video_name="video.avi"
):
    height, width, layers = image_array[0].shape
    size = (width,height)

    out = cv2.VideoWriter(video_name,cv2.VideoWriter_fourcc(*'mp4v'), 24, size)

    for i in range(len(image_array)):
        out.write(image_array[i])

    out.release()

def create(radar_pkl = "/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/radar/run5_2d.pkl", imu_pkl = "/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/imu/jun7_shake_white_3.pkl",
vid_name = "video.avi"
):
    t,ax,ay,az = readIMUPKL(pkl_loc=imu_pkl,imu_num=3)
    t_r,t_i = readRADARPKL(pkl_loc=radar_pkl)

    p = pairData(imu_t=t,imu_ax=ax,imu_ay=ay,imu_az=az,radar_t=t_r,radar_i=t_i)

    fig = plt.figure(figsize=(15,10))
    gs = GridSpec(3, 3, figure=fig)
    ax_image = fig.add_subplot(gs[:,0])
    ax_ax = fig.add_subplot(gs[0,1:])

    ax_ay = fig.add_subplot(gs[1,1:])
    ax_az = fig.add_subplot(gs[2,1:])

    ax_ax.set_title("AX")
    ax_ay.set_title("AY")
    ax_az.set_title("AZ")

    ax_ax.set_xlabel("Time (s)")
    ax_ay.set_xlabel("Time (s)")
    ax_az.set_xlabel("Time (s)")

    t_acc = []
    ax_acc = []
    ay_acc = []
    az_acc = []

    im_counter = 0
    image_array = []

    image = None

    for i in tqdm(range(len(p))):
        t,r,ax,ay,az = p[i]
        if image is None:
            image = ax_image.imshow(r)
        else:
            image.set_data(r)
        # image = ax_image.imshow(r)

        t_acc.append(t)
        ax_acc.append(ax)
        ay_acc.append(ay)
        az_acc.append(az)

        ax_ax.plot(t_acc,ax_acc,color="tab:blue")
        ax_ay.plot(t_acc,ay_acc,color="tab:orange")
        ax_az.plot(t_acc,az_acc,color="tab:green")
        
        plt.savefig(f"images/{im_counter}.png")
        plt.cla()

        image_array.append(cv2.imread(f"images/{im_counter}.png"))

        im_counter += 1

    createVideo(image_array=image_array,video_name=vid_name)

    os.system(f"rm images/*")

    del image_array
    del image

create(
    radar_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/radar/run3_2d.pkl",
    imu_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/imu/jun7_shake_white.pkl",
    vid_name="white_1.avi"
)

create(
    radar_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/radar/run4_2d.pkl",
    imu_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/imu/jun7_shake_white_2.pkl",
    vid_name="small_1.avi"
)

create(
    radar_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/radar/run5_2d.pkl",
    imu_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/imu/jun7_shake_white_3.pkl",
    vid_name="white_2.avi"
)

create(
    radar_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/radar/run6_2d.pkl",
    imu_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/imu/jun7_shake_white_4.pkl",
    vid_name="small_2.avi"
)

create(
    radar_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/radar/run7_2d.pkl",
    imu_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/imu/jun7_shake_white_5.pkl",
    vid_name="white_3.avi"
)

create(
    radar_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/radar/run8_2d.pkl",
    imu_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/imu/jun7_shake_white_6.pkl",
    vid_name="large_1.avi"
)

create(
    radar_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/radar/run9_2d.pkl",
    imu_pkl="/home/caluckal/Developer/summer2024/ros_ws/src/shake_test/src/day_3/imu/jun7_shake_white_7.pkl",
    vid_name="white_4.avi"
)
