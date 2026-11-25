import numpy as np
import matplotlib.pyplot as plt

a_out=np.loadtxt('mat_out.txt')
if((a_out.shape[1] != 19) & (a_out.shape[1] != 20)):
    ######for ikfom
    fig, axs = plt.subplots(4,2)
    lab_out = ['', 'out-x', 'out-y', 'out-z']
    plot_ind = range(7,10)
    time=a_out[:,0]
    axs[0,0].set_title('Attitude')
    axs[1,0].set_title('Translation')
    axs[2,0].set_title('Velocity')
    axs[3,0].set_title('Angular velocity')
    axs[0,1].set_title('Acceleration')
    axs[1,1].set_title('Gravity')
    axs[2,1].set_title('bg')
    axs[3,1].set_title('ba')
    for i in range(1,4):
        for j in range(8):
            axs[j%4, j//4].plot(time, a_out[:,i+j*3],'.-', label=lab_out[i])
    for j in range(8):
        axs[j%4, j//4].grid()
        axs[j%4, j//4].legend()
    plt.grid()
    ######for ikfom#######
else:
    #######for normal#######
    fig, axs = plt.subplots(3,2)
    lab_pre = ['', 'pre-x', 'pre-y', 'pre-z']
    lab_out = ['', 'out-x', 'out-y', 'out-z']
    plot_ind = range(7,10)
    time=a_out[:,0]
    time1 = a_pre[:,0]
    axs[0,0].set_title('Attitude')
    axs[1,0].set_title('Translation')
    axs[2,0].set_title('Velocity')
    axs[0,1].set_title('bg')
    axs[1,1].set_title('ba')
    axs[2,1].set_title('Gravity')
    for i in range(1,4):
        for j in range(6):
            axs[j%3, j/3].plot(time1, a_pre[:,i+j*3],'.-', label=lab_pre[i])
            axs[j%3, j/3].plot(time, a_out[:,i+j*3],'.-', label=lab_out[i])
    for j in range(6):
        axs[j%3, j//3].grid()
        axs[j%3, j//3].legend()
    plt.grid()
    #######for normal#######

#### Draw IMU data
fig, axs = plt.subplots(2)
imu=np.loadtxt('imu_pbp.txt')
time=imu[:,0]
axs[0].set_title('Gyroscope')
axs[1].set_title('Accelerameter')
lab_1 = ['gyr-x', 'gyr-y', 'gyr-z']
lab_2 = ['acc-x', 'acc-y', 'acc-z']
for i in range(3):

    	axs[0].plot(time, imu[:,i+1],'.-', label=lab_1[i])
    	axs[1].plot(time, imu[:,i+4],'.-', label=lab_2[i])
for i in range(2):
    axs[i].grid()
    axs[i].legend()
plt.grid()

# #### Draw time calculation
# 'weight' : 'normal',
# 'size'   : 12,
# }
# # n = a_out[:,1].size
# # time_mean = a_out[:,1].mean()
# # time_se   = a_out[:,1].std() / np.sqrt(n)
# # time_err  = a_out[:,1] - time_mean
# # feat_mean = a_out[:,2].mean()
# # feat_err  = a_out[:,2] - feat_mean
# # feat_se   = a_out[:,2].std() / np.sqrt(n)

# # # print(time_se)
# # # print(a_out3[:,2])
plt.show()
