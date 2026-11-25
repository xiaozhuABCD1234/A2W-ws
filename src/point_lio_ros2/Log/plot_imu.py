import numpy as np
import matplotlib.pyplot as plt

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
