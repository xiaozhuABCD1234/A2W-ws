import numpy as np
import matplotlib.pyplot as plt

a_out=np.loadtxt('mat_out.txt')
#######for normal#######
fig, axs = plt.subplots(3,2)
lab_out = ['', 'out-x', 'out-y', 'out-z']
plot_ind = range(7,10)
time=a_out[:,0]
axs[0,0].set_title('Attitude')
axs[1,0].set_title('Translation')
axs[2,0].set_title('Velocity')
axs[0,1].set_title('bg')
axs[1,1].set_title('ba')
axs[2,1].set_title('Gravity')
for i in range(1,4):
	for j in range(6):
	    axs[j%3, j//3].plot(time, a_out[:,i+j*3],'.-', label=lab_out[i])
	for j in range(6):
		axs[j%3, j//3].grid()
		axs[j%3, j//3].legend()
plt.grid()
    #######for normal#######

#### Draw IMU data
#for i in range(3):

#for i in range(2):

plt.show()
