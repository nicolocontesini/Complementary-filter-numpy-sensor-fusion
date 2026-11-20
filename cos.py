
import numpy as np
import matplotlib.pyplot as plt
tv = np.linspace(0, 4000, 625)
t1 = np.linspace(0, 4000, 25001)
true_angle = np.sin(0.05*t1)

#INTRODUCING GAUSSIAN NOISE
noise = np.random.normal(0.0, 0.5, 25001)
signal = noise + true_angle


t2 = np.linspace(0, 4000, 25001)
omega_noise = np.random.normal(0.0, 0.025, 25001)
true_speed = np.cos(0.05*t2) 


dt = 0.01
alpha = 0.962
#tau = dt * alpha / (1 - alpha)
comp_angle = np.zeros(25001)
accel_angle = np.array(signal)
omega_measured = np.zeros(25001)
B = np.zeros(25001)
filter_accuracy = np.zeros(25001)
comp_angle[0] = accel_angle[0]
B_max = 1.25 
k_drift = 0.05
           



list_RMSE = []
for i in range(1, 25001):
    B[i] = B_max * (1 - np.exp(-k_drift * i * dt))
    omega_measured[i] = true_speed[i] + B[i] + omega_noise[i]
    comp_angle[i] = alpha*(comp_angle[i-1] + (omega_measured[i])*dt) + (1-alpha)*(accel_angle[i])   
    filter_accuracy[i] = comp_angle[i] - true_angle[i]
    pwr = (comp_angle[i] - true_angle[i])**2
    list_RMSE.append(pwr)
  

RMSE = np.zeros(625)

for sample in range(625):
    
    RMSE[sample] = np.sqrt((np.sum(list_RMSE[sample * 40 : (sample + 1) * 40])) / 40)





plt.figure(figsize=(20, 7))
plt.plot(tv, RMSE, color="#1f77b4", linewidth=1.5, label='variance over time')
plt.xlabel("t (s)")
plt.ylabel("RMSE (°)")
plt.savefig('RMSE1.png', dpi=300, bbox_inches='tight')

plt.figure(figsize=(20, 7))
plt.plot(t1, comp_angle, color="#a65313", linewidth=1.5)
plt.plot(t1, true_angle, color="#1f77b4", linewidth=1.5)
plt.xlabel("t (s)")
plt.ylabel("heading (°)")
plt.savefig('comparison1.png', dpi=300, bbox_inches='tight')


plt.show()




