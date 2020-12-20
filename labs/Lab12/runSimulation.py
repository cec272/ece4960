# RUN ME
# This file gives the initial condition to the scipy.integrat.odeint function 
# and plots the resulting state outputs at each time step in an animation and
# on a plot that compares the actual output with the reference input

import numpy as np
import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import control
import pendulumParam as P
import scipy.linalg
import numpy.linalg
from kalmanFilter import kalmanFilter
from pendulumNonlinearDynamics import Pendulum
from pendulumAnimation import pendulumAn
from plotDataZ import plotData      
from signalGenerator import signalGen

#Compute controller
#dpoles = np.array([-1.1,-1.2,-1.3,-1.4])
#Kr = control.place(P.A,P.B,dpoles)
# LQR
Q = np.array([[0.4, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 821, 0],
              [0, 0, 0, 206]])
R = np.array([[1]])
S = scipy.linalg.solve_continuous_are(P.A, P.B, Q, R)
Kr = np.linalg.inv(R).dot(P.B.transpose().dot(S))

#Initialize and rename for convenience
ref = signalGen(amplitude=.5, frequency=0.05, y_offset=0) 
pendulum = Pendulum(param=P)

states = [np.array([[P.z0], [P.zdot0], [P.theta0], [P.thetadot0]])]
states_est = [states[0]]
mu = [np.asarray(states[0])+0.1] #np.array([[P.z0], [P.zdot0], [P.theta0], [P.thetadot0]])
sigma = [np.eye(4)*0.00001]
u=[0]
u_des=[0]
control_times = [0]
tof_times = [0]
imu_times = [0]

#performs very simple first order integration 
length = int((P.t_end-P.t_start)/P.Ts)              #The number of time steps over the time interval
t_array = np.linspace(P.t_start, P.t_end, length)   #The time vector for integration.
dt=t_array[1] - t_array[0]

for t in t_array[:-1]:
    des_state = np.array([[ref.square(t)[0]], [0.0], [0.0], [0.0]])
    old_state=states[-1]

    #Update controller and sensors every <T_update> seconds
    if (t % P.T_update_controller) < dt: # controller update rate
        # calculate control inputs and include deadband and saturation
        u_des.append(float(-Kr.dot(mu[-1]-des_state)))
        if abs(u_des[-1]) <= P.u_min: # deadband
            u.append(0)
        elif abs(u_des[-1]) >= P.u_max: #saturation
            u.append(np.sign(u_des[-1])*P.u_max)
        else:
            u.append(u_des[-1])
        # get sensor measurements
        n_y = P.C.dot(np.array([[np.random.normal(0, P.sig_z)], [np.random.normal(0, P.sig_zdot)], [np.random.normal(0, P.sig_theta)], [np.random.normal(0, P.sig_thetadot)]]))
        y_kf = P.C.dot(old_state)
        if (t % P.T_update_tof) < dt: # tof update rate
            y_kf[0,0] = P.C[0,0]*old_state[0,0] + n_y[0,0]
            tof_times.append(t)
        if (t % P.T_update_imu) < dt: # imu update rate
            y_kf[1,0] = P.C[0,3]*old_state[3,0] + n_y[1,0]
            imu_times.append(t)
        # estimate state from kalman filter
        mu_out,sigma_out = kalmanFilter(mu[-1],sigma[-1],u[-1],y_kf)
        # add stuff to lists for debugging
        mu.append(mu_out)
        sigma.append(sigma_out)
        control_times.append(t)
        
    new_state=old_state + np.array(pendulum.cartpendfunc(old_state,u[-1])) * dt
    
    #Arrays for debugging
    states_est.append(mu[-1])
    states.append(new_state)
    
#animation
plt.close('all')
animation = pendulumAn()

#Recast arrays for plotting
states = np.asarray(states, dtype=np.float64)
states_est = np.asarray(states_est, dtype=np.float64)
mu = np.asarray(mu, dtype=np.float64)
sigma = np.asarray(sigma, dtype=np.float64)
control_times = np.asarray(control_times)
imu_times = np.asarray(imu_times)
tof_times = np.asarray(tof_times)

i = 0
reference = np.zeros((length,1))
while i < len(t_array):  
    #calculate the reference value
    t_curr = t_array[i]                 #Current time step
    z_ref = ref.square(t_curr)[0]       #Get cart z location
    reference[i:(i+100)] = z_ref        #Which reference value is used

    #update animation and data plots
    animation.drawPendulum(states[i,:]) #Animate
    plt.pause(0.001)                    #Pause while plot updates
    i = i + 100                         #speeds up the simulation by not plotting all the points
 
# Plot how closely the actual performance of the pendulum on a cart matched the desired performance
dataPlot = plotData()                       #initializes plot
dataPlot.Plot(t_array, reference, states, control_times, u_des, 2, 'state', 'desired')     #plots the data
dataPlot.Plot(t_array, reference, states_est, control_times, u, .5, 'est. state', 'actual') #plot the estimate

# Plot our sigmas
fig, axs = plt.subplots(4, 1, sharex=True)
#plt.tight_layout(pad=1.04, h_pad=None, w_pad=None, rect=None)
plt.xlabel('Time [s]')
axs[0].plot(control_times, sigma[:, 0, 0])
axs[0].set_ylabel('[m]')
axs[0].set_title('sigma-z')
axs[1].plot(control_times, sigma[:, 1, 1])
axs[1].set_ylabel('[m/s]')
axs[1].set_title('sigma-zdot')
axs[2].plot(control_times, sigma[:, 2, 2])
axs[2].set_ylabel('[m/s]')
axs[2].set_title('sigma-thetadot')
axs[3].plot(control_times, sigma[:, 3, 3])
axs[3].set_ylabel('[m]')
axs[3].set_title('sigma-theta')

# Check our sample rate
trigger_end = 0.22 #seconds
control_indicies = np.where(control_times<=trigger_end)
tof_indicies = np.where(tof_times<=trigger_end)
imu_indicies = np.where(imu_times<=trigger_end)

fig, axs = plt.subplots(3, 1, sharex=True)
fig.suptitle('Sampling Comparison from t=0s to t=' + str(trigger_end) + 's')
plt.xlabel('Time [s]')
axs[0].scatter(control_times[control_indicies], np.zeros(len(control_times[control_indicies])))
axs[0].set_ylabel('u samples')
axs[1].scatter(tof_times[tof_indicies], np.zeros(len(tof_times[tof_indicies])))
axs[1].set_ylabel('z samplels')
axs[2].scatter(imu_times[imu_indicies], np.zeros(len(imu_times[imu_indicies])))
axs[2].set_ylabel('thetadot samples')
