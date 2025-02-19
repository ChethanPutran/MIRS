import math
import numpy as np
import matplotlib.pyplot as plt

class Trajectory:
    def __init__(self,theta=[],theta_dot=[],theta_dotdot=[],times=[]):
        self.trajectory = {}
        self.trajectory['theta'][:] = theta
        self.trajectory['theta_dot'][:] = theta_dot
        self.trajectory['theta_dotdot'][:] = theta_dotdot
        self.trajectory['times'][:] = times

    def get_trajectory_points(self):
        return self.trajectory['theta'],self.trajectory['theta_dot'],self.trajectory['theta_dotdot']
    
    def get_trajectory_times(self):
        return self.trajectory['times']
    
    def set_trajectory(self,theta,theta_dot,theta_dotdot,times):
        self.trajectory['theta'][:] = theta
        self.trajectory['theta_dot'][:] = theta_dot
        self.trajectory['theta_dotdot'][:] = theta_dotdot
        self.trajectory['times'][:] = times


class TrajectoryPlanner:
    def __init__(self,polynomial="cubic"):
        self.trajectory = Trajectory()
        self.polynomial = polynomial

    def compute_trapezoidal_trajectory(self,theta_0,theta_f,theta_dot_0,theta_dot_f,theta_dotdot_0,theta_dotdot_f):
        # self.polynomial = polynomial
        # qg=100*10**(-3)
        # qs= 0
        # tg= 6
        # delT = 0.1 #s (10Hz)
        # stepsize = delT
        # accl=20*10**(-3)
        # qm=0.5*(qg+qs)  # midposition
        # tm=tg/2         # mid-time

        # tb=0.5*tg-0.5*math.sqrt(((accl*tg**2)-4*(qg-qs))/(accl)) #blend time
        # tp=tg-tb                                                 #tg-tb
        # qb=qm-accl*tb*(tm-tb)                                    #blend position


        # tx = np.linspace(0,tb,int(1/delT))           # 0 <= t <= tb          -> 1st segment position 0 to tb
        # ty = np.linspace(tb,tp,int(1/delT))          # tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
        # tz = np.linspace(tp,tg,int(1/delT))
        # # tx = np.arange(0,tb,delT)            #1st segment time interval  0 to tb
        # # ty= np.arange(tb+delT,tp,delT)            #2nd segment time interval  tb to tg-tb
        # # tz= np.arange(tp,tg ,delT)          #3rd segment time interval  tg-tb to tg
        # t=np.concatenate((tx,ty,tz))


        # #Displacements
        # d1=qs + (0.5*accl*(tx**2))                       # 0 <= t <= tb  -> 1st segment position   0 to tb 
        # d2=qs + (0.5*accl*(tb**2)) + (accl*tb*(ty-tb))   # tb <= t <= (tg - tb) -> 2nd segment position  tb to tg-tb
        # d3=qg - (0.5*accl*((tg-tz)**2))                  #(tg - tb) <= t <= tg -> 3rd segment position  tg-tb to tg


        # d=np.concatenate((d1,d2,d3))

        # #Velocities
        # v1=accl*tx                         # 0 <= t <= tb  -> 1st segment velocity     0   to tb
        # v2=accl*tb*np.ones(ty.shape[0])    # tb <= t <= (tg - tb) -> 2nd segment velocity     tb  to tg-tb
        # v3=accl*(tg-tz)                    #(tg - tb) <= t <= tg -> 3rd segment velocity     tg-tb  to tg

        # v=np.concatenate((v1,v2,v3))

        # #Accelerations
        # a1=accl*np.ones(tx.shape[0])      # 0 <= t <= tb  -> 1st segment acceleration    0      to tb
        # a2=np.zeros(ty.shape[0])          # tb <= t <= (tg - tb) ->  2nd segment acceleration    tb     to tg-tb
        # a3=-accl*np.ones(tz.shape[0])     #(tg - tb) <= t <= tg -> 3rd segment acceleration    tg-tb  to tg
        # a =np.concatenate((a1,a2,a3))

        # fig, ax = plt.subplots(1,3)
        # fig.set_figwidth(20)
        # fig.set_figheight(5)

        # #Plot of Position vs Time 
        # ax[0].plot(t,d)
        # ax[0].set_title("Plot of Position vs Time ")
        # ax[0].set_ylabel('Position, q(s) (in m)')
        # ax[0].set_xlabel('Time, t (in s)')

        # #Plot of Velocity vs Time
        # ax[1].plot(t,v)
        # ax[1].set_title("Plot of Velocity vs Time")
        # ax[1].set_ylabel('Velocity (in m/s)')
        # ax[1].set_xlabel('Time (in s)')

        # #Plot of Acceleration vs Time
        # ax[2].plot(t,a)
        # ax[2].set_title("Plot of Acceleration vs Time")
        # ax[2].set_ylabel('Acceleration (in m/s**2)')
        # ax[2].set_xlabel('Time (in s)')

        # plt.savefig("Plots",dpi=150)
        # plt.show()

        # def generateSignals(n,amps,times,step_size):    
        #     vals = []
        #     for i in range(0,n):
        #         data = signal(times[i],amps[i],step_size)

        #         for d in data:
        #             vals.append(d)
        #     data = np.array(vals)
        #     X = data[:,0]
        #     Y = data[:,1]
            
        #     return X,Y
            
        # def signal(time,amp,step_size):
        #     a = [(time,0),(time,amp),(time+(step_size/2),amp),(time+(step_size/2),0),(time+step_size,0)]
        #     return a


        # X,Y = generateSignals(n,v,t,stepsize)

        # f = plt.figure()
        # f.set_figwidth(20)
        # f.set_figheight(5)
        # plt.plot(t,v)
        # plt.scatter(t,v)
        # plt.plot(X,Y)
        # plt.xlabel("Time (in s)")
        # plt.ylabel("Step size (in m)")
        # plt.title("Plot of Step size (in m) vs Time (in s)")
        # plt.savefig('plot.png',dpi=150)


        # qg = 100*10**-3
        # qs = 0
        # tg = 6
        # #Fs = 10000
        # accl = 20*10**-3
        # delT = .1 # (in s) (10Hz)
        # stepsize = delT
        # qm = 0.5*(qg + qs)                                         # midposition
        # tm = tg / 2                                                # mid-time
        # tb = 0.5 * tg - (0.5*np.sqrt(((accl*tg**2)-4*(qg-qs))/(accl))) # blend time
        # tp = tg - tb                                               # tg-tb
        # qb = qm - (accl * tb * (tm - tb))                          # blend position


        # #============= Square signal generator function #=============#

        # def signal(time,amplitude,step_size):
        #     sig = [
        #             [time                 ,     0],
        #             [time                 , amplitude],
        #             [time + (step_size/2) , amplitude],
        #             [time + (step_size/2) ,     0],
        #             [time +  step_size    ,     0],
        #         ]
        #     return sig

        # def generateSignal(n,amplitudes,time_stamps,step_size):
        #     points = np.zeros((5*n,2))
        #     num = 0

        #     for i in range(0,n):
        #         wave = signal(time_stamps[i],amplitudes[i],step_size)
        #         for j in range(0,len(wave)):
        #             points[int(num),:] = wave[j]
        #             num += 1

        #     X = points[:,0]
        #     Y = points[:,1]
        #     return X,Y


        # #=============================================== Time ===============================================#
        # tx = np.linspace(0,tb,int(1/delT))           # 0 <= t <= tb          -> 1st segment position 0 to tb
        # ty = np.linspace(tb,tp,int(1/delT))           # tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
        # tz = np.linspace(tp,tg,int(1/delT))            # (tg - tb) <= t <= tg  -> 3rd segment position tg-tb to tg

        # t = np.concatenate((tx,ty,tz))


        # #=============================================== Displacements ===============================================#
        # d1 = qs + 0.5*accl*(tx**2)                         # 0 <= t <= tb          -> 1st segment position 0 to tb
        # d2 = qs + (0.5*accl*(tb**2)) + (accl*tb*(ty-tb))   # tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
        # d3 = qg - (0.5*accl*((tg-tz)**2))                  # (tg - tb) <= t <= tg  -> 3rd segment position tg-tb to tg
        # d = np.concatenate((d1,d2,d3))


        # #=============================================== Velocities ===============================================#

        # v1 = accl*tx                         # 0 <= t <= tb          -> 1st segment position 0 to tb
        # v2 = accl*tb*np.ones(ty.shape[0])          # tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
        # v3 = accl*(tg-tz)                    # (tg - tb) <= t <= tg  -> 3rd segment position tg-tb to tg
        # v = np.concatenate((v1,v2,v3))



        # #p = polyfit(t,v,1)
        # #xlswrite('data.xlsx',p)

        # #=============================================== Accelerations ===============================================#

        # a1 = accl*np.ones(tx.shape[0])              #1st segment acceleration 0 to tb
        # a2 = 0*ty                             #2nd segment acceleration tb to tg-tb
        # a3 = -accl*np.ones(tz.shape[0])             #3rd segment acceleration tg-tb to tg
        # a = np.concatenate((a1,a2,a3))



        # # Generating square signal
        # n = len(t)
        # [X,Y]= generateSignal(n,v,t,stepsize)


        # fig, ax = plt.subplots(1,3)
        # fig.set_figwidth(20)
        # fig.set_figheight(5)

        # #Plot of Position vs Time 
        # ax[0].plot(t,d)
        # ax[0].set_title("Plot of Position vs Time ")
        # ax[0].set_ylabel('Position, q(s) (in m)')
        # ax[0].set_xlabel('Time, t (in s)')

        # #Plot of Velocity vs Time
        # ax[1].plot(t,v)
        # ax[1].set_title("Plot of Velocity vs Time")
        # ax[1].set_ylabel('Velocity (in m/s)')
        # ax[1].set_xlabel('Time (in s)')

        # #Plot of Acceleration vs Time
        # ax[2].plot(t,a)
        # ax[2].set_title("Plot of Acceleration vs Time")
        # ax[2].set_ylabel('Acceleration (in m/s**2)')
        # ax[2].set_xlabel('Time (in s)')

        # plt.savefig("Plots",dpi=150)
        # plt.show()

        # f = plt.figure()
        # f.set_figwidth(20)
        # f.set_figheight(5)
        # plt.plot(t,v)
        # plt.scatter(t,v)
        # plt.plot(X,Y)
        # plt.xlabel("Time (in s)")
        # plt.ylabel("Step size (in m)")
        # plt.title("Plot of Step size (in m) vs Time (in s)")
        # plt.savefig('plot.png',dpi=150)
        pass

    def compute_cubic_trajectory(self,theta_0,theta_f,theta_dot_0,theta_dot_f,theta_dotdot_0,theta_dotdot_f,t_f,time_step=0.1):
        """Perform a cubic interpolation between two trajectory points."""
        
        t = np.linspace(0,t_f,time_step,endpoint=True)

        a = theta_0
        b = theta_dot_0
        c = (-3 * theta_0 + 3 * theta_f- 2 * t_f * theta_dot_0 - t_f * theta_dot_f) /t_f**2
        d = (2 * theta_0 - 2 * theta_f + t_f * theta_dot_0 + t_f * theta_dot_f) / t_f**3

        q = a + b * t + c * t**2 + d * t**3
        qdot = b + 2 * c * t + 3 * d * t**2
        qddot = 2 * c + 6 * d * t
        return Trajectory(q, qdot, qddot, t)
        
    def compute_quintic_trajectory(self,theta_0,theta_f,theta_dot_0,theta_dot_f,theta_dotdot_0,theta_dotdot_f,t_f,time_step=0.1):
        """Perform a quintic interpolation between two trajectory points."""
        
        t = np.linspace(0,t_f,time_step,endpoint=True)

        a = theta_0
        b = theta_dot_0
        c = (-3 * theta_0 + 3 * theta_f- 2 * t_f * theta_dot_0 - t_f * theta_dot_f) /t_f**2
        d = (2 * theta_0 - 2 * theta_f + t_f * theta_dot_0 + t_f * theta_dot_f) / t_f**3
        e = 0
        f = 0

        q = a + b * t + c * t**2 + d * t**3 + e * t**4 + f * t**5
        qdot = b + 2 * c * t + 3 * d * t**2 + 4*e*t**3 + 5*f*t**4
        qddot = 2 * c + 6 * d * t + 12*e*t**2 + 20*f*t**3
        return Trajectory(q, qdot, qddot, t)
        
    def plan(self,theta_0,theta_f,theta_dot_0,theta_dot_f,theta_dotdot_0,theta_dotdot_f):
        if(self.polynomial=='cubic'):
            trajectory = self.compute_cubic_trajectory(theta_0,theta_f,theta_dot_0,theta_dot_f,theta_dotdot_0,theta_dotdot_f)
        elif(self.polynomial=="trapezoidal"):
            trajectory = self.compute_trapezoidal_trajectory(theta_0,theta_f,theta_dot_0,theta_dot_f,theta_dotdot_0,theta_dotdot_f)
        else:
            trajectory = self.compute_quintic_trajectory(theta_0,theta_f,theta_dot_0,theta_dot_f,theta_dotdot_0,theta_dotdot_f)

        return trajectory
    
