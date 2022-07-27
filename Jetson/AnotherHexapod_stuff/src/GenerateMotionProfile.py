import matplotlib.pyplot as plt
import numpy as np

class TrapezoidalProfile:
    def __init__(self):
        pass

    def GenerateProfile(self,start_pos,end_pos,time = None):
        Vmax = 0.1
        Tot_distance = end_pos - start_pos
        self.time = Tot_distance/Vmax+10 if time <= Tot_distance/Vmax else time
        
        dis_acc_dec = self.time /2*Vmax #only accelerating to top speed then immediately decelerating back

        if dis_acc_dec >= Tot_distance:
            Vmax = Tot_distance/(self.time /2)
            t_acc = self.time /2
            t_dec = t_acc
            t_vel = 0

        elif dis_acc_dec < Tot_distance:
            t_acc = self.time  - Tot_distance/Vmax
            t_dec = t_acc
            t_vel = self.time  - t_acc - t_dec

        self.F = lambda t: 0 if t<=0 else (Vmax/t_acc*t if t>0 and t<t_acc else (Vmax if t>=t_acc and t<=t_acc+t_vel else (-Vmax/t_dec*t + Vmax*self.time/t_dec if t>t_acc+t_vel and t<=t_acc+t_dec+t_vel else 0)))

class StepProfile:
    def __init__(self,Vmax,time):
        self.Vmax = Vmax
        self.Tot_time = time

    def GenerateProfile(self,cur_pos,end_pos,cur_time = None):
        Tot_distance = end_pos - cur_pos
        Remain_time = self.Tot_time - cur_time
        Vtop = Tot_distance/Remain_time

        if Vtop > self.Vmax:
            Remain_time = Tot_distance/self.Vmax
            self.Tot_time = Remain_time + cur_time
            Vtop = self.Vmax
            print('Velocity can not exceed {}, now using {} as total time' .format(self.Vmax,self.Tot_time))

        self.F = lambda t: 0 if t<0 else (Vtop if t>=0 and t<= cur_time+Remain_time else 0)
    
            

        




# trap = StepProfile(0.1)
# trap.GenerateProfile(0,10,50)

# tpoints = np.linspace(0,trap.time,1000)
# fpoints = [trap.F(val) for val in tpoints]
# plt.plot(tpoints,fpoints)
# plt.show()

