import time
from numpy import zeros, array

class Ramp():
    def __init__(self):
        self.val = 0
        self.dur = 0.0
        self.A = 0 #origin val
        self.B = 0 #destination val
        self.t = 0.0  #previous time 
        self.pos = 0.0    #ramp position
        self.grain = 10.0 #millisec

    def go(self,val,dur): 
        self.A = self.val   
        self.B = val
        self.dur = dur
        self.t = time.time()*1000 


    def update(self):
        newTime = time.time()*1000 
        delta = newTime - self.t
        doUpdate = delta >= self.grain

        if(doUpdate):
            self.t = newTime
            if self.pos + delta < self.dur:
                self.pos = self.pos + delta
            else:
                self.pos = self.dur

            k = self.pos/self.dur
            self.val = self.A + (self.B - self.A)*k
        
        return self.val
        
class Interpolation():
    def __init__(self):
        self.myramp = Ramp()
        self.Flag = 0
        self.savedValue = 0

    def go(self,Input,duration):
        if Input != self.savedValue:
            self.Flag = 0
        self.savedValue = Input

        if self.Flag == 0:
            self.myramp.go(Input,duration)
            self.Flag = 1

        output = self.myramp.update()
        return output

inX1 = Interpolation()
inX2 = Interpolation()
inX3 = Interpolation()
inX4 = Interpolation()
inX5 = Interpolation()
inX6 = Interpolation()
x = 0
number = [283.71,141.855,-141.855,-283.71,-141.855,141.855]
count = 0
prevtime = time.time()*1000
while 1:
    currenttime = time.time()*1000
    for i in range(1000):
        pass
    #if x[0] < 50:
    for i in range(6):
        if i == 0:
            x = inX1.go(number[i],1000/3)
            print(x,end='\t')
        if i == 1:
            x = inX2.go(number[i],1000/3)
            print(x,end='\t')
        if i == 2:
            x = inX3.go(number[i],1000/3)
            print(x,end='\t')
        if i == 3:
            x = inX4.go(number[i],1000/3)
            print(x,end='\t')
        if i == 4:
            x = inX5.go(number[i],1000/3)
            print(x,end='\t')
        if i == 5:
            x = inX6.go(number[i],1000/3)
            print(x)
    count = count + 1
    if currenttime - prevtime >= 1000/3:
        print(count)
        prevtime = currenttime
        break  
    time.sleep(0.01)


