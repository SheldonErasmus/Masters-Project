import time
from numpy import zeros, array

class Ramp():
    def __init__(self):
        self.val = 0.0
        self.dur = 0.0
        self.A = 0.0 #origin val
        self.B = 0.0 #destination val
        self.t = 0.0  #previous time 
        self.pos = 0.0    #ramp position
        self.grain = 10.0 #millisec

    def go(self,val,dur): 
        self.A = self.val   
        self.B = val
        self.dur = dur
        self.t = time.time()*1000 
        self.pos = 0.0

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
        self.savedValue = 0.0

    def go(self,Input,duration):
        if Input != self.savedValue:
            self.Flag = 0
        self.savedValue = Input

        if self.Flag == 0:
            self.myramp.go(Input,duration)
            self.Flag = 1

        output = self.myramp.update()
        return output

"""inX1 = Interpolation()
inX1.myramp.val = 429.75
inY1 = Interpolation()
inY1.myramp.val = 0.0
inZ1 = Interpolation()
inZ1.myramp.val = 8.0

inX2 = Interpolation()
inX2.myramp.val = 214.875
inY2 = Interpolation()
inY2.myramp.val = -372.174
inZ2 = Interpolation()
inZ2.myramp.val = 8.0

inX3 = Interpolation()
inX3.myramp.val = -214.875
inY3 = Interpolation()
inY3.myramp.val = -372.174
inZ3 = Interpolation()
inZ3.myramp.val = 8.0

inX4 = Interpolation()
inX4.myramp.val = -429.75
inY4 = Interpolation()
inY4.myramp.val = 0.0
inZ4 = Interpolation()
inZ4.myramp.val = 8.0

inX5 = Interpolation()
inX5.myramp.val = -214.875
inY5 = Interpolation()
inY5.myramp.val = 372.174
inZ5 = Interpolation()
inZ5.myramp.val = 8.0

inX6 = Interpolation()
inX6.myramp.val = 214.875
inY6 = Interpolation()
inY6.myramp.val = 372.174
inZ6 = Interpolation()
inZ6.myramp.val = 8.0
number = [283.71,141.855,-141.855,-283.71,-141.855,141.855]
number1 = [283.71,141.855,-141.855,-283.71,-141.855,141.855]
number2 = [283.71,141.855,-141.855,-283.71,-141.855,141.855]
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
            y = inY1.go(number[i],1000/3)
            z = inZ1.go(number[i],1000/3)
            print(x,end='\t')
        if i == 1:
            x = inX2.go(number[i],1000/3)
            y = inY2.go(number[i],1000/3)
            z = inZ2.go(number[i],1000/3)
            print(x,end='\t')
        if i == 2:
            x = inX3.go(number[i],1000/3)
            y = inY3.go(number[i],1000/3)
            z = inZ3.go(number[i],1000/3)
            print(x,end='\t')
        if i == 3:
            x = inX4.go(number[i],1000/3)
            y = inY4.go(number[i],1000/3)
            z = inZ4.go(number[i],1000/3)
            print(x,end='\t')
        if i == 4:
            x = inX5.go(number[i],1000/3)
            y = inY5.go(number[i],1000/3)
            z = inZ5.go(number[i],1000/3)
            print(x,end='\t')
        if i == 5:
            x = inX6.go(number[i],1000/3)
            y = inY6.go(number[i],1000/3)
            z = inZ6.go(number[i],1000/3)
            print(x)
    count = count + 1
    if currenttime - prevtime >= 1000/3:
        print(currenttime - prevtime)
        prevtime = currenttime
        break  
    time.sleep(0.01)"""


