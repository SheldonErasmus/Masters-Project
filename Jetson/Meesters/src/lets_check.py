import time

if __name__ == "__main__":

    try:
        prevtime = time.time()*1000
        while True:
            currtime = time.time()*1000
            if currtime - prevtime >= 1000:
                prevtime = currtime
                x = time.time()*1000
                for i in range(1000000): pass
                y = time.time()*1000
                print(y-x)
            

    except KeyboardInterrupt:
        print("Exit")