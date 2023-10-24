from collections import defaultdict
import numpy as np
import sys

# Constant
class Global:
    n = 0 # Number of pickup points
    carLen = 5.00 # Length of car
    d_c = 20.00 # Distance between pickup points
    t_s = 30 # Expected service time
    v = 30/3.6 # Safe driving speed
    t = 0.01 # time step
    service_t = 99.0
    l_road = 100

# Global variables
current_code = 1 # Car count
conflicts = set() # Number of conflicts
remainingServiceTime = {} # Remain service time for every pickup point
carsOnRoad = {}
endTime = 1000
undefined = 99.99 # Free pickup point

def dfFunc_normal():
    Global.service_t = np.random.normal(loc=Global.t_s)

def dfFunc_exp():
    Global.service_t = np.random.exponential(Global.t_s)

def caller(func):
    func()

def init_start(N, v, d, func):
    global current_code, conflicts
    conflicts = set()
    for idx in range(N):
        current_code = idx + 1
        caller(func)
        remainingServiceTime[idx] = [Global.service_t, current_code]

    return remainingServiceTime

# Start car parking simulation
# Assumption: Every car starts from 0
def simulate(N, lenRoad, d, timeStep, v, func):
    global conflicts, current_code

    remainingServiceTime = init_start(N, v, d, func)
    currentTime = 0
    conflicts = set()
    while True:
        # End simulation after endTime
        if currentTime >= endTime:
            #print("here", len(conflicts))
            return current_code-len(conflicts)
        # Time step
        currentTime += timeStep
        # Update car distances
        for k in carsOnRoad.keys():
            # If car available
            if k != 0:
                try:
                    carsOnRoad[k][0] += v * timeStep
                except:
                    pass
            # If car has passenger
            else:
                if k in carsOnRoad.keys():
                    for car in carsOnRoad[k]:
                        if type(car)==list:
                            car[0] += v * Global.t
                        else:
                            carsOnRoad[k].remove(car)
                else:
                    pass

        for k in remainingServiceTime.keys():
            if type(remainingServiceTime[k][0])==float:
                # Reduce service time
                remainingServiceTime[k][0] -= Global.t

        Global.l_road = Global.carLen + (Global.n-1) * Global.d_c
        # Remove car
        remove_out_cars()
        # Car would go out and next waiting car come in
        done_service()
        # Next cat start servicing
        caller(func)
        move_cars_to_service(func)

        # Calculate if there would be conflicts
        calculate_conflcts()

# Event: Car finish servicing and leave
def remove_out_cars():
    if 0 not in carsOnRoad.keys():
        return
    else:
        for car in carsOnRoad[0]:
            if type(car)==list:
                if car[0] >= Global.l_road:#当距离大于通道长时
                    # 将车移出通道
                    carsOnRoad[0].remove(car)
        if len(carsOnRoad[0])==0: del(carsOnRoad[0])
    return

# Event: Car start servicing
def move_cars_to_service(func):
    for k in list(carsOnRoad.keys()):
        if k == 0:
            continue
        else:
            v = carsOnRoad[k]
            if v[0] >= Global.carLen + Global.d_c * (k-1):
                del(carsOnRoad[k])
                # Update service time and car number
                caller(func)
                remainingServiceTime[k] = [Global.service_t, v[1]]
    return


# Event: Service donw
def done_service():
    global undefined, current_code
    for k, v in remainingServiceTime.items():
        # v[0] is remaining service time，v[1] is car number
        if v[1] != 0:
            try:
                if v[0] <= 0:
                    # If not car on roads
                    if any_car_on_road() == False:
                        remainingServiceTime[k] = [undefined, 0]
                        current_code += 1
                        carsOnRoad[k] = [0, current_code]# Add a car from starting point

                    else:
                        last_car = undefined
                        # Update kth point as available
                        remainingServiceTime[k] = [undefined, 0]
                        # Car finish servicing passenger
                        if 0 in carsOnRoad.keys():
                            carsOnRoad[0].append([Global.carLen+Global.d_c, v[1]])
                        else:
                            carsOnRoad[0] = [ [Global.carLen+Global.d_c, v[1]] ]
                            # If last car have left
                            if last_car <= 0:
                                current_code += 1
                                carsOnRoad[k] = [last_car - Global.carLen, current_code]
                            # If last car have left
                            else:
                                current_code += 1
                                # Start from starting point
                                carsOnRoad[k] = [0, current_code]
            except: pass
    return


# Event: Conflict
def calculate_conflcts():
    global conflicts
    heading_cars = []
    leaving_cars = []
    for goal, car in carsOnRoad.items():
        if goal != 0:
            if len(car) > 1: heading_cars.append(car)
        else:
            if len(carsOnRoad[goal]) > 0:
                leaving_cars = carsOnRoad[goal]

    for c1 in heading_cars:
        d1, code1 = c1[0], c1[1]
        for c2 in leaving_cars:
            try:
                d2, code2 = c2[0], c2[1]
                print(d1,d2)
                if abs(d1 - d2) < Global.carLen:
                    conflicts.add(tuple((code1, code2)))
            except:
                pass
    return

def any_car_on_road():
    for k in carsOnRoad.keys():
        if len(carsOnRoad[k]) > 0:
            return True
    return False


def find_opt_val(fname):
    global current_code
    results = []
    for n in range(2, 20):
        Global.l_road = Global.carLen + (n-1) * Global.d_c
        current_code = 0

        goodness = simulate(n, Global.l_road, Global.d_c, Global.t, Global.v, fname)
        results.append(tuple((goodness, n)))
    results.sort(reverse=True)
    return results[0][1]

if __name__ == "__main__":
    print("--------------------------")
    results = []
    for n in range(2, 20):
        Global.l_road = Global.carLen + (n-1) * Global.d_c
        current_code = 0

        dfFunc = dfFunc_normal

        if (len(sys.argv) > 1):
            fname = sys.argv[1]
            if (fname == "Normal"):
                dfFunc = dfFunc_normal
            elif (fname == "Exponential"):
                dfFunc = dfFunc_exp

        goodness = simulate(n, Global.l_road, Global.d_c, Global.t, Global.v, dfFunc)
        print(goodness)
        results.append(tuple((goodness, n)))
    results.sort(reverse=True)
    print(results[0][1])