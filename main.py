import traci
import traci.constants as tc
import os, sys
import random
import numpy as np
from math import *

INTERSECTION_COORDINATE_X = 83.18
INTERSECTION_COORDINATE_Y = 148.72

## ------------------Starting SUMO simulator gui version----------------------------
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'.")
sumoCmd = ['sumo-gui', "-c", "intersection.sumocfg"]
traci.start(sumoCmd)
## -----------------SUMO starting parts end and algorithms starts-------------------------

veh_queue_route0 = []
veh_queue_route1 = []
veh_queue_route2 = []
veh_queue_route3 = []

def addVehicleToQueue(veh_no, route_no):
    print("route_no:", route_no)
    if route_no == 0:
        veh_queue_route0.append(str(veh_no))
    if route_no == 1:
        veh_queue_route1.append(str(veh_no))
    if route_no == 2:
        veh_queue_route2.append(str(veh_no))
    if route_no == 3:
        print(veh_no)
        veh_queue_route3.append(str(veh_no))
        print(veh_queue_route3)

def checkForExitedVehicles():
    if len(veh_queue_route0) != 0:
        if traci.vehicle.getPosition(str(veh_queue_route0[0]))[0] > 87.12:
            veh_queue_route0.pop(0)
    if len(veh_queue_route1) != 0:
        if traci.vehicle.getPosition(str(veh_queue_route1[0]))[0] > 90.88:
            veh_queue_route1.pop(0)
    if len(veh_queue_route2) != 0:
        if traci.vehicle.getPosition(str(veh_queue_route2[0]))[0] < 80.34:
            print("Here")
            # veh_queue_route2.pop(0)
    if len(veh_queue_route3) != 0:
        if traci.vehicle.getPosition(str(veh_queue_route3[0]))[0] < 76.95:
            print("Here")
            # veh_queue_route3.pop(0)

def getTOA(veh_ID):
    x, y = traci.vehicle.getPosition(veh_ID)
    a = traci.vehicle.getAccel(veh_ID)
    v = traci.vehicle.getSpeed(veh_ID)
    d = sqrt((INTERSECTION_COORDINATE_Y-y)**2 + (INTERSECTION_COORDINATE_X-x)**2)
    toa = (sqrt(v*v + 4*a*d) - v)/(2*a)
    return toa

def advantageArray():

    dict = {'0': [],
            '1': [],
            '2': [],
            '3': []}

    routesWhereVehicleExist = []

    if len(veh_queue_route0)!=0:
        dict['0'].append(str(veh_queue_route0[0]))
        dict['0'].append(getTOA(str(veh_queue_route0[0])))
        routesWhereVehicleExist.append(0)
    if len(veh_queue_route1)!=0:
        dict['1'].append(str(veh_queue_route1[0]))
        dict['1'].append(getTOA(str(veh_queue_route1[0])))
        routesWhereVehicleExist.append(1)
    if len(veh_queue_route2)!=0:
        dict['2'].append(str(veh_queue_route2[0]))
        dict['2'].append(getTOA(str(veh_queue_route2[0])))
        routesWhereVehicleExist.append(2)
    if len(veh_queue_route3)!=0:
        dict['3'].append(str(veh_queue_route3[0]))
        dict['3'].append(getTOA(str(veh_queue_route3[0])))
        routesWhereVehicleExist.append(3)

    advantage_array = []   

    for _ in routesWhereVehicleExist:
        temp = -1
        min = inf
        for i in routesWhereVehicleExist:
            if dict[str(i)][1] < min:
                print(dict[str(i)][1])
                temp =  i
                min = dict[str(i)][1]
                dict[str(i)][1] = inf
        advantage_array.append(temp)
    print("advantage_array: ",advantage_array)
    return advantage_array

# def enforceRSS(advantage_array):


step = 0
i = 0
flag = True
PDG = {}
leaderNotDefined = True
leaders = [0, 0, 0, 0]

while(step < 100000 and flag):
    traci.simulationStep()

    ## Vehicle addition block, and identify leaders of the particular routes.
    if step%100 == 0:
        route_no = random.randint(0, 3)
        traci.vehicle.add(str(step/100), str(route_no), typeID='vtypeauto')
        traci.vehicle.setSpeedMode(str(step/100), 32)
        addVehicleToQueue(step/100, route_no)
    checkForExitedVehicles()
    # print("route 0: ", veh_queue_route0)
    # print("route 1: ", veh_queue_route1)
    # print("route 2: ", veh_queue_route2)
    # print("route 3: ", veh_queue_route3)
    # advantage_array = advantageArray()
    # enforceRSS(advantage_array)
    if len(veh_queue_route3) != 0:
        print(traci.vehicle.getPosition(str(veh_queue_route3[0])))

    # -------------- COLLISION BLOCK----------------------------
    ## Terminate the program if a collision occurs.
    if len(traci.simulation.getCollisions()) !=0:
        print(traci.simulation.getCollisions())
        print("Terminating on collision............")
        flag = False
    # -----------------------------------------------------------
        
    step += 1





    
