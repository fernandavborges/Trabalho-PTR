# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

VENTOINHA = '/Cuboid/Joint_Ventoinha'
PA_VENTOINHA = 'Cuboid/Pa_Ventoinha'
MOSQUITO = '/target'

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19998,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print ('Connected to remote API server')
    sim.simxAddStatusbarMessage(clientID, 'Iniciando...', sim.simx_opmode_oneshot_wait)
    time.sleep(0.02)

    # ------------------------------------------------

    [erro, ventoinha] = sim.simxGetObjectHandle(clientID, VENTOINHA, sim.simx_opmode_blocking)
    [erro, mosquito] = sim.simxGetObjectHandle(clientID, MOSQUITO, sim.simx_opmode_blocking)
    [erro, pa_ventoinha] = sim.simxGetObjectHandle(clientID, PA_VENTOINHA, sim.simx_opmode_blocking)

    sim.simxSetJointTargetVelocity(clientID, ventoinha, 2.0, sim.simx_opmode_oneshot)

    #Pega posição da ventoinha
    posicao = [9.0, 2.5, 1.0]

    sim.simxSetObjectPosition(clientID, mosquito, -1, posicao, sim.simx_opmode_oneshot)

    time.sleep(30)
    
    # Now close the connection to CoppeliaSim ------------------
    sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxAddStatusbarMessage(clientID, 'Finalizando...', sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
