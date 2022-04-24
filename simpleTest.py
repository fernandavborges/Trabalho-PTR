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
PA_VENTOINHA = '/Cuboid/Pa_Ventoinha'
MOSQUITO_BASE = '/Quadcopter'
MOSQUITO_MIRA = '/target'

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print ('Connected to remote API server')
    sim.simxAddStatusbarMessage(clientID, 'Iniciando...', sim.simx_opmode_oneshot_wait)
    time.sleep(0.02)

    # ------------------------------------------------

"""
    # tenta carregar novo modelo (ainda nao funciona)
    [error, modelobase] = sim.simxLoadModel(clientID, "./Quadcopter.ttm", 0, sim.simx_opmode_blocking)
    print(error)
    input()

    [error, modelobase] = sim.simxCopyPasteObjects(clientID, modelobase, sim.simx_opmode_blocking)
    print(error)
    input()
"""

    [erro, ventoinha] = sim.simxGetObjectHandle(clientID, VENTOINHA, sim.simx_opmode_blocking)
    if(erro != 0):
        print("erro handle ventoinha " + erro)
    print('vrau')
    input()
    
    [erro, pa_ventoinha] = sim.simxGetObjectHandle(clientID, PA_VENTOINHA, sim.simx_opmode_blocking)
    if(erro != 0):
        print("erro handle pas ventoinha " + erro)
    print('vrau')
    input()
    
    [erro, mosquito] = sim.simxGetObjectHandle(clientID, MOSQUITO_BASE, sim.simx_opmode_blocking)
    if(erro != 0):
        print("erro handle mosquito base " + erro)
    print('vrau')
    input()
    
    [erro, mosquitom] = sim.simxGetObjectHandle(clientID, MOSQUITO_MIRA, sim.simx_opmode_blocking)
    if(erro != 0):
        print("erro handle mosquito mira " + erro)
    print('vrau')
    input()
    
    # liga ventoinha
    print('vrau')
    sim.simxSetJointTargetVelocity(clientID, ventoinha, 8, sim.simx_opmode_oneshot)

    print('vrau')
    
    #Pega posição das pas da ventoinha
    [error, posicao] = sim.simxGetObjectPosition(clientID, pa_ventoinha, -1, sim.simx_opmode_blocking)
    print(posicao)
    
    # atrai mosquito
    sim.simxSetObjectPosition(clientID, mosquitom, -1, posicao, sim.simx_opmode_oneshot_wait)
    
    while 1==1:
        #checa colisao
        [error, checa_colisao] = sim.simxCheckDistance(clientID, pa_ventoinha, mosquito, sim.simx_opmode_blocking)
        if(erro != 0):
            print("erro checa colisao")
            
        print(checa_colisao)

        if(checa_colisao < 0.2):
        # matando o mosquito:
            sim.simxRemoveModel(clientID, mosquito, sim.simx_opmode_oneshot_wait)
            sim.simxRemoveObject(clientID, mosquitom, sim.simx_opmode_oneshot_wait)
            print("colidiu!")
            break
    
    input()

    # Now close the connection to CoppeliaSim ------------------
    sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxAddStatusbarMessage(clientID, 'Finalizando...', sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
input()
