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

# Bibliotecas importadas -----------------------------
import time
import pyRTOS


# ---------------------------------------------------

# Definição das task's -----------------------------
def sample_task(self):
    ### Setup code here


    ### End Setup code

    # Pass control back to RTOS
    yield

    # Thread loop
    while True:

        ### Work code here
        


        ### End Work code

        yield [pyRTOS.timeout(0.5)] # timeout -> tempo mínimo entre uma chama e outra da task (em segundos)

def task_le_sensores(self):
    ### Setup code here


    ### End Setup code

    # Pass control back to RTOS
    yield

    # Thread loop
    while True:

        ### Work code here
        


        ### End Work code

        yield [pyRTOS.timeout(0.5)] 



# ---------------------------------------------------

# Path's para objetos e modelos ---------------------

VENTOINHA = '/Cuboid/Joint_Ventoinha'
CORPO_VENTOINHA = '/Cuboid'
PA_VENTOINHA = 'Cuboid/Pa_Ventoinha'
MOSQUITO = '/target'
CORPO_MOSQUITO = '/Mosquito'

#----------------------------------------------------

# Iniciando conexão cliente-servidor com o Coppelia --------
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19998,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print ('Connected to remote API server')
    sim.simxAddStatusbarMessage(clientID, 'Iniciando...', sim.simx_opmode_oneshot_wait)
    time.sleep(0.02)

    # ------------------------------------------------

    # Handle dos objetos e modelos -----------------------------------------------------------------------
    [erro, ventoinha] = sim.simxGetObjectHandle(clientID, VENTOINHA, sim.simx_opmode_blocking)
    [erro, corpo_ventoinha] = sim.simxGetObjectHandle(clientID, CORPO_VENTOINHA, sim.simx_opmode_blocking)
    [erro, mosquito] = sim.simxGetObjectHandle(clientID, MOSQUITO, sim.simx_opmode_blocking)
    [erro, pa_ventoinha] = sim.simxGetObjectHandle(clientID, PA_VENTOINHA, sim.simx_opmode_blocking)
    [erro, corpo_mosquito] = sim.simxGetObjectHandle(clientID, CORPO_MOSQUITO, sim.simx_opmode_blocking)

    # ----------------------------------------------------------------------------------------------------



    #Funções que vao ser usadadas dentro das tasks

    sim.simxSetJointTargetVelocity(clientID, ventoinha, 2.0, sim.simx_opmode_oneshot)

    # Posição da ventoinha
    posicao = [9.0, 2.5, 1.0]

    sim.simxSetObjectPosition(clientID, mosquito, -1, posicao, sim.simx_opmode_oneshot)


    time.sleep(5)
    # Checar colisão entre mosquito e a ventoinha
    for i in range (0, 100000):
        [erro, colisao] = sim.simxCheckDistance(clientID, corpo_mosquito, corpo_ventoinha, sim.simx_opmode_buffer)
        if(colisao < 0.5):
            sim.simxRemoveModel(clientID, corpo_mosquito, sim.simx_opmode_oneshot)
            print('detecteeeei')
    
    time.sleep(30)
    # ------------------------------------------------------------------------------

    # Adcionando as tasks -----------------------------------------------------------
    #pyRTOS.add_task(pyRTOS.Task(task, priority=, came=, notification=, mailbox=))

    # -------------------------------------------------------------------------------

    # Play no RTOS
    #pyRTOS.start()
    
    # Now close the connection to CoppeliaSim --------------------------------------
    sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxAddStatusbarMessage(clientID, 'Finalizando...', sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
