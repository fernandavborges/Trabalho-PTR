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

# Definições usadas no PyRTOS para mensagens ----------------
DETECTOU_PASSAGEM = 128
LIGOU_VENTOINHA = 129
DESLIGOU_VENTOINHA = 130

# -----------------------------------------------------------

# Path's para objetos e modelos ---------------------

VENTOINHA = '/Ventoinha/Joint_Ventoinha'
CORPO_VENTOINHA = '/Ventoinha'
PA_VENTOINHA = '/Ventoinha/Pa_Ventoinha'
MOSQUITO = '/target'
CORPO_MOSQUITO = '/Mosquito'
SENSOR = '/Proximity_sensor'
BOTAO = '/PushButton'

#---------------------------------------------------------------------------------------------------------

# Iniciando conexão cliente-servidor com o Coppelia ------------------------------------------------------
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print ('Connected to remote API server')
    sim.simxAddStatusbarMessage(clientID, 'Iniciando...', sim.simx_opmode_oneshot_wait)
    print('Para desligar o sistema aperte o botão da armadilha!')
    time.sleep(0.02)

    # Handle dos objetos e modelos -----------------------------------------------------------------------
    [erro, ventoinha] = sim.simxGetObjectHandle(clientID, VENTOINHA, sim.simx_opmode_blocking)
    if(erro != 0):
        print("Erro ao solicitar controle de objeto do Coppelia!")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()
    [erro, corpo_ventoinha] = sim.simxGetObjectHandle(clientID, CORPO_VENTOINHA, sim.simx_opmode_blocking)
    if(erro != 0):
        print("Erro ao solicitar controle de objeto do Coppelia!")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()
    [erro, pa_ventoinha] = sim.simxGetObjectHandle(clientID, PA_VENTOINHA, sim.simx_opmode_blocking)
    if(erro != 0):
        print("Erro ao solicitar controle de objeto do Coppelia!")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()
    [erro, botao] = sim.simxGetObjectHandle(clientID, BOTAO, sim.simx_opmode_blocking)
    if(erro != 0):
        print("Erro ao solicitar controle de objeto do Coppelia!")
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(-1)
        exit()
    sensor = []
    for i in range(0,13):
        [erro, aux] = sim.simxGetObjectHandle(clientID, SENSOR+str(i), sim.simx_opmode_blocking)
        if(erro != 0):
            print("Erro ao solicitar controle de objeto do Coppelia!")
            sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
            sim.simxFinish(-1)
            exit()
        sensor.append(aux)
    mosquito = []
    for i in range(0,6):
        [erro, aux] = sim.simxGetObjectHandle(clientID, MOSQUITO+"["+str(i)+"]", sim.simx_opmode_blocking)
        if(erro != 0):
            print("Erro ao solicitar controle de objeto do Coppelia!")
            sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
            sim.simxFinish(-1)
            exit()
        mosquito.append(aux)
    corpo_mosquito = []
    for i in range(0,6):
        [erro, aux] = sim.simxGetObjectHandle(clientID, CORPO_MOSQUITO+str(i), sim.simx_opmode_blocking)
        if(erro != 0):
            print("Erro ao solicitar controle de objeto do Coppelia!")
            sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
            sim.simxFinish(-1)
            exit()
        corpo_mosquito.append(aux)


    # ----------------------------------------------------------------------------------------------------


    # Definição das task's -------------------------------------------------------------------------------
    
    def mosquitos(self):
        ### Setup code here
        posicao = [0.0, 0.0, 0.0]

        ### End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:

            ### Work code here

            # Primeiro teste
            
            [erro, posicao] = sim.simxGetObjectPosition(clientID, pa_ventoinha, -1, sim.simx_opmode_blocking)
            #print("posicao ventoinha: " + str(posicao))
            sim.simxSetObjectPosition(clientID, mosquito[0], -1, posicao, sim.simx_opmode_oneshot_wait)
            yield [pyRTOS.timeout(15)]
            
            
            # Segundo teste
            posicao = [7.0, 2.0, 1.0]
            sim.simxSetObjectPosition(clientID, mosquito[1], -1, posicao, sim.simx_opmode_oneshot_wait)
            yield [pyRTOS.timeout(5)]
            
            posicao = [8.0, 4.0, 1.0]
            sim.simxSetObjectPosition(clientID, mosquito[1], -1, posicao, sim.simx_opmode_oneshot_wait)
            yield [pyRTOS.timeout(15)]

            # Terceiro teste
            [erro, posicao] = sim.simxGetObjectPosition(clientID, pa_ventoinha, -1, sim.simx_opmode_blocking)
            sim.simxSetObjectPosition(clientID, mosquito[2], -1, posicao, sim.simx_opmode_oneshot_wait)
            sim.simxSetObjectPosition(clientID, mosquito[3], -1, posicao, sim.simx_opmode_oneshot_wait)
            yield [pyRTOS.timeout(15)]

            # Quarto teste
            posicao = [9.0, 2.5, 1.0]
            sim.simxSetObjectPosition(clientID, mosquito[4], -1, posicao, sim.simx_opmode_oneshot_wait)
            yield [pyRTOS.timeout(3)]
            
            posicao = [7.0, 1.9, 1.0]
            sim.simxSetObjectPosition(clientID, mosquito[5], -1, posicao, sim.simx_opmode_oneshot_wait)
            yield [pyRTOS.timeout(5)]
            
            posicao = [9.0, 2.5, 1.5]
            sim.simxSetObjectPosition(clientID, mosquito[5], -1, posicao, sim.simx_opmode_oneshot_wait)
            
            ### End Work code
            return


    def task_le_sensores(self):
        ### Setup code here

        detectionState = []
        detectedPoint = []
        detectedObjectHandle = []
        detectedSurfaceNormalVector = []

        for i in range(0,13):
            [erro, aux1, aux2, aux3, aux4] = sim.simxReadProximitySensor(clientID, sensor[i], sim.simx_opmode_streaming)
            detectionState.append(aux1)
            detectedPoint.append(aux2)
            detectedObjectHandle.append(aux3)
            detectedSurfaceNormalVector.append(aux4)

        ### End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:

            ### Work code here
            detectou = False
            for i in range(0,13):
                [erro, detectionState[i], detectedPoint[i], detectedObjectHandle[i], detectedSurfaceNormalVector[i]] = sim.simxReadProximitySensor(clientID, sensor[i], sim.simx_opmode_buffer)
                if(erro != 0 and erro != 1):
                    print("Falha na leitura de um dos sensores!")
                    print("Sistema será desligado!")
                    self.send(pyRTOS.Message(pyRTOS.QUIT, self, "desliga_sistema")) 
                if(detectionState[i]):
                    detectou = True
                    break
            if(detectou):
                self.send(pyRTOS.Message(DETECTOU_PASSAGEM, self, "liga_ventoinha", detectou))                             
            ### End Work code

            yield [pyRTOS.timeout(0.1)] 

    def task_liga_ventoinha(self):
        
        ### Setup code here
        tempo_ligamento = 0.0
        tempo_atual = 0.0

        ### End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:

            # Check messages
            
            msgs = self.recv()
            for msg in msgs:

                ### Handle messages 
                if msg.type == DETECTOU_PASSAGEM:

                    ### Tear down code here
                    sim.simxSetJointTargetVelocity(clientID, ventoinha, 5.0, sim.simx_opmode_oneshot)
                    tempo_ligamento = time.time() # pega a hora que ele foi ligado
                    self.send(pyRTOS.Message(LIGOU_VENTOINHA, self, "vento_mata_mosquito"))
                    ### End of Tear down code

                ### End Message Handler

            ### Work code here
            tempo_atual = time.time()
            if((tempo_atual - tempo_ligamento) > 10): # 10 segundos se passaram desde o desligamento
                sim.simxSetJointTargetVelocity(clientID, ventoinha, 0.0, sim.simx_opmode_oneshot)
                self.send(pyRTOS.Message(DESLIGOU_VENTOINHA, self, "vento_mata_mosquito"))
            ### End Work code

            yield [pyRTOS.wait_for_message(self), pyRTOS.timeout(10.0)]

    def task_desliga_sistema(self):
        ### Setup code here
        button = sim.simxGetInt32Signal(clientID, "myButton", sim.simx_opmode_streaming)
        ### End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:

            # Check messages
            msgs = self.recv()
            for msg in msgs:

                ### Handle messages 
                if msg.type == pyRTOS.QUIT:
                    ### Tear down code here
                    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
                    sim.simxAddStatusbarMessage(clientID, 'Sistema interrompido!', sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, ventoinha, 0.0, sim.simx_opmode_oneshot)
                    print("Sistema desligado!")
                    print("Terminando a comunicação com o Coppelia!")
                    sim.simxFinish(-1)
                    exit()
                    ### End of Tear down code

            ### Work code here
            button = sim.simxGetInt32Signal(clientID, "myButton", sim.simx_opmode_buffer)
            if(button == (0, 1)):
                sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
                sim.simxAddStatusbarMessage(clientID, 'Sistema interrompido!', sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID, ventoinha, 0.0, sim.simx_opmode_oneshot)
                print("Sistema desligado!")
                print("Terminando a comunicação com o Coppelia!")
                sim.simxFinish(-1)
                exit()

            ### End Work code

            yield [pyRTOS.timeout(0.5)]

    def vento_mata_mosquito(self):
        ### Setup code here
        ventoinha_ligada = False

        ### End Setup code

        # Pass control back to RTOS
        yield

        # Thread loop
        while True:
            
            # Check messages
            msgs = self.recv()
            
            for msg in msgs:
                
                ### Handle messages 
                if msg.type == LIGOU_VENTOINHA:  
                    ### Tear down code here
                    ventoinha_ligada = True

                if msg.type == DESLIGOU_VENTOINHA:  

                    ### Tear down code here
                    ventoinha_ligada = False
                ### End Message Handler
            yield
            ### Work code here
            if ventoinha_ligada == True:
                checa_colisao = 100
                for i in range(0,6):
                    #print(str(i) + "colisao: " + str(checa_colisao))
                    [error, checa_colisao] = sim.simxCheckDistance(clientID, pa_ventoinha, corpo_mosquito[i], sim.simx_opmode_oneshot_wait)
                    yield
                    #print(str(i) + "colisao: " + str(checa_colisao))

                    if(checa_colisao < 0.5):
                        # matando o mosquito:
                        sim.simxRemoveModel(clientID, corpo_mosquito[i], sim.simx_opmode_oneshot_wait)
                        sim.simxRemoveObject(clientID, mosquito[i], sim.simx_opmode_oneshot_wait)
                        #print(str(i) + "colidiu!")

            ### End Work code

            yield [pyRTOS.timeout(1)] 

    # ------------------------------------------------------------------------------

    # Adicionando as tasks -----------------------------------------------------------
    pyRTOS.add_task(pyRTOS.Task(task_le_sensores, priority=1, name="le_sensores", notifications=None, mailbox=False))
    pyRTOS.add_task(pyRTOS.Task(task_liga_ventoinha, priority=2, name="liga_ventoinha", notifications=None, mailbox=True))
    pyRTOS.add_task(pyRTOS.Task(task_desliga_sistema, priority=3, name="desliga_sistema", notifications=None, mailbox=True))


    pyRTOS.add_task(pyRTOS.Task(vento_mata_mosquito, priority=5, name="vento_mata_mosquito", notifications=None, mailbox=True))
    pyRTOS.add_task(pyRTOS.Task(mosquitos, priority=4, name="teste_mosquitos", notifications=None, mailbox=False))

    # -------------------------------------------------------------------------------

    # Play no RTOS
    pyRTOS.start()
    
    # Now close the connection to CoppeliaSim --------------------------------------
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxAddStatusbarMessage(clientID, 'Finalizando...', sim.simx_opmode_blocking)
    sim.simxFinish(-1)
else:
    sim.simxFinish(-1)
    print ('Failed connecting to remote API server')
print ('Program ended')
