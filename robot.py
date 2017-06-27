from __future__ import division
from PIL import Image as I
from math import sin, cos, pi
from busca.Graph import Comando
from busca import Path
import vrep,array,time,sys,random
from processamento.DetetorDeProduto import DetetorDeProduto


import threading

R = 0.097;      # raio da roda em m
L = 0.381;      # distancia entre as 2 rodas em m
PI = 3.14159265359

DEBUG = True
VELOCIDADE_RODA = 3

data = []


class Robot:
    """Classe robo do V-REP"""
    handle = 0                  # robot handle
    encoderHandle = [0,0]       # left and right encoder handlers
    motorHandle = [0,0]         # left and right motor handlers
    encoder = [0,0]
    lastEncoder = [0,0]
    angularDiff = [0,0]
    gyro = 0
    queue = []
    sonarHandle = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]      # 16 sonar handlers
    sonarReading = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]
    robotPosition = []
    robotOrientation = []
    visionSensorHandles=[0,0,0,0]
    blackVisionReading=[False,False,False]
    redVisionReading=[False,False,False]
    pos = '1D'
    comandos=[]
    corredor=2
    faixaASeguir=0
    countFaixas=0
    bifurcacao = False
    entrar = False
    countdown = 0
    sobreBifurcacao = False
    i=-1
    andaRetoCount = 0
    entrarDireita = False
    entrarEsquerda = False
    distanceAfterRedMarker = 0
    stoppingAtRedMarker = False
    vrepLastTime = 0
    vrepDT = 0
    targetOrientation = 0
    rotating = False
    test = 0
    count = 0

    comandoAtual = Comando.NONE
    ignoraProximaBifurcacao = False

    destino = ""

    pose = [0,0,PI/2]          # [x,y,teta]

    andaRetoCount = 0;

    def __init__(self, clientID, name):

        print "initializing robot... "

        self.clientID = clientID
        self.name = name
        self.faixaASeguir = self.corredor*2-1

        # Get robot handle
        _,self.handle = vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking);

        # Get handles of sensors and actuators
        _,self.encoderHandle[0] = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftWheel", vrep.simx_opmode_oneshot_wait)
        _,self.encoderHandle[1] = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightWheel", vrep.simx_opmode_oneshot_wait)

        #  Get handles of sensors and actuators
        _,self.motorHandle[0] = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait)
        _,self.motorHandle[1] = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor",vrep.simx_opmode_oneshot_wait)

        # Vision handles
        _,self.visionSensorHandles[0]=vrep.simxGetObjectHandle(clientID, "Camera_Faixa_Esq", vrep.simx_opmode_oneshot_wait)
        _,self.visionSensorHandles[1]=vrep.simxGetObjectHandle(clientID, "Camera_Faixa_Meio", vrep.simx_opmode_oneshot_wait)
        _,self.visionSensorHandles[2]=vrep.simxGetObjectHandle(clientID, "Camera_Faixa_Dir", vrep.simx_opmode_oneshot_wait)
        _,self.visionSensorHandles[3]=vrep.simxGetObjectHandle(clientID, "Camera_Gondola", vrep.simx_opmode_oneshot_wait)

        for i in range(16):
            sensorName = "Pioneer_p3dx_ultrasonicSensor" + str(i+1)
            r,self.sonarHandle[i] = vrep.simxGetObjectHandle(clientID, sensorName, vrep.simx_opmode_oneshot_wait)
            if r != vrep.simx_return_ok:
                print "Error on connecting to ultrasonicSensor " + str(i+1)
            # else:


                # print "Connected to ultrasonicSensor " + str(i+1)

                # rob.queueAdd(rob.rollSection())   < insere uma nova secao na fila de locais
                #                                   < que devem ser visitados
                #
                # section = rob.queueGetFirst()     < extrai a proxima secao da fila
                #
                # product = rob.rollProduct(section) < sorteia um produto a ser analisado.
                #                                    < product eh uma tupla (COR, QUANTIDADE)
                #                                    < que determina quantos produtos de cor
                #                                    < COR devem estar presentes na secao


        self.sorteiaComandos()

    def sorteiaDestino(self):
        self.destino = self.pos
        while self.destino == self.pos:
            self.destino = self.rollSection()
            print "DESTINO : " + self.destino

    def sorteiaComandos(self):
        self.sorteiaDestino()
        self.pPlanner = Path.PathPlanner()
        self.debug("initial pos = " + str(self.pos))
        self.pos, self.comandos = self.pPlanner.getPath(self.pos, self.destino)

        if self.comandos[0] == Comando.ESQ or self.comandos[0] == Comando.DIR:
            self.comandos.insert(0, Comando.RETO)

        elif self.comandos[0] == Comando.ROT and (self.comandos[1] == Comando.ESQ or self.comandos[1] == Comando.DIR ):
            self.comandos.insert(1, Comando.RETO)

        print "comandos = " + str(self.comandos)
        print "finalPos = " + str(self.pos)
        self.i = -1
        self.stoppingAtRedMarker = False



    def resetFaixaASeguir(self):
        self.debug("RESET FAIXA")
        self.faixaASeguir = 2;

    def run(self):
        # print "dt = "+str(self.vrepDT)

        # Get the robot current absolute position
        _,self.robotPosition = vrep.simxGetObjectPosition(self.clientID, self.handle,-1,vrep.simx_opmode_oneshot_wait);
        _,self.robotOrientation = vrep.simxGetObjectOrientation(self.clientID, self.handle,-1,vrep.simx_opmode_oneshot_wait);

        # if self.firstRun:
        #     pose = [self.robotPosition[0], self.robotPosition[1],self.robotOrientation[2]]
        #     firstRun = False

        self.updateEncoders()
        dist = self.distanceForward()
        self.updatePose()
        # print "\n-----------------\nangularDiff = " + str(self.angularDiff)+" dist = "+str(dist)+"\n pos = "+str(self.robotPosition)+"\n------------------\n"

        #print "-------------------------------------------------"
        #print "robotPosition = " + str(self.robotPosition)
        # print "robotOrientation = " + str(self.robotOrientation[2])
        self.readGyro()

        self.readSonars()
        fator = self.getVelocityFactor()
        # print "FACTOR"
        # print fator
        self.readVision()
        #vLeft, vRight = self.avoidObstacle()
        self.bifurcacao = self.checkBifurcacao()

        if len(self.queue) == 0 and len(self.comandos) == 0:
            self.move(0,0)
            return


        if self.i == -1 or (self.bifurcacao and not self.sobreBifurcacao):

            if self.i == -1:
                self.debug("PRIMEIRO COMANDO !")

            if self.ignoraProximaBifurcacao:
                self.ignoraProximaBifurcacao = False
                self.sobreBifurcacao = True
                self.comandoAtual = Comando.RETO
                self.countdown = 5

            else:
                self.i+=1
                self.comandoAtual = self.comandos[self.i]

                self.faixaASeguir-=1
                self.sobreBifurcacao = True
                self.debug("BIFURCACAO " + str(self.bifurcacao))
                self.debug("COMANDO " + str(self.i) + " " + str(self.comandoAtual))



                if self.comandoAtual == Comando.ROT_CAM:
                    self.debug("ROTATE CAMERA")
                    self.rotateCamera()
                    self.i += 1
                    self.comandoAtual = self.comandos[self.i]
                    self.countdown = 1

                if self.comandoAtual == Comando.ROT:
                    self.debug("ROTATION")
                    self.rotate180()
                    self.i += 1
                    self.comandoAtual = self.comandos[self.i]
                    self.countdown = 1

                elif self.comandoAtual == Comando.ESQ:
                    self.debug("ESQUERDA")
                    self.andaRetoCount = 0
                    self.andaRetoCount = 0
                elif self.comandoAtual == Comando.RETO:
                    if True in self.redVisionReading or self.i == 0 or self.comandos[self.i-1] == Comando.ROT:
                        self.debug("RETO RED")
                        self.countdown = 5
                        self.comandoAtual = Comando.RETO
                    else:
                        self.debug("RETO NORMAL")
                        self.countdown =  5
                        self.ignoraProximaBifurcacao = True
                elif self.comandoAtual == Comando.DIR:
                    self.debug("DIREITA")
                    self.andaRetoCount = 0
                elif self.comandoAtual == Comando.FOTO:
                    self.debug( "TAKE PICTURE" )
                    self.countdown = 5
                    self.distanceAfterRedMarker = 0
                    self.stoppingAtRedMarker = True


        elif not self.bifurcacao:
            self.countdown -=1
        if self.countdown==0 and self.comandoAtual == Comando.RETO:
            self.debug("ACABOU A BIFURCACAO #1")
            self.sobreBifurcacao = False
            self.comandoAtual = Comando.NONE
        vLeft, vRight = self.followLine()
        self.move(fator*vLeft, fator*vRight)

        if self.rotating:
            #self.debug("___ rotating "+str(self.rotating))
            vLeft, vRight = self.rotate180()
            self.move(vLeft, vRight)
        else:
            vLeft, vRight = self.followLine()
            self.move(fator*vLeft, fator*vRight)

        # ## salvar dados de localizacao em arquivo para plotar
        #
        # if self.count > 4 and self.count < 1000:
        #     self.count += 1
        #     print "count = "+str(self.count)
        # elif self.count == 1000:
        #     self.count += 1
        #     self.writeFile(data)




    def followLine(self):
        if self.comandoAtual == Comando.ROT and not self.rotating:
            return self.rotate180()

        if self.stoppingAtRedMarker:
            #self.debug("DISTANCE FORWARD " + str(self.distanceForward()))
            if abs(self.distanceForward()) < 0.005:
                time.sleep(0.1) # espera o tranco
                self.takePicture("Camera_Gondola")
                self.comandoAtual = Comando.NONE
                self.sobreBifurcacao = False
                return 1,1

            if self.distanceForward() == -1:
                self.distanceAfterRedMarker = self.distanceAfterRedMarker + 0.02
            else:
                self.distanceAfterRedMarker = self.distanceAfterRedMarker + self.distanceForward()
            #self.debug("ANDANDO PARA PARAR NA FAIXA " + str(self.distanceAfterRedMarker))
            if self.distanceAfterRedMarker > 0.9:
                self.debug("ANDOU 0.5")
                return 0,0


        if self.comandoAtual==Comando.RETO:
            return VELOCIDADE_RODA,VELOCIDADE_RODA
        if self.blackVisionReading[2] and not self.comandoAtual == Comando.ESQ:#direita
            self.andaRetoCount = 0
            return 2,1
        if self.blackVisionReading[0] and not self.comandoAtual == Comando.DIR:#esquerda
            self.andaRetoCount = 0
            return 1,2

        self.andaRetoCount += 1

        if self.andaRetoCount ==20 and (self.comandoAtual == Comando.ESQ or self.comandoAtual == Comando.DIR):
            self.debug("ACABOU A BIFURCACAO #2")
            self.sobreBifurcacao = False
            self.comandoAtual = Comando.NONE

        return VELOCIDADE_RODA,VELOCIDADE_RODA


    def move(self, leftMotorVelocity, rightMotorVelocity):
        vrep.simxSetJointTargetVelocity(self.clientID, self.motorHandle[0], leftMotorVelocity, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(self.clientID, self.motorHandle[1], rightMotorVelocity, vrep.simx_opmode_oneshot);

    def readSonars(self):
        # Reads sonar's current value
        # if state == 0 then nothing detected and sonar value is -1
        # otherwise sonar value that matter is only the third coordinate returned
        for i in range(16):
            _,detectedState,self.sonarReading[i],_,_ = vrep.simxReadProximitySensor(self.clientID,self.sonarHandle[i],vrep.simx_opmode_streaming);
            self.sonarReading[i] = self.sonarReading[i][2] if detectedState > 0 else -1
            #print "sonarReading["+str(i)+"] = "+str(self.sonarReading[i])
        #print "-------------------------------------------------\n"

    def readVision(self):
        for i in range (3):
            err,detectedState,data=vrep.simxReadVisionSensor(self.clientID,self.visionSensorHandles[i], vrep.simx_opmode_streaming)
            if len(data) > 0:
                self.blackVisionReading[i]=(data[0][10]<0.1)    # !!!data[10]!!! is the average of intensity of the image
                                                                # TRUE: sensor esta sobre a linha preta
                #print 'avg camera '+str(i)+' = ' + str(self.blackVisionReading[i])
                self.redVisionReading[i] = (data[0][6] > 0.85)   # True: sensor captou vermelho
        # print 'max red = ' + str(self.redVisionReading)

    def rotate180(self):
        if not self.rotating:
            self.test = 1
            self.rotating = True
            if abs(self.pose[2] - PI/2) < 0.2:
                self.targetOrientation = -PI/2
            else:
                self.targetOrientation = PI/2

        if abs(self.pose[2] - self.targetOrientation) > 0.1:
            #self.debug("### precisa chega no 0 --> "+str(abs(self.pose[2] - self.targetOrientation)))
            return 1,-1
        else:
            self.debug("chegou!!!")
            self.rotating = False
            return 0,0


    def updatePose(self):
        if (self.count < 5):
            self.pose = [self.robotPosition[0],self.robotPosition[1],self.robotOrientation[2]]
            self.count += 1
        else:
            vLeft = self.angularDiff[0]*R
            vRight = self.angularDiff[1]*R
            dS = (vLeft+vRight)/2

            dTeta = self.gyro*self.vrepDT

            dX = dS*cos(self.pose[2]+dTeta/2);
            dY = dS*sin(self.pose[2]+dTeta/2);

            self.pose[0] += dX;
            self.pose[1] += dY;

            self.pose[2] += dTeta;
            if self.pose[2] > PI:
                self.pose[2] = -PI+(self.pose[2]-PI);
            elif self.pose[2] < -PI:
                self.pose[2] = PI-(self.pose[2]+PI);
        data.append((self.pose[0],self.pose[1],self.robotPosition[0],self.robotPosition[1]))
        # print "------------------------------------------------"
        # print "> gt = "+str((self.robotPosition[0],self.robotPosition[1]))+" "+str(self.robotOrientation[2])
        # print "> odo= "+str(self.pose)
        # print "------------------------------------------------"



    def readGyro(self):
        _,self.gyro = vrep.simxGetFloatSignal(self.clientID, 'gyroZ', vrep.simx_opmode_streaming)
        t = vrep.simxGetLastCmdTime(self.clientID)/1000
        self.vrepDT = (t - self.vrepLastTime)
        self.vrepLastTime = t
        # dTeta = (self.angularDiff[1]*R-self.angularDiff[0]*R)/L;
        # self.pose[2] += self.gyro*self.vrepDT # dTeta # self.gyro*self.vrepDT
        # if (self.pose[2] > PI):
        #     self.pose[2] = -PI+(self.pose[2]-PI)
        # if (self.pose[2] < -PI):
        #     self.pose[2] = PI-(self.pose[2]+PI)
        # print "teta = "+str(self.pose[2])+"\ngyro = "+str(self.gyro)+" dT = "+str(self.vrepDT)
        #print "errOdoGt = "+str((self.pose[0]-self.robotPosition[0], self.pose[1]-self.robotPosition[1], self.pose[2]-self.robotOrientation[2]))

    def getVelocityFactor(self):
        sonars = []
        NEAR_MAX = 0.3
        MEDIUM_MAX = 0.8
        NEAR = 0
        MEDIUM = 1
        FAR = 2
        STOP = 0
        SLOW = 0.4
        FREE = 1
        for i in range(2,6):
            if(self.sonarReading[i] == -1):
                sonars.append(2)
            else:
                sonars.append(self.sonarReading[i])
        # print self.sonarReading
        # print sonars
        frontObstacle = min(sonars[1],sonars[2])
        sideObstacle = min(sonars[0],sonars[3])
        if(frontObstacle >= MEDIUM_MAX):
            frontVal = FAR
        elif(frontObstacle >= NEAR_MAX):
            frontVal = MEDIUM
        else:
            frontVal = NEAR
        if(sideObstacle >= MEDIUM_MAX):
            sideVal = FAR
        elif(sideObstacle >= NEAR_MAX):
            sideVal = MEDIUM
        else:
            sideVal = NEAR

        # RULES
        if(frontVal == FAR and (sideVal == FAR or sideVal == MEDIUM)):
            return FREE
        elif(frontVal == FAR or sideVal == NEAR):
            return STOP
        elif(frontVal == MEDIUM):
            return SLOW
        else:
            return STOP

    def checkBifurcacao(self):
        self.countFaixas = 0
        #print self.blackVisionReading
        if not self.bifurcacao:
            for i in range(3):
                if self.blackVisionReading[i]:
                    self.countFaixas+=1
                if self.countFaixas==2:
                    self.countFaixas = 0
                    return True

            if True in self.redVisionReading:
                self.debug("CHECK BIFURCATION : RED")
                return True

        return False

    def avoidObstacle(self):
        for i in range(2,8):
            if self.sonarReading[i] > -1 and self.sonarReading[i] < 0.4:
                return 1,-1
        return 2,2

    def takePicture(self,visionSensorName):
        print 'Taking picture from ' + visionSensorName + '... '
        res1,visionSensorHandle=vrep.simxGetObjectHandle(self.clientID,visionSensorName,vrep.simx_opmode_oneshot_wait)
        res2,resolution,image=vrep.simxGetVisionSensorImage(self.clientID,visionSensorHandle,0,vrep.simx_opmode_streaming)
        res,resolution,image=vrep.simxGetVisionSensorImage(self.clientID,visionSensorHandle,0,vrep.simx_opmode_buffer)
        time.sleep(0.5)
        res,resolution,image=vrep.simxGetVisionSensorImage(self.clientID,visionSensorHandle,0,vrep.simx_opmode_buffer)
        image_byte_array = array.array('b',image)
        im = I.frombuffer("RGB", (resolution[1],resolution[0]), image_byte_array, "raw", "RGB", 0, 1)
        im = im.rotate(180)
        im = im.transpose(I.FLIP_LEFT_RIGHT)
        im.save('images/' + visionSensorName + '.png', 'png')
        self.debug('done!')

        produto, quantidadeEsperada = self.rollProduct(self.destino)

        #tratar imagem
        dp = DetetorDeProduto()
        produtosAchados = dp.detectColor(produto)
        print "PRODUTO : ", produto
        print "ESPERADOS : ", quantidadeEsperada
        print "ACHADOS : ", produtosAchados

        if float(produtosAchados)/float(quantidadeEsperada) < 0.6:
            print "PRECISA REPOR O PRODUTO ", produto
        else:
            print "EXPOSICAO CONFORME ESPERADO"


        #se a camera foi girada antes de tirar a foto, a gente a reposiciona depois de tirar a foto
        if(self.comandos[self.i - 1] == Comando.ROT_CAM):
            self.rotateCamera()

        self.sobreBifurcacao = False
        self.comandoAtual = Comando.NONE

        self.sorteiaComandos()

    def updateEncoders(self):
        _,self.encoder[0] = vrep.simxGetJointPosition(self.clientID, self.motorHandle[0], vrep.simx_opmode_oneshot);
        _,self.encoder[1] = vrep.simxGetJointPosition(self.clientID, self.motorHandle[1], vrep.simx_opmode_oneshot);
        if self.angularDiff[0] >= 0:
            self.angularDiff[0] = self.encoder[0]-self.lastEncoder[0] if self.encoder[0]*self.lastEncoder[0] >= 0 else 2*PI-self.lastEncoder[0]+self.encoder[0]
        else:
            self.angularDiff[0] = self.encoder[0]-self.lastEncoder[0] if self.encoder[0]*self.lastEncoder[0] >= 0 else self.encoder[0]-self.lastEncoder[0]-2*PI

        if self.angularDiff[1] >= 0:
            # print "~~~~~ => enc1, lastEnc1 = "+str((self.encoder[1],self.lastEncoder[1]))
            self.angularDiff[1] = self.encoder[1]-self.lastEncoder[1] if self.encoder[1]*self.lastEncoder[1] >= 0 else 2*PI-self.lastEncoder[1]+self.encoder[1]
        else:
            self.angularDiff[1] = self.encoder[1]-self.lastEncoder[1] if self.encoder[1]*self.lastEncoder[1] >= 0 else self.encoder[1]-self.lastEncoder[1]-2*PI

        if self.angularDiff[0] > 1:
            self.angularDiff[0] = 0
        if self.angularDiff[1] > 1:
            self.angularDiff[1] = 0


        self.lastEncoder[0] = self.encoder[0]
        self.lastEncoder[1] = self.encoder[1]

    def distanceForward(self):
        if abs(self.angularDiff[0] - self.angularDiff[1]) < 0.01:
            leftDS = self.angularDiff[0]*R
            rightDS = self.angularDiff[1]*R
            dS = (leftDS+rightDS)/2;
            return dS
        else:
            return -1           # robo provavelmente nao esta em linha reta


    def rotateCamera(self):
        vrep.simxSetObjectOrientation(self.clientID, self.visionSensorHandles[3], self.visionSensorHandles[3], [0, pi,0], vrep.simx_opmode_oneshot_wait)
        time.sleep(0.3)


    # funcoes para sortear secao e produto
    # exemplo:
    #
    # rob.queueAdd(rob.rollSection())   < insere uma nova secao na fila de locais
    #                                   < que devem ser visitados
    #
    # section = rob.queueGetFirst()     < extrai a proxima secao da fila
    #
    # product = rob.rollProduct(section) < sorteia um produto a ser analisado.
    #                                    < product eh uma tupla (COR, QUANTIDADE)
    #                                    < que determina quantos produtos de cor
    #                                    < COR devem estar presentes na secao
    def rollProduct(self, section):
        if section == False:
            return False,False

        with open("mapa_produtos.txt") as f:
            for line in f:
                if line.startswith(section):
                    data = line.split(' ')
                    data[-1] = data[-1].replace('\n','');
                    print data
                    break
        i = random.randint(0,(len(data)-1)/2-1)
        color = int(data[1+(2*i)])
        amount = data[1+(2*i)+1]
        return color,amount

    def rollSection(self):
        corridor = random.randint(1,4)
        if 2 <= corridor <= 3:
            section = chr(random.randint(65,70)) # A a F
        elif corridor == 4:
            section = chr(random.randint(65,67)) # A a C
        elif corridor == 1:
            section = chr(random.randint(68,70)) # D a F

        return str(corridor) + section

    def queuePrint(self):
        for s in self.queue:
            print s,
        print ""

    def queueAdd(self, section):
        self.queue.append(section)

    def queueGetFirst(self):
        if len(self.queue) != 0:
            s = self.queue.pop(0)
            return s
        else:
            return False

    # salva dados em arquivo data.log, para plotar:
    # 'gnuplot -p script.plt'
    def writeFile(self,data):
        self.debug("writing data to data.log...")
        file = open("data.log","w")
        for d in data:
            [file.write(str(n)+' ') for n in d]
            file.write('\n')
        file.close()
        self.debug("finished writing file")

    def debug(self, message):
        if DEBUG:
            print message