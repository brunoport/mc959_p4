from PIL import Image as I
import vrep,array,time,sys
import threading

R = 0.097;      # raio da roda em m
L = 0.381;      # distancia entre as 2 rodas em m
PI = 3.14159265359

class Robot:
    """Classe robo do V-REP"""
    handle = 0                  # robot handle
    encoderHandle = [0,0]       # left and right encoder handlers
    motorHandle = [0,0]         # left and right motor handlers
    encoder = [0,0]
    lastEncoder = [0,0]
    angularDiff = [0,0]
    sonarHandle = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]      # 16 sonar handlers
    sonarReading = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]
    robotPosition = []
    robotOrientation = []
    visionSensorHandles=[0,0,0]
    blackVisionReading=[False,False,False]
    redVisionReading=[False,False,False]
    comandos=[1,2,0]
    corredor=2
    faixaASeguir=0
    countFaixas=0
    bifurcacao = False
    entrar = False
    countdown = 0
    sobreBifurcacao = False
    i=0
    andaRetoCount = 0
    entrarDireita = False
    entrarEsquerda = False
    distanceAfterRedMarker = 0
    stoppingAtRedMarker = False

    def __init__(self, clientID, name):
        self.clientID = clientID
        self.name = name
        self.faixaASeguir = self.corredor*2-1
        print "initializing robot... "

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

        for i in range(16):
            sensorName = "Pioneer_p3dx_ultrasonicSensor" + str(i+1)
            r,self.sonarHandle[i] = vrep.simxGetObjectHandle(clientID, sensorName, vrep.simx_opmode_oneshot_wait)
            if r != vrep.simx_return_ok:
                print "Error on connecting to ultrasonicSensor " + str(i+1)
            # else:
                # print "Connected to ultrasonicSensor " + str(i+1)

    def resetFaixaASeguir(self):
        print "RESET FAIXA"
        self.faixaASeguir = 2;

    def run(self):
        # Get the robot current absolute position
        _,self.robotPosition = vrep.simxGetObjectPosition(self.clientID, self.handle,-1,vrep.simx_opmode_oneshot_wait);
        _,self.robotOrientation = vrep.simxGetObjectOrientation(self.clientID, self.handle,-1,vrep.simx_opmode_oneshot_wait);

        self.updateEncoders()
        dist = self.distanceForward()
        print "\n-----------------\nangularDiff = " + str(self.angularDiff)+" dist = "+str(dist)+"\n pos = "+str(self.robotPosition)+"\n------------------\n"

        #print "-------------------------------------------------"
        #print "robotPosition = " + str(self.robotPosition)
        #print "robotOrientation = " + str(self.robotOrientation)

        self.readSonars()
        fator = self.getVelocityFactor()
        print "FACTOR"
        print fator
        self.readVision()
        #vLeft, vRight = self.avoidObstacle()
        self.bifurcacao = self.checkBifurcacao()
        if self.bifurcacao and not self.sobreBifurcacao:
            self.faixaASeguir-=1
            self.sobreBifurcacao = True
            print "BIFURCACAO " + str(self.bifurcacao)
            print "ENTRAR DAQUI " + str(self.faixaASeguir)
            print "COMANDO " + str(self.i) + " " + str(self.comandos[self.i])
            if self.comandos[self.i] == 2:
                print "ENTRAR DIREITA"
                self.entrarDireita = True
                self.entrarEsquerda = False
                self.andaRetoCount = 0
                self.countdown = 30
                self.andaRetoCount = 0
            if self.comandos[self.i] == 0:
                print "ENTRAR ESQUERDA"
                self.entrarEsquerda = True
                self.entrarDireita = False
                self.andaRetoCount = 0
                self.countdown = 30
                self.andaRetoCount = 0
            elif self.comandos[self.i] ==1:
                print "RETO"
                self.entrarEsquerda = True
                self.entrarDireita = True
                self.countdown = 30
            self.i+=1
        elif not self.bifurcacao:
            self.countdown -=1
        if self.countdown==0 and self.entrarDireita and self.entrarEsquerda:
            self.sobreBifurcacao = False
            self.entrarEsquerda = False
            self.entrarDireita = False
        vLeft, vRight = self.followLine()
        self.move(fator*vLeft, fator*vRight)




    def followLine(self):
        if True in self.redVisionReading:
            print "viu vermelho"
            self.stoppingAtRedMarker = True
            self.distanceAfterRedMarker = 0
        if self.entrarEsquerda and self.entrarDireita:
            return 2,2
        if self.blackVisionReading[2] and not self.entrarEsquerda:#direita
            self.andaRetoCount = 0
            return 2,1
        if self.blackVisionReading[0] and not self.entrarDireita:#esquerda
            self.andaRetoCount = 0
            return 1,2

        self.andaRetoCount += 1

        if self.andaRetoCount ==10 and (self.entrarEsquerda or self.entrarDireita):
            self.sobreBifurcacao = False
            self.entrarEsquerda = False
            self.entrarDireita = False
            print "RESET FAIXA"

        return 2,2


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
        print 'max red '+str(i)+' = ' + str(self.redVisionReading)

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
        print self.sonarReading
        print sonars
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
        elif(frontVal == FAR):
            return STOP
        elif(frontVal == MEDIUM):
            return SLOW
        else:
            return STOP

    def checkBifurcacao(self):
        self.countFaixas = 0
        print self.blackVisionReading
        if not self.bifurcacao:
            for i in range(3):
                if self.blackVisionReading[i]:
                    self.countFaixas+=1
                if self.countFaixas==2:
                    self.countFaixas = 0
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
        print 'done!'

    def updateEncoders(self):
        _,self.encoder[0] = vrep.simxGetJointPosition(self.clientID, self.motorHandle[0], vrep.simx_opmode_oneshot);
        _,self.encoder[1] = vrep.simxGetJointPosition(self.clientID, self.motorHandle[1], vrep.simx_opmode_oneshot);
        if self.angularDiff[0] >= 0:
            self.angularDiff[0] = self.encoder[0]-self.lastEncoder[0] if self.encoder[0]>=self.lastEncoder[0] else 2*PI-self.lastEncoder[0]+self.encoder[0]
        else:
            self.angularDiff[0] = self.encoder[0]-self.lastEncoder[0] if self.encoder[0]<=self.lastEncoder[0] else self.encoder[0]-self.lastEncoder[0]-2*PI

        if self.angularDiff[1] >= 0:
            self.angularDiff[1] = self.encoder[1]-self.lastEncoder[1] if self.encoder[1]>=self.lastEncoder[1] else 2*PI-self.lastEncoder[1]+self.encoder[1]
        else:
            self.angularDiff[1] = self.encoder[1]-self.lastEncoder[1] if self.encoder[1]<=self.lastEncoder[1] else self.encoder[1]-self.lastEncoder[0]-2*PI


        # self.angularDiff[0] = self.encoder[0]-self.lastEncoder[0]
        # self.angularDiff[1] = self.encoder[1]-self.lastEncoder[1]
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
    def goToPosition(self,xObj,yObj):
        xRobot = robotPosition[0] # ?
        yRobot = robotPosition[1] # ?
        orientation = robotOrientation[2] # ?
        thresh = 0.5
        # Estamos no corredor certo
        if(abs(xRobot-xObj) < thresh):
            # findY()
        else:
            if((yRobot-yObj) < 0 and )


