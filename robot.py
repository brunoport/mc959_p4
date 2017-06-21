from PIL import Image as I
import vrep,array,time,sys

class Robot:
    """Classe robo do V-REP"""

    handle = 0                  # robot handle
    encoderHandle = [0,0]       # left and right encoder handlers
    motorHandle = [0,0]         # left and right motor handlers
    sonarHandle = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]      # 16 sonar handlers
    sonarReading = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]
    robotPosition = []
    robotOrientation = []
    visionSensorHandles=[0,0,0]
    visionSensorReading=[False,False,False]
    corredor=2
    faixaASeguir=0
    countFaixas=0
    bifurcacao = False
    entrar = False
    countdown = 0
    sobreBifurcacao = False
    
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



    def run(self):
        # Get the robot current absolute position
        _,self.robotPosition = vrep.simxGetObjectPosition(self.clientID, self.handle,-1,vrep.simx_opmode_oneshot_wait);
        _,self.robotOrientation = vrep.simxGetObjectOrientation(self.clientID, self.handle,-1,vrep.simx_opmode_oneshot_wait);

        #print "-------------------------------------------------"
        #print "robotPosition = " + str(self.robotPosition)
        #print "robotOrientation = " + str(self.robotOrientation)

        #self.readSonars()
        self.readVision()
        #vLeft, vRight = self.avoidObstacle()
        self.bifurcacao = self.checkBifurcacao()
        if self.bifurcacao and not self.sobreBifurcacao:
            self.faixaASeguir-=1
            self.sobreBifurcacao = True
            print "BIFURCACAO " + str(self.bifurcacao)
            print "ENTRAR DAQUI " + str(self.faixaASeguir)
            if self.faixaASeguir == 0:
                print "ENTRAR AQUI ============>"
                self.entrar = True
                self.faixaASeguir=1#errado
            else:
                print "NAO EH ESSA AINDA"
                self.entrar = False
                self.move(2, 2)
                self.countdown = 10
                return
        elif not self.bifurcacao:
            self.countdown -=1
        
        if self.countdown>0:
            return
        self.sobreBifurcacao = False
        entrar = False
        vLeft, vRight = self.followLine()
        self.move(vLeft, vRight)

    
    def followLine(self):
        if self.visionSensorReading[2]:#direita
            return 2,1
        if not self.entrar and self.visionSensorReading[0]:#esquerda
            return 1,2
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
                self.visionSensorReading[i]=(data[0][11]<0.1) # data[11] is the average of intensity of the image
                # TRUE: sensor esta sobre a linha preta
                #print 'avg camera '+str(i)+' = ' + str(self.visionSensorReading[i])
    
    def checkBifurcacao(self):    
        self.countFaixas = 0            
        print self.visionSensorReading
        if not self.bifurcacao:
            for i in range(3):
                if self.visionSensorReading[i]:
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
