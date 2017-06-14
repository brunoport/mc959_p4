import vrep

class Robot:
    """Classe robo do V-REP"""

    handle = 0                  # robot handle
    encoderHandle = [0,0]       # left and right encoder handlers
    motorHandle = [0,0]         # left and right motor handlers
    sonarHandle = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]      # 16 sonar handlers
    sonarReading = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]
    robotPosition = []
    robotOrientation = []

    def __init__(self, clientID, name):
        self.clientID = clientID
        self.name = name
        print "initializing robot... "

        # Get robot handle
        _,self.handle = vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking);

        # Get handles of sensors and actuators
        _,self.encoderHandle[0] = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftWheel", vrep.simx_opmode_oneshot_wait)
        _,self.encoderHandle[1] = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightWheel", vrep.simx_opmode_oneshot_wait)

        #  Get handles of sensors and actuators
        _,self.motorHandle[0] = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait)
        _,self.motorHandle[1] = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor",vrep.simx_opmode_oneshot_wait)

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

        print "robotPosition = " + str(self.robotPosition)
        print "robotOrientation = " + str(self.robotOrientation)

        self.move(2,2)
        self.readSonars()

    def move(self, leftMotorVelocity, rightMotorVelocity):
        vrep.simxSetJointTargetVelocity(self.clientID, self.motorHandle[0], leftMotorVelocity, vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetVelocity(self.clientID, self.motorHandle[1], rightMotorVelocity, vrep.simx_opmode_streaming);

    def readSonars(self):
        # Reads sonar's current value
        for i in range(16):
            _,_,self.sonarReading[i],_,_ = vrep.simxReadProximitySensor(self.clientID,self.sonarHandle[i],vrep.simx_opmode_streaming);
            print "sonarReading["+str(i)+"] = "+str(self.sonarReading[i])
