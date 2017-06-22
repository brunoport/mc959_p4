import vrep,robot

print '****** Program started ******'
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',25000,True,True,5000,5)

if clientID!=-1:
    print 'Connected to remote API server'

    rob = robot.Robot(clientID, "Pioneer_p3dx")

    #rob.takePicture("Camera_Gondola")

    while True:
        rob.run()
    vrep.simxFinish(clientID)
else:
    print 'Failed connecting to remote API server'
print '****** Program ended ******'
