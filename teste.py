import vrep,robot
import random,time

def rollProduct(section):
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
    color = data[1+(2*i)]
    amount = data[1+(2*i)+1]
    return color,amount 

def rollSection():
    corridor = random.randint(1,4)
    if 2 <= corridor <= 3:
        section = chr(random.randint(65,70)) # A a F
    elif corridor == 4:
        section = chr(random.randint(65,67)) # A a C
    elif corridor == 1:
        section = chr(random.randint(68,70)) # D a F
    
    return str(corridor) + section 

def queuePrint(q):
    for s in q:
        print s,
    print ""

def queueAdd(q, section):
    q.append(section)

def queueGetFirst(q):
    if len(q) != 0:
        s = q.pop(0)
        return s
    else:
        return False

q = []
queueAdd(q, rollSection())
section = queueGetFirst(q)
print rollProduct(section)

#vrep.simxFinish(-1) # just in case, close all opened connections
#clientID=vrep.simxStart('127.0.0.1',25000,True,True,5000,5)
#
#if clientID!=-1:
#    print 'Connected to remote API server'
#
#    rob = robot.Robot(clientID, "Pioneer_p3dx")
#
#    rob.takePicture("Camera_Gondola")
#
#    while True:
#        rob.run()
#    vrep.simxFinish(clientID)
#else:
#    print 'Failed connecting to remote API server'
#print '****** Program ended ******'
