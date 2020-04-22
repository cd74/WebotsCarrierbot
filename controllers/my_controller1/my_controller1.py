from controller import Robot, Compass
from controller import CameraRecognitionObject
import time
from transitions import Machine
# Create the Robot instance.
robot = Robot()
shelf = [None,None,None,None]
pi = 3.14159265
toward = 0
a=1
gpsValue =[0,1,2]
def setpos(device,sersor,pos):
    while(abs(sensor.getValue()-pos)>0.01):
        device.setPosition(pos)
        print('aaa')

    # Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
#------------------- motor control-------------------------#
# Inizialize base motors.

wheels = []
wheels.append(robot.getMotor("wheel1"))
wheels.append(robot.getMotor("wheel2"))
wheels.append(robot.getMotor("wheel3"))
wheels.append(robot.getMotor("wheel4"))
for wheel in wheels:
    # Activate controlling the motors setting the velocity.
    # Otherwise by default the motor expects to be controlled in force or position,
    # and setVelocity will set the maximum motor velocity instead of the target velocity.
    wheel.setPosition(float('+inf'))
for wheel in wheels:
    wheel.setVelocity(0)
#-------------------- arm control --------------------------#
# Initialize arm motors.
arms = []
arms.append(robot.getMotor("arm1"))
arms.append(robot.getMotor("arm2"))
arms.append(robot.getMotor("arm3"))
arms.append(robot.getMotor("arm4"))
arms.append(robot.getMotor("arm5"))
# arms[0].setVelocity(0.2)
# arms[1].setVelocity(0.5)
# arms[2].setVelocity(0.5)
# arms[3].setVelocity(0.3)
# Set the maximum motor velocity.
finger1 = robot.getMotor("finger1")
finger2 = robot.getMotor("finger2")
fingerMin = finger1.getMinPosition()
fingerMax = finger1.getMaxPosition()

#-------------------- sensor --------------------------#
# Initialize arm position sensors.
# These sensors can be used to get the current joint position and monitor the joint movements.
camera1=robot.getCamera('camera1')
camera1.enable(50)
camera1.recognitionEnable(50)
camera2=robot.getCamera('camera2')
camera2.enable(50)
camera2.recognitionEnable(50)

compass = robot.getCompass("compass")
gps = robot.getGPS("gps")
compass.enable(timestep)
gps.enable(timestep)
sensors = []
wsensor = []
sensors.append(robot.getPositionSensor("arm1sensor"))
sensors.append(robot.getPositionSensor("arm2sensor"))
sensors.append(robot.getPositionSensor("arm3sensor"))
sensors.append(robot.getPositionSensor("arm4sensor"))
sensors.append(robot.getPositionSensor("arm5sensor"))
wsensor.append(robot.getPositionSensor("wheel1sensor"))
wsensor.append(robot.getPositionSensor("wheel2sensor"))
wsensor.append(robot.getPositionSensor("wheel3sensor"))
wsensor.append(robot.getPositionSensor("wheel4sensor"))
for sensor in wsensor:
    sensor.enable(timestep)
for sensor in sensors:
    sensor.enable(timestep)   
#-------------------- finger control --------------------------#
# Initialize gripper motors.

def closeto(x,y):
    if(abs(x-y)<0.01):
        return 1
    return 0
    
def setwheel(v1,v2,v3,v4):
    wheels[0].setVelocity(v1)
    wheels[1].setVelocity(v2)
    wheels[2].setVelocity(v3)
    wheels[3].setVelocity(v4)

# def rotate(t):
    # d0 = wsensor[0].getValue()
    # dist = 0.277395*pi/2
    # while(1):
        # robot.step(timestep)
        # run = 0.05*abs(wsensor[0].getValue()-d0)
        # v = ((dist-run)/dist)*10*t
        # setwheel(v,-v,v,-v)
        # if (run>=dist):
            # setwheel(0,0,0,0)
            # break

def rotate(dst,t):
    dir = compass.getValues()
    d1 = [0,1,0,-1]
    d2 = [1,0,-1,0]
    while(1):  
        robot.step(timestep)      
        run1 = abs(compass.getValues()[0]-d1[dst])
        run2 = abs(compass.getValues()[1]-d2[dst])
        v = max(run1,0.1)*10*t       
        if (run1<=0.01 and run2<=0.01):
            setwheel(0,0,0,0)
            #print('complete')
            break
        setwheel(v,-v,v,-v)
            
def setarm(a1,a2,a3,a4,a5,wait):
    arms[0].setPosition(a1)
    arms[1].setPosition(a2)
    arms[2].setPosition(a3)
    arms[3].setPosition(a4)  
    arms[4].setPosition(a5) 
    if(wait):
        while(not(closeto(sensors[0].getValue(),a1) and
              closeto(sensors[1].getValue(),a2) and
              closeto(sensors[2].getValue(),a3) and
              closeto(sensors[3].getValue(),a4) and
              closeto(sensors[4].getValue(),a5))):
            robot.step(timestep)
            
def go_forward(distance,scale=1):
    global a
    global gpsValue
    while(1):
        robot.step(timestep)
        while(a):
            gpsValue = gps.getValues()
            a=0
        run = ((gps.getValues()[0]-gpsValue[0])**2+(gps.getValues()[2]-gpsValue[2])**2)**0.5
        if(distance >=0):
            vel = ((distance-run)/distance)*15*scale
            setwheel(-vel,-vel,-vel,-vel)
            #print("foraward")
            if (abs(run-distance)<=0.01):
                setwheel(0,0,0,0)
                #print("here we go")
                a = 1
                break
        else:
            vel = -1*((distance*(-1)-run)/(distance*(-1)))*15*scale
            setwheel(-vel,-vel,-vel,-vel)
            #print("backing")
            if (((-1*distance)-run)<=0.01):
                setwheel(0,0,0,0)
                #print("here we go")
                a = 1
                break    
                          
def go_rl(distance,scale=1):
    global a
    global gpsValue
    while(1):
        robot.step(timestep)
        while(a):
            gpsValue = gps.getValues()
            a=0
        run = ((gps.getValues()[0]-gpsValue[0])**2+(gps.getValues()[2]-gpsValue[2])**2)**0.5
        if(distance >=0):
            vel = ((distance-run)/distance)*15*scale
            setwheel(-vel,vel,vel,-vel)  # +,-,-+ :right
            #print("right")
            if (abs(run-distance)<=0.01):
                setwheel(0,0,0,0)
                #print("here we go")
                a = 1
                break
        else:
            vel = -1*((distance*(-1)-run)/(distance*(-1)))*15*scale
            setwheel(-vel,vel,vel,-vel)
            #print("left")
            if (((-1*distance)-run)<=0.01):
                setwheel(0,0,0,0)
                #print("here we go")
                a = 1
                break
            
def simplego(x,y,scale=1):
    cord = gps.getValues()
    t = toward
    if(t%2==0):
        go_forward((x-cord[2])*(t-1),scale)
        go_rl((y-cord[0])*(t-1),scale)
    else: 
        go_forward((y-cord[0])*(t-2),scale)
        go_rl((x-cord[2])*(-t+2),scale)

def findshelf():
    global toward
    xd = [-1.32,-0.84,1.32,0.2]
    yd = [0.84,-1.32,-0.84,1.2]
    simplego(0.84,1.32)
    for j in range(4):
        for i in range(7):
            go_forward(0.26)
            if(camera2.getRecognitionObjects()):
                for item in camera2.getRecognitionObjects():
                    shelf[j] = item.get_colors()[0]
            # robot.step(timestep)
        rotate((toward+1)%4,1)
        toward = (toward+1)%4
        simplego(xd[j],yd[j])

def searchobj(p):
    global toward
    xd = [1,-1,-1,1]
    yd = [1,1,-1,-1]
    simplego(xd[p],yd[p])
    for j in range(4):
        go_forward(0.4)
        for i in range(6):
            go_forward(0.2)
            if(camera1.getRecognitionObjects()):
                for item in camera1.getRecognitionObjects():
                    if (item.get_colors()[0]==0.1 or item.get_colors()[0]==0.2 or item.get_colors()[0]==0.3 or item.get_colors()[0]==0.4 or item.get_colors()[0]==0.5 or item.get_colors()[0]==0.6):
                        if ((item.get_position()[0]**2+item.get_position()[2]**2)**0.5) < size:
                            return
            # robot.step(timestep)
        go_forward(0.4)
        rotate((toward+1)%4,1)
        toward = (toward+1)%4
        simplego(xd[(p+1)%4],yd[(p+1)%4])
        p=p+1


def grip():
    #setarm(1.59,1.5,1.5,-1.5,0,1)
    finger1.setPosition(0.0)
    finger2.setPosition(0.0)
    #setarm(0,0.8,1.7,-1.5,0,1)
    
def release():
    # setarm(-1.5,0.8,1.5,-1.5,0,1)
    setarm(-1.59,1.5,1.5,-1.5,0,1)
    finger1.setPosition(fingerMax)
    finger2.setPosition(fingerMax)
    #setarm(0,0,1.7,0,0,1)

def goobj(xobject,yobject,lengthobject):
    t = toward
    if(toward%2==0):
        simplego(xobject-0.151*(-t+1),yobject+(0.336+lengthobject)*(-t+1))
    else:
        simplego(yobject-0.151*(-t+2),xobject-(0.336+lengthobject)*(-t+2))
i=2
toward=0
class Matter(object):
    pass
model = Matter()
xmid = [0.48,-1.1,-0.48,1.1]
ymid = [1.1,0.48,-1.1,-0.48] #0.48

yfin = [1.37,0.48,-1.37,-0.48]
xfin = [0.48,-1.37,-0.48,1.37]
# 状态定义
states=['raozhuzou','search object', 'goto shelf', 'back to object']

# 定义状态转移
# The trigger argument defines the name of the new triggering method
transitions = [
    {'trigger': 'raowan', 'source': 'raozhuzou', 'dest': 'search object'},
    {'trigger': 'getobject', 'source': 'search object', 'dest': 'goto shelf'},
    {'trigger': 'putontheshelf', 'source': 'goto shelf', 'dest': 'back to object'},
    {'trigger': 'arriveobject', 'source': 'back to object', 'dest': 'search object'}]

machine = Machine(model=model, states=states, transitions=transitions, initial='raozhuzou')
#每个color编号的物体宽度
lengthobject = [0.08,0.09,0.09,0.088,0.0635,0.0635,0.062,0.08,0.04]
# for item in camera1.getRecognitionObjects():
#     print(item.get_model())
finger1.setPosition(0.1)
finger2.setPosition(0.1) 
size = 0.6
gotcolor = 1


def fullgrip(item,leng):
    xobject = item.get_position()[0] + gps.getValues()[2]+0.15
    yobject = item.get_position()[2] + gps.getValues()[0]-0.14
    t = toward
    if(toward%2==0):
        print("here")
        setarm(1.59,1.5,1.5,-1.5,0,1)
        simplego(xobject-0.151*(-t+1),yobject+(0.336+leng*0.5)*(-t+1))
        grip()
        go_rl(-0.26)#上面的抓取好像自己抬手了，缩合抓手之后向右动固定距离
        setarm(0,0.8,1.7,-1.5,0,1)
    else:
        print("here1")
        setarm(1.59,1.5,1.5,-1.5,0,1)
        simplego(yobject-0.151*(-t+2),xobject-(0.336+leng*0.5)*(-t+2))
        grip()
        go_rl(-0.26)
        setarm(0,0.8,1.7,-1.5,0,1)





while(robot.step(timestep)!=-1):
    if (model.state == "raozhuzou"):
        setarm(0,0.8,1.7,-1.5,0,1)
        print("raozhuzou")
        findshelf()
        for i in shelf:
            print(i)
        model.raowan()
    elif (model.state=="search object"):
        print("serach object")
        # if(camera1.getRecognitionObjects()):
        #     for item in camera1.getRecognitionObjects():
        #         if ((item.get_position()[0]**2+item.get_position()[2]**2)**0.5) < size:
        #             gotcolor = item.get_colors()[0]
        #             p = item.get_position()
        #             size = (item.get_position()[0]**2+item.get_position()[2]**2)**0.5

        if(camera1.getRecognitionObjects()):
            for item in camera1.getRecognitionObjects():
                #判断盒子是否是侧面
                if (item.get_colors()[0]==0.7 or item.get_colors()[0]==0.0 or item.get_colors()[0] == 0.8):
                    if (item.get_size_on_image()[0]/item.get_size_on_image()[1]) < 0.3:
                        if ((item.get_position()[0]**2+item.get_position()[2]**2)**0.5) < size:
                            gotcolor = item.get_colors()[0]
                            p = item.get_position()
                            size = (item.get_position()[0]**2+item.get_position()[2]**2)**0.5
                    else:
                        continue
                #非盒子直接选最近的抓
                else:
                    if ((item.get_position()[0]**2+item.get_position()[2]**2)**0.5) < size:
                            gotcolor = item.get_colors()[0]
                            p = item.get_position()
                            size = (item.get_position()[0]**2+item.get_position()[2]**2)**0.5                    
            if(gotcolor != 1):
                got = gotcolor
            else:
                searchobj(toward)
                break
            fullgrip(item,lengthobject[int(10*got)])
            model.getobject()

  

        else:
            searchobj(toward)
            #go_forward(0.26)
            print("xiangqin")

        #抓取
        
    elif(model.state =="goto shelf"):
        print("goto shelf")
        #移动到判断好的货架处
        for _ in range(len(shelf)):
            if (shelf[_] == got):
                simplego(xmid[_],ymid[_])#存储四个点即1.1位置点 我坐标已经全乱了 ，改一下 我写不明白了
                while(toward != _):
                    robot.step(timestep)
                    rotate((toward+1)%4,1)
                    toward = (toward+1)%4#此时到达1.1 朝向也调整好
                #寻找空柜
                #---




                #调整抓手
                setarm(-1.59,0.8,1.5,-1.5,0,1)
                setarm(-1.59,1.5,1.5,-1.5,0,1)
                simplego(xfin[_],yfin[_])#到达放货地点，也是一组放货坐标 1.37位置处，我也找不明白了
                #放下货物
                release()
                model.putontheshelf()#状态切换语句，完成动作后的地方加上就行
                break
    elif(model.state=="back to object"):
        print("back to object")
        # 就近返回
        setarm(0,0.8,1.7,-1.5,0,1)
        go_rl(0.26)#左移一个确定的距离，回到货物附近#)
        model.arriveobject()