from controller import Robot, Compass
from controller import CameraRecognitionObject
import time
from transitions import Machine
import warnings
warnings.filterwarnings('ignore')
# Create the Robot instance.
robot = Robot()
shelf = [0.4,0.3,0.5,0.6,0.0,0.1,0.8,0.7]
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
camera1.enable(10)
camera1.recognitionEnable(10)
camera2=robot.getCamera('camera2')
camera2.enable(10)
camera2.recognitionEnable(10)
camera3=robot.getCamera('camera3')
camera3.enable(10)
camera3.recognitionEnable(10)

compass = robot.getCompass("compass")
gps = robot.getGPS("gps")
compass.enable(timestep)
gps.enable(timestep)
sensors = []
wsensor = []
fsensor = []
fsensor.append(robot.getPositionSensor("finger1sensor"))
fsensor.append(robot.getPositionSensor("finger1sensor"))
sensors.append(robot.getPositionSensor("arm1sensor"))
sensors.append(robot.getPositionSensor("arm2sensor"))
sensors.append(robot.getPositionSensor("arm3sensor"))
sensors.append(robot.getPositionSensor("arm4sensor"))
sensors.append(robot.getPositionSensor("arm5sensor"))
wsensor.append(robot.getPositionSensor("wheel1sensor"))
wsensor.append(robot.getPositionSensor("wheel2sensor"))
wsensor.append(robot.getPositionSensor("wheel3sensor"))
wsensor.append(robot.getPositionSensor("wheel4sensor"))
for sensor in fsensor:
    sensor.enable(timestep)
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
            vel = ((distance-run)/distance)*14*scale
            setwheel(-vel,-vel,-vel,-vel)
            #print("foraward")
            if (abs(run-distance)<=0.01):
                setwheel(0,0,0,0)
                #print("here we go")
                a = 1
                break
        else:
            vel = -1*((distance*(-1)-run)/(distance*(-1)))*14*scale
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
            vel = ((distance-run)/distance)*14*scale
            setwheel(-vel,vel,vel,-vel)  # +,-,-+ :right
            #print("right")
            if (abs(run-distance)<=0.01):
                setwheel(0,0,0,0)
                #print("here we go")
                a = 1
                break
        else:
            vel = -1*((distance*(-1)-run)/(distance*(-1)))*14*scale
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
            if(camera2.getRecognitionObjects()):
                for item in camera1.getRecognitionObjects():
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

def fullgrip(p,leng):
    xobject = -p[0] + gps.getValues()[2]+0.15
    yobject = p[2] + gps.getValues()[0]-0.14
    t = toward
    if(toward%2==0):
        print("here")
        setarm(1.59,1.5,1.5,-1.44,0,1)
        go_forward(p[0]-0.15+0.17,scale=0.2)
        go_rl(-p[2]+0.15-0.336-leng*0.5,scale=0.2)
        #print(xobject,yobject)
        #print('togo',xobject-0.161*(-t+1),yobject+(0.336+leng*0.5)*(-t+1))
        #simplego(xobject-0.161*(-t+1),yobject+(0.336+leng*0.5)*(-t+1))
        print('gps',gps.getValues())
        grip()
        go_rl(-0.26,scale=0.2)#上面的抓取好像自己抬手了，缩合抓手之后向右动固定距离
        setarm(0,0.8,1.7,-1.5,0,1)
    else: #3.12669e-05
        print("here1")
        setarm(1.59,1.5,1.5,-1.44,0,1)
        go_forward(p[0]-0.15+0.17,scale=0.2)
        go_rl(-p[2]+0.15-0.336-leng*0.5,scale=0.2)
        # print(xobject,yobject)
        # simplego(yobject-0.161*(-t+2),xobject-(0.336+leng*0.5)*(-t+2))
        print('gps',(gps.getValues()))
        grip()
        go_rl(-0.26,scale=0.2)
        setarm(0,0.8,1.7,-1.5,0,1)

# armpos = [[-1.59,1.5,1.5,-1.5,0],[-1.59,0.8,1.7,-1.0,0],[-1.590,0.5,0.5,0.5,0]]

# def release(height=0):
#     setarm(-1.59,0.8,1.5,-1.5,0,1)
#     setarm(armpos[height][0],armpos[height][1],armpos[height][2],armpos[height][3],armpos[height][4],1)
#     go_rl(-0.26)
#     finger1.setPosition(fingerMax)
#     finger2.setPosition(fingerMax)
#     #setarm(0,0,1.7,0,0,1)

# armpos = [[-1.59,1.5,1.5,-1.5,0],[-1.59,0.8,1.7,-1.0,0],[-1.590,0.25,0.2,1,0]]
armpos = [[-1.59,1.5,1.5,-1.5,0],[-1.59,0.6,1.5,-0.7,0],[-1.590,0.25,0.15,1,0]]#0.2


def release(height=0):
    setarm(-1.59,0.8,1.5,-1.5,0,1)
    setarm(armpos[height][0],armpos[height][1],armpos[height][2],armpos[height][3],armpos[height][4],1)
    if(height==2):
        go_rl(-0.344)
    else:
        go_rl(-0.285)
    finger1.setPosition(fingerMax)
    finger2.setPosition(fingerMax)
    while(not(closeto(fsensor[0].getValue(),fingerMax) and
            closeto(fsensor[1].getValue(),fingerMax))):
        robot.step(timestep)



def goobj(xobject,yobject,lengthobject):
    t = toward
    if(toward%2==0):
        simplego(xobject-0.151*(-t+1),yobject+(0.336+lengthobject)*(-t+1))
    else:
        simplego(yobject-0.151*(-t+2),xobject-(0.336+lengthobject)*(-t+2))

toward=0
class Matter(object):
    pass
model = Matter()
xmid = [0.667,-1.1,-0.667,1.1]
ymid = [1.1,0.667,-1.1,-0.667] #0.48
xmid1 = [1.05,-1.05,-1.05,1.05]
ymid1 = [1.05,1.05,-1.05,-1.05] #0.48
xmid2 = [1.1,-1.1,-1.1,1.1]
ymid2 = [1.1,1.1,-1.1,-1.1]

xxun = [0.84,-1,-0.84,1]
yxun = [1,0.84,-1,-0.84]

# 状态定义
states=['raozhuzou','search object', 'goto shelf', 'back to object']

# 定义状态转移
# The trigger argument defines the name of the new triggering method
transitions = [
    {'trigger': 'raowan', 'source': 'raozhuzou', 'dest': 'search object'},
    {'trigger': 'getobject', 'source': 'search object', 'dest': 'goto shelf'},
    {'trigger': 'putontheshelf', 'source': 'goto shelf', 'dest': 'back to object'},
    {'trigger': 'arriveobject', 'source': 'back to object', 'dest': 'search object'}]

machine = Machine(model=model, states=states, transitions=transitions, initial='search object')
#每个color编号的物体宽度
lengthobject = [0.2,0.06,0.06,0.04,0.049,0.049,0.05,0.2,0.2]
# for item in camera1.getRecognitionObjects():
#     print(item.get_model())
finger1.setPosition(0.1)
finger2.setPosition(0.1)
size = 0.6
toward = 0
gotcolor = 1
setarm(0,0.8,1.7,-1.5,0,1)
# setarm(-1.590,0.25,0.15,1,0,1)
# robot.step(10000)
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
        if(camera1.getRecognitionObjects()):
            print("heee")
            for item in camera1.getRecognitionObjects():
                print("aaaa")
                print(item.get_position())
                if ((item.get_position()[0]**2+item.get_position()[2]**2)**0.5) < size:
                    gotcolor = item.get_colors()[0]
                    p = item.get_position()
                    size = (item.get_position()[0]**2+item.get_position()[2]**2)**0.5
        else:
            searchobj(toward)
            continue
        if(gotcolor != 1):
            got = gotcolor
        else:
            print('ssss')
            searchobj(toward)
            continue
            # break
        if p[0] :
            fullgrip(p,lengthobject[int(10*got)])
            size = 0.5  
            gotcolor = 1
            p =[]
            model.getobject() #至此抓取完成 回到抓取线


    elif(model.state =="goto shelf"):
        print("goto shelf")
        #移动到判断好的货架处
        for _ in range(len(shelf)):
            if (shelf[_] == got):
                weizhi = _%4
                #下面全换成weizhi，_作为物体shelf的索引
                if got == 0.7 or got == 0.8 or got ==0.0:
                    simplego(xmid2[weizhi],ymid2[weizhi])
                
                    # simplego(xmid2[weizhi],ymid2[weizhi],scale=0.2)
                else:
                    simplego(xmid1[weizhi],ymid1[weizhi])
                    
                    # simplego(xmid1[weizhi],ymid1[weizhi],scale=0.2)
                simplego(xmid[weizhi],ymid[weizhi],scale=0.2)#存储四个点即1.1位置点      
                while(toward != weizhi):
                    robot.step(timestep)
                    rotate((toward+1)%4,1)
                    toward = (toward+1)%4#此时到达1.1 朝向也调整好
                #寻找空柜并对准准备放货
                simplego(xmid[weizhi],ymid[weizhi],scale=0.2)
                while(model.state=="goto shelf"):
                    print("jjjjjjjjjj")
                    robot.step(timestep)
                    #下排物体寻空格子
                    if (got == 0.3 or got ==0.4 or got ==0.5 or got ==0.6):
                        if(camera2.getRecognitionObjects()):
                            flags = len(camera2.getRecognitionObjects())
                            s = 0
                            for i in camera2.getRecognitionObjects():
                                if i.get_position_on_image()[0] > 22 and i.get_position_on_image()[0]<49 :
                                    #此处判定为格子非空 前进 寻找下一个格子
                                    go_forward(0.25)
                                    break
                                else:
                                    s += 1
                                    if s == flags:
                                    #格子是空的，进行放货
                                        print('yeyeyeyeyp')
                                        release(height=0)
                                        model.putontheshelf()
                                        break
                                    else:
                                        print("uuuuuuuuuu")
                                        continue
                        else:
                            print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
                            #附近没有货物，也判定为空的，进行放货。
                            release(height = 0)
                            model.putontheshelf()
                            break
                    #上排物体寻空格子
                    elif (got == 0.0 or got ==0.8):
                        if(camera3.getRecognitionObjects()):
                            flags = len(camera3.getRecognitionObjects())
                            s = 0
                            for i in camera3.getRecognitionObjects():
                                if i.get_position_on_image()[0] > 22 and i.get_position_on_image()[0]<49:
                                    #此处判定为格子非空 前进 寻找下一个格子
                                    go_forward(0.25)
                                    break
                                else:
                                    s += 1
                                    if s == flags:
                                    #格子是空的，进行放货
                                        print('yeyeyeyeyp')
                                        release(height=1)
                                        model.putontheshelf()
                                        break
                                    else:
                                        print("uuuuuuuuuu")
                                        continue
                        else:
                            print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
                            #附近没有货物，也判定为空的，进行放货。
                            release(height = 1)
                            model.putontheshelf()
                            break
                    #最上排的物体
                    else:
                        if(camera3.getRecognitionObjects()):
                            flags = len(camera3.getRecognitionObjects())
                            s = 0
                            for i in camera3.getRecognitionObjects():
                                if i.get_position_on_image()[0] > 22 and i.get_position_on_image()[0]<49:
                                    #此处判定为格子非空 前进 寻找下一个格子
                                    go_forward(0.25)
                                    break
                                else:
                                    s += 1
                                    if s == flags:
                                        #格子是空的，进行放货
                                        print('yeyeyeyeyp')
                                        release(height=2)
                                        model.putontheshelf()
                                        break
                                    else:
                                        print("uuuuuuuuuu")
                                        continue

                        else:
                            #附近没有货物，也判定为空的，进行放货。
                            release(height = 2)
                            model.putontheshelf()
                            break
                    continue

    elif(model.state=="back to object"):
        print("back to object")
        # 就近返回
        # go_rl(0.26)#左移一个确定的距离，回到货物附近#)
        go_rl(max(abs(gps.getValues()[0]),abs(gps.getValues()[2]))-1.1)
        rotate(toward,1)
        setarm(0,0.8,1.7,-1.5,0,1)
        model.arriveobject()
    pass

