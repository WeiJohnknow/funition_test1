import time
import numpy as np
from dynamixel_sdk import *

address_TORQUE_ENABLE = 64
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

PROTOCOL_VERSION      = 2.0
BAUDRATE              = 3000000             
DEVICENAME            = 'COM9'    
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
# Open port
portHandler.openPort()
# Set port baudrate
portHandler.setBaudRate(BAUDRATE)  
#初始值(注意!不要覆蓋掉初始值)

Write_inds =np.arange(2,4)
Read_inds = np.arange(2,4)
  
#初始值

Goal_vels =np.ones(22)*20
Goal_poss =np.ones(22)*2048
Now_vels = np.ones(22)*20
Now_poss = np.ones(22)*2048
PWM = np.ones(22)*885
D = np.ones(22)*0
I = np.ones(22)*0
P = np.ones(22)*850



Data_cur_value = []
Data_vel_value = []
Data_pos_value = []
#換算後的電流與速度值
Data_ch_cur=[]
Data_ch_vel=[]
#0為顯示原值，1為顯示換算後值
Unit = 0 
unit_cur =0

Data_cur={}
Data_vel={}
Data_pos={}
Data_Unit_cur={}
Data_Unit_vel={}
#%%
def data2byte(data):
    return [DXL_LOBYTE(data), DXL_HIBYTE(data)]

def data4byte(data):
    return [DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data))]

def data6byte(data,data1,data2):
    return data2byte(data) + data2byte(data2) + data2byte(data3)

def data8byte(data,data2):
    return data4byte(data) + data4byte(data2)

def WriteVP(Write_inds,Goal_vels,Goal_poss,D,I,P,PWM,address_w,byte_w):
    global portHandler,packetHandler
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, address_w, byte_w)
    if byte_w == 2:
        for i in range(len(Write_inds)):
            vp = data2byte(PWM)
            groupSyncWrite.addParam(Write_inds[i],vp)
    elif byte_w == 6:
        for i in range(len(Write_inds)):
            vp = data2byte(D)+data2byte(I)+data2byte(P)
            groupSyncWrite.addParam(Write_inds[i],vp)
            
    elif byte_w == 8:
        for i in range(len(Write_inds)):
            Goal_vels = vels
            Goal_poss = poss
            vels = int(input('請輸入ID'+str(i)+'之速度 = '))
            poss = int(input('請輸入ID'+str(i)+'之位置 = '))
            vp = data8byte(int(Goal_vels[Write_inds[i]]), int(Goal_poss[Write_inds[i]]))
            groupSyncWrite.addParam(Write_inds[i],vp)
        
    groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()
    
    
def ReadVP(Read_inds, address_r, byte_r, pos, PWM):
    global portHandler,packetHandler
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, address_r, byte_r)
    for i in range(len(Read_inds)):
        groupSyncRead.addParam(Read_inds[i])
    if byte_r == 2:
        while True:
            groupSyncRead.txRxPacket()
            for i in range(len(Read_inds)):
                pwm= groupSyncRead.getData(Read_inds[i], 124, 2)
                print(pwm)
            if pwm == PWM :
                break
    elif byte_r == 10:  
        while True:
            groupSyncRead.txRxPacket()
            for i in range(len(Read_inds)):
                present_cur = groupSyncRead.getData(Read_inds[i], 126, 2)
                present_vel = groupSyncRead.getData(Read_inds[i], 128, 4)
                present_pos = groupSyncRead.getData(Read_inds[i], 132, 4)
                #value
                Data_cur_value.append(present_cur)
                Data_vel_value.append(present_vel)
                Data_pos_value.append(present_pos)
                if len(Data_cur_value) >= 6 and len(Data_vel_value) >= 6 and len(Data_pos_value) >= 6:
                    Data_cur_value.pop(0)
                    Data_vel_value.pop(0)
                    Data_pos_value.pop(0)
                #key
                Data_cur['ID'+str(Read_inds[i])+' : ']=Data_cur_value
                Data_vel['ID'+str(Read_inds[i])+' : ']=Data_vel_value
                Data_pos['ID'+str(Read_inds[i])+' : ']=Data_pos_value
            
            if (present_pos-3) < pos <(present_pos+3):
                break
    Unit_cur()
    Unit_vel()
    print(Data_pos)
    
def Unit_cur():
    for i in range(len(Data_cur_value)):
        if (Data_cur_value[i]&(1<<15))>>15 == 1 :
            unit_cur=(65535-(Data_cur_value[i]-1))*0.00336
        elif (Data_cur_value[i]&(1<<15))>>15 == 0 :        
            unit_cur=Data_cur_value[i]*0.00336  
        Data_ch_cur.append(np.round(unit_cur,2))
        if len(Data_ch_cur) >= 6 :
            Data_ch_cur.pop(0)

    for i in range(len(Read_inds)):    
        Data_Unit_cur['ID'+str(Read_inds[i])+' : ']=Data_ch_cur
    
    print(Data_Unit_cur)
def Unit_vel():
    for i in range(len(Data_vel_value)):
        if (Data_vel_value[i]&(1<<31))>>31 == 1 :
            Unit_vel=(4294967296-(Data_vel_value[i]-1))
            
        elif (Data_vel_value[i]&(1<<31))>>31 == 0 :
            Unit_vel=Data_vel_value[i]
        
        Data_ch_vel.append(np.round(Unit_vel,2))
        if len(Data_ch_vel) >= 6 :
            Data_ch_vel.pop(0)
    for i in range(len(Read_inds)):   
        Data_Unit_vel['ID'+str(Read_inds[i])+' : ']=Data_ch_vel
    print(Data_Unit_vel)
def TORQUE_ON(Write_inds):
    global portHandler,packetHandler
    for i in range(len(Write_inds)):
        packetHandler.write1ByteTxRx(portHandler, Write_inds[i], address_TORQUE_ENABLE, TORQUE_ENABLE)
def TORQUE_OFF(Write_inds):
    global portHandler,packetHandler
    for i in range(len(Write_inds)):
        packetHandler.write1ByteTxRx(portHandler, Write_inds[i], address_TORQUE_ENABLE, TORQUE_DISABLE)
        
def sin2pift():
    t = 0
    data_vel = [] #初始化
    data_pos = [] #初始化
    while True:    
        rad = np.deg2rad(deg*(np.sin(2*np.pi*f*t)))
        #將收集的弳度資料收集至pos的list裡，並取用倒數第一個值(最新)與倒數第二個值來相減，即可得到位移量。
        pos.append(rad)    
        rad_l =pos[-1]-pos[-2]          #位移量
        rad_s =rad_l/dt                 #位移量/分割時間=轉速            
        rpm=rad_s*60/(2*np.pi)          #將rad/s換成rad/m之後除以2pi會等於rpm
        vel = int(np.abs(rpm/0.229))    #馬達手冊之轉速
        change_pos = int(2048+deg*(np.sin(2*np.pi*f*t)*4095/360))
        if vel == 0:
            vel =1
        data_vel.append(vel)            #紀錄轉速參數
        data_pos.append(change_pos)     #紀錄位置變化
        t += dt
        if t >= T:
            break
        print('共有',len(data_vel),'筆資料')
#%%
print('控制方案選擇:\n1. PID\n2. PWM\n3. Volicity、Position')
ch = int(input('請選擇控制方案 = '))
TORQUE_ON(Write_inds)
ReadVP(Read_inds,132, 4, pos, PWM)
if ch == 1:
    P = int(input('請輸入P = '))
    I = int(input('請輸入I = '))
    D = int(input('請輸入D = '))
    address_w = 80
    byte_w = 6
    address_r = 80
    byte_r = 12
    WriteVP(Write_inds,Goal_vels,Goal_poss,D,I,P,PWM,address,byte)
elif ch == 2:
    PWM = int(input('請輸入PWM = '))
    address_w = 100
    byte_w = 2
    address_r = 124
    byte_r = 2
    WriteVP(Write_inds,Goal_vels,Goal_poss,D,I,P,PWM,address_w,byte_w)
    ReadVP(Read_inds, address_r, byte_r, pos, PWM)
elif ch == 3: 
    address_w = 112
    byte_w = 8
    address_r = 126
    byte_r = 10
    WriteVP(Write_inds,Goal_vels,Goal_poss,D,I,P,PWM,address_w,byte_w)
    ReadVP(Read_inds, address_r, byte_r, pos, PWM)
    
else:
    TORQUE_OFF(Write_inds)
    portHandler.closePort()