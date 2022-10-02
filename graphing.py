import numpy as np, matplotlib.pyplot as plt
from numpy.lib.function_base import append
chestStraight, x=0,0
def chestP(x):
    if (x>=30): chestStraight = np.tanh(-1 * (np.abs(x)/30))
    if (x<=-20): chestStraight = np.tanh(-1 * (np.abs(x)/20))
    if (x<30 and x>-20 and x!= 0): chestStraight = np.tanh(+1 / (np.abs(x)/30))
    return chestStraight
def head(x):
    if (np.abs(x)>=20): headStraight= np.tanh(-1 * (np.abs(x)/20))
    if (x<20 and x>-20 and x!= 0): headStraight = np.tanh(+1 / (np.abs(x)/20))
    return headStraight
def waistP(x):
    if (np.abs(x)>=10): waistStraight = np.tanh(-1 * (np.abs(x)/10))
    if (np.abs(x)<10): waistStraight = np.tanh(+1 / (np.abs(x)/10))
    return waistStraight 
def waistR(x):    
    if (np.abs(x)>=5): waistTilt = np.tanh(-1 * (np.abs(x)/5))
    if (np.abs(x)<5): waistTilt = np.tanh(+1 / (np.abs(x)/5))    
    return waistTilt
def footP(x):
    leftFlat = (2 * np.exp (-0.5 * (x/5)**2))-1.0   
    return leftFlat
def LfootR(x):
    if (x>20): leftFlat = (-1 * (np.abs(x)/40))
    if (x<0): leftFlat = (-1 * np.abs(x))
    if (x>=0 and x<=20): leftFlat = (+1 / (np.abs(x)/5))
    return leftFlat
def RfootR(x):
    rightNoPitch= (2 * np.exp( -0.1 * (x/2)**2))-1.0

    return rightNoPitch

def imuRollCost(x):
    rollCost= (2 * np.exp( -0.75 * (x/15)**2))-1
    # if x>=0:rollCost= (2 * np.exp( -0.7 * (x/30)**2))-1.0    
    # else: rollCost= (2 * np.exp( -6 * (x/30)**2))-1.0 
    # if (x ==0): rollCost = 0.75
    # if (abs(x) <15): rollCost = 0.75 * np.tanh(abs(x))
    # else: rollCost = -0.75 * np.tanh(abs(x))#/(rollCost-0.26))
    return rollCost
def imuP(x):
    if x>=0:pitchCost= (2 * np.exp( -0.75 * (x/30)**2))-1
    else: pitchCost= (2 * np.exp( -5 * (x/15)**2))-1
    return pitchCost

def torsoX(x): #ROLL
    Xcost = (2 * np.exp( -170 * (x/3)**2))-1
    return Xcost
def torsoY(x):
    if x<0: Ycost = (2 * np.exp( -300 * (x/1)**2))-1
    else: Ycost = (2 * np.exp( -50 * (x/1)**2))-1
    return Ycost
def torsoZDiff(x):
    XYcost = (2 * np.exp( -170 * (x/3)**2))-1
    return XYcost
def goodFunc(x):
    y=(2* np.exp(-0.5 * (x/15)**2))-1
    return y

def waistH(x):
    if x>0:y = (2 * np.exp( -75 * (x/0.75)**2))-1
    else: y = (2 * np.exp( -1000 * (x/0.99)**2))-1
    return y

def waistfeetX(avgFeetX):
    if avgFeetX<0: waistFeetXCost = (2 * np.exp( -200 * (avgFeetX/2)**2))-1
    else: waistFeetXCost = (2 * np.exp( -400 * (avgFeetX/8)**2))-1 
    return waistFeetXCost

def waistfeetY(avgFeetX):
    waistFeetXCost = (2 * np.exp( -70 * (avgFeetX/2)**2))-1
    return waistFeetXCost

x=np.linspace(-180,180, 1000)
xx=np.linspace(-180, 180, 1000)
res, res2, res3, res4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15 = [], [], [], [],[],[],[], [],[],[],[],[],[],[],[]
for i in x:
    res.append(chestP(i))
    res2.append(head(i))
    res3.append(waistP(i))
    res4.append(waistR(i))
    r5.append(footP(i))
    r6.append(LfootR(i))
    r8.append(RfootR(i))
    r7.append(goodFunc(i))
    r9.append(imuRollCost(i))
    r10.append(imuP(i))
    r11.append(torsoY(i))
    r12.append(waistH(i))
    r13.append(waistfeetX(i))
    r14.append(waistfeetY(i))
# plt.plot(x, res, "b", label="Chest Pitch")
# plt.plot(x, res2, "g", label="Head Pitch")
# plt.plot(x, res3, "c", label="Waist Pitch")
# plt.plot(x, res4, "r", label="Waist Roll")
# plt.plot(x, r5, "g", label="Foot Pitch")
# plt.plot(x, r6, "g", label="Left Foot Roll")
# plt.plot(x, r8, "r", label="Foot Roll")
# plt.plot(x, r9, "r", label="IMU ROLL (X axis)")
# plt.plot(x, r10, "g", label="IMU PITCH (Y axis)")
# plt.plot(x, r11, 'r', label= "TORSO X. Y MOVEMENT")
# plt.plot(x, r13, 'r', label= "Waist X feet X")
# plt.plot(x, r12, 'r', label= "Waist Y Shift")

plt.axhline(0, color='k')
plt.axvline(0, color='k')
# plt.title("COST FOR X, Y MOVEMENT (#20)")
plt.title("COST FOR FLAT FEET")
# plt.title("IMU Orientation (#19)")
plt.xlabel("Rotation (degrees)")
# plt.xlabel("Shift (metres)")
plt.ylabel("Reward")
plt.ylim(-1,1)
plt.xlim(-90,90)
plt.legend()
plt.show()


# import pandas as pd
# import numpy as np
# import matplotlib
# import matplotlib.pyplot as plt

# content = []

# with open("output/agent0_log.txt", "r") as f:
#     content = f.readlines()

# for i in range(len(content)):
#     content[i] = ' '.join(content[i].split())
#     content[i] = content[i].split(" ")

# header = content[0]
# df = pd.DataFrame(content[1:], columns=header, dtype=float)

# tr_vals = df["Train_Return"].values
# te_vals = df["Test_Return"].values
# x = range(0, len(tr_vals)*400, 400)

# fig, ax = plt.subplots()
# ax.plot(x, tr_vals, label="Train")
# ax.plot(x, te_vals, label="Test")
# ax.set_xlim(0, (len(tr_vals)+1)*400)
# ax.set_xlabel('Iterations')
# ax.set_ylim(0, 100, 10)
# ax.set_ylabel('Return')
# ax.legend()
# plt.savefig("output/rewards_log.png")