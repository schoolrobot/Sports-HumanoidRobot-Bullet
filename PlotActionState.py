import numpy as np, matplotlib.pyplot as plt, matplotlib.animation as animation
from matplotlib import style

# style.use('fivethirtyeight')

fig = plt.figure(num=None, figsize=(24, 24), dpi=80, facecolor='w', edgecolor='k')

ax1 = plt.subplot2grid((7, 3), (0, 0))
ax2 = plt.subplot2grid((7, 3), (0, 1))
ax3 = plt.subplot2grid((7, 3), (0, 2))
ax4 = plt.subplot2grid((7, 3), (1, 0))
ax5 = plt.subplot2grid((7, 3), (1, 1))
ax6 = plt.subplot2grid((7, 3), (1, 2))
ax7 = plt.subplot2grid((7, 3), (2, 0))
ax8 = plt.subplot2grid((7, 3), (2, 1))
ax9 = plt.subplot2grid((7, 3), (2, 2))
ax10 = plt.subplot2grid((7, 3), (3, 0))
ax11 = plt.subplot2grid((7, 3), (3, 1))
ax12 = plt.subplot2grid((7, 3), (3, 2))
ax13 = plt.subplot2grid((7, 3), (4, 0))
ax14 = plt.subplot2grid((7, 3), (4, 1))
ax15 = plt.subplot2grid((7, 3), (4, 2))
ax16 = plt.subplot2grid((7, 3), (5, 0))
ax17 = plt.subplot2grid((7, 3), (5, 1))
ax18 = plt.subplot2grid((7, 3), (5, 2))
ax19 = plt.subplot2grid((7, 3), (6, 0))
ax20 = plt.subplot2grid((7, 3), (6, 2))
ax21 = plt.subplot2grid((7, 3), (6, 2))
ax22 = plt.subplot2grid((7, 3), (6, 2))
ax23 = plt.subplot2grid((7, 3), (6, 2))
ax24 = plt.subplot2grid((7, 3), (6, 2))
ax25 = plt.subplot2grid((7, 3), (6, 2))
ax26 = plt.subplot2grid((7, 3), (6, 2))
ax27 = plt.subplot2grid((7, 3), (6, 2))
ax28 = plt.subplot2grid((7, 3), (6, 2))
ax29 = plt.subplot2grid((7, 3), (6, 2))
ax30 = plt.subplot2grid((7, 3), (6, 2))
ax31 = plt.subplot2grid((7, 3), (6, 2))
ax32 = plt.subplot2grid((7, 3), (6, 2))

def animate(i):
    with open("/home/admin/dribble_repo/action_log.txt") as f:
        # for line in f:
        
        lines = f.readlines()
        x = [j for j in range(len(lines))]
        # y = line.split()
        y = [str(line) for line in lines]
        # for line in y:
        #     y = line.split()
        #     y = [float(e) for e in y]
            # print(len(y))
        
        a = [float(line.split(' ')[0]) for line in lines]
        ax1.clear()
        ax1.set_title('Curriculum')
        ax1.set_xlabel('Number of Episodes')
        ax1.set_ylabel('Number of Pull Ups * 2')
        # if x[len(x)-1] > 50:
        #     ax1.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax1.plot(x,a)
        
        b = [float(line.split(' ')[1]) for line in lines]
        ax2.clear()
        ax2.set_title('HEAD_Y')
        # ax2.set_xlabel('Timestep')
        ax2.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax2.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax2.plot(x,b)

        c = [float(line.split(' ')[2]) for line in lines]
        ax3.clear()
        ax3.set_title('L_SHOULDER_P')
        # ax3.set_xlabel('Timestep')
        ax3.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax3.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax3.plot(x,c)

        d = [float(line.split(' ')[3]) for line in lines]
        ax4.clear()
        ax4.set_title('L_SHOULDER_R')
        # ax4.set_xlabel('Timestep')
        ax4.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax4.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax4.plot(x,d)

        e = [float(line.split(' ')[4]) for line in lines]
        ax5.clear()
        ax5.set_title('L_SHOULDER_Y')
        # ax5.set_xlabel('Timestep')
        ax5.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax5.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax5.plot(x,e)

        f = [float(line.split(' ')[5]) for line in lines]
        ax6.clear()
        ax6.set_title('L_ELBOW_P')
        # ax6.set_xlabel('Timestep')
        ax6.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax6.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax6.plot(x,f)

        g = [float(line.split(' ')[6]) for line in lines]
        ax7.clear()
        ax7.set_title('L_ELBOW_Y')
        # ax7.set_xlabel('Timestep')
        ax7.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax7.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax7.plot(x,g)

        h = [float(line.split(' ')[7]) for line in lines]
        ax8.clear()
        ax8.set_title('L_WRIST_P')
        # ax8.set_xlabel('Timestep')
        ax8.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax8.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax8.plot(x,h)

        i = [float(line.split(' ')[8]) for line in lines]
        ax9.clear()
        ax9.set_title('L_WRIST_Y')
        # ax9.set_xlabel('Timestep')
        ax9.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax9.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax9.plot(x,i)

        j = [float(line.split(' ')[9]) for line in lines]
        ax10.clear()
        ax10.set_title('R_SHOULDER_P')
        # ax10.set_xlabel('Timestep')
        ax10.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax10.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax10.plot(x,j)

        k = [float(line.split(' ')[10]) for line in lines]
        ax11.clear()
        ax11.set_title('R_SHOULDER_R')
        # ax11.set_xlabel('Timestep')
        ax11.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax11.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax11.plot(x,k)

        l = [float(line.split(' ')[11]) for line in lines]
        ax12.clear()
        ax12.set_title('R_SHOULDER_Y')
        # ax12.set_xlabel('Timestep')
        ax12.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax12.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax12.plot(x,l)

        m = [float(line.split(' ')[12]) for line in lines]
        ax13.clear()
        ax13.set_title('R_ELBOW_P')
        # ax13.set_xlabel('Timestep')
        ax13.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax13.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax13.plot(x,m)

        o = [float(line.split(' ')[13]) for line in lines]
        ax14.clear()
        ax14.set_title('R_ELBOW_Y')
        # ax14.set_xlabel('Timestep')
        ax14.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax14.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax14.plot(x,o)

        p = [float(line.split(' ')[14]) for line in lines]
        ax15.clear()
        ax15.set_title('R_WRIST_P')
        # ax15.set_xlabel('Timestep')
        ax15.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax15.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax15.plot(x,p)

        q = [float(line.split(' ')[15]) for line in lines]
        ax16.clear()
        ax16.set_title('R_WRIST_Y')
        # ax16.set_xlabel('Timestep')
        ax16.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax16.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax16.plot(x,q)

        r = [float(line.split(' ')[16]) for line in lines]
        ax17.clear()
        ax17.set_title('L_CROTCH_P')
        # ax17.set_xlabel('Timestep')
        ax17.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax17.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax17.plot(x,r)

        s = [float(line.split(' ')[17]) for line in lines]
        ax18.clear()
        ax18.set_title('L_KNEE_P')
        # ax17.set_xlabel('Timestep')
        ax18.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax18.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax18.plot(x,s)

        t = [float(line.split(' ')[19]) for line in lines]
        ax19.clear()
        ax19.set_title('R_CROTCH_P')
        # ax18.set_xlabel('Timestep')
        ax19.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax19.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax19.plot(x,t)

        u = [float(line.split(' ')[20]) for line in lines]
        ax20.clear()
        ax20.set_title('R_KNEE_P')
        # ax17.set_xlabel('Timestep')
        ax20.set_ylabel('Joint Angle (Rad)')
        # if x[len(x)-1] > 50:
        #     ax20.set_xlim([x[len(x)-1]-50,x[len(x)-1]])
        ax20.plot(x,u)
        
        # y = [1*159.3*j for j in y]
        # plt.ylabel('HC')
        
        # r = 0
        # if len(y) % 10 == 0:
        #     r = len(y)
        
        
        # print(len(y),"\n")
        # ax1.plot(x[len(x)-1],y[len(y)-1])
        # print(len(x))
        
        # print(len(a), " ", len(y))
            # print(x, " ", y[1])
            # x += 1
        
        # print(y[len(y)-1])

    # graph_data = open('example.txt','r').read()
    # lines = graph_data.split('\n')
    # xs = []
    # ys = []
    # for line in lines:
    #     if len(line) > 1:
    #         x, y = line.split(',')
    #         xs.append(float(x))
    #         ys.append(float(y))
    # ax1.clear()
    # ax1.plot(xs, ys)
ani = animation.FuncAnimation(fig, animate, interval=10000)
plt.show()

