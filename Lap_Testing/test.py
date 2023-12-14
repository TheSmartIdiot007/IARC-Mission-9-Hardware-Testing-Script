import pickle
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from math import sqrt

def max(list):
    m = abs(list[0])
    for i in range(len(list)-1):
        if (abs(list[i+1] > m)):
            m = abs(list[i+1])
    return m


if __name__ == '__main__':

    pickle_off = open("1D.txt", "rb")    #("Documents/IARC/5D10.txt", "rb")
    gen = pickle.load(pickle_off)
    print(gen)

    x = gen[2]
    y = gen[5]

    print("max in x = ", max([max(x), -min(x)]))
    print("max in y = ", max([max(y), -min(y)]))
    
    fig_vx = plt.figure(figsize=(10,8))
    ax_vx = plt.axes(projection = '3d')

    ax_vx.plot3D(gen[0], gen[3], [0]*len(gen[0]), linestyle = '-', marker = '.', color = 'red')
    ax_vx.plot3D(gen[0], gen[3], gen[2], linestyle = '-', marker = '.', color = 'lime')
    ax_vx.plot3D(gen[0], gen[3], gen[5], linestyle = '-', marker = '.', color = 'teal')

    ax_vx.set_title('Accelerations').set_fontsize(20)
    ax_vx.set_xlabel('$x$').set_fontsize(20)
    ax_vx.set_ylabel('$y$').set_fontsize(20)
    ax_vx.set_zlabel('$a$').set_fontsize(20)
    ax_vx.axes.set_xlim3d(left=-10, right=10)
    plt.legend(['Planned Path','Desired $a_x$','Desired $a_y$'], fontsize = 14)

    plt.show()

    speed = []
    for i in range(len(gen[1])):
        speed.append(sqrt(gen[1][i]**2 + gen[4][i]**2))

    fig_vx = plt.figure(figsize=(10,8))
    ax_vx = plt.axes(projection = '3d')

    ax_vx.plot3D(gen[0], gen[3], [0]*len(gen[0]), linestyle = '-', marker = '.', color = 'red')
    ax_vx.plot3D(gen[0], gen[3], gen[1], linestyle = '-', marker = '.', color = 'lime')
    ax_vx.plot3D(gen[0], gen[3], gen[4], linestyle = '-', marker = '.', color = 'teal')
    ax_vx.plot3D(gen[0], gen[3], speed, linestyle = '-', marker = '.', color = 'cyan')

    ax_vx.set_title('Velocities').set_fontsize(20)
    ax_vx.set_xlabel('$x$').set_fontsize(20)
    ax_vx.set_ylabel('$y$').set_fontsize(20)
    ax_vx.set_zlabel('$v$').set_fontsize(20)
    ax_vx.axes.set_xlim3d(left=-10, right=10)
    plt.legend(['Planned Path','Desired $v_x$','Desired $v_y$', 'Desired speed'], fontsize = 14)

    plt.show()
