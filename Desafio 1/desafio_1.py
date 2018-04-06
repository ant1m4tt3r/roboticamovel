# pylint: disable=C0321, C0103, C0111, E1101, W0621, I0011
# encoding: UTF-8
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

KP = 0.001
KI = 0.001
MIN_ERROR = 0.01

integ = 0

fig = plt.figure()
plt.ion() #interative mode
plt.grid(True)

ax = fig.add_subplot(111)
ax.set_xlim(-2, 12)
ax.set_ylim(-2, 12)
data, = ax.plot([],[], color='blue', marker='o', markeredgecolor='r')

def animate(i,data,pos,clear):
    if clear:
        # show i-th elements from data
        data.set_xdata(pos[i,0])
        data.set_ydata(pos[i,1])
    else:
        #show 0 to i-th elements from data
        data.set_xdata(pos[:i,0])
        data.set_ydata(pos[:i,1])


# Normaliza
def normalizer(arr):
    norm = np.linalg.norm(arr)
    if norm == 0:
        return arr
    return arr / norm

# Calcula a ação integradora
def integrator(old_integ, T, err, old_error):
    integ = old_integ + KI * T * calc_module(err + old_error) / 2
    return integ

# Calcula a ação proporcional
def proportional(err):
    return KP * calc_module(err)

# Retorna a nova posição (atual + deslocamento)
def get_pos(current_pos, final_pos, u, ts):
    desloc = u * ts
    pos = current_pos + desloc
    return pos

# Retorna o erro atual
def get_error(final_pos, current_pos):
    return final_pos - current_pos

# Obtem a velocidade (com base na ação de controle)
def get_velocity(err, new_err, max_v, T):
    v = proportional(err) + integrator(integ, T, new_err, err)
    if v > max_v:
        return max_v
    return v

# Controlador
def controller(p0, pf, ta, max_v, ts):
    err = pf - p0
    current_pos = p0

    positions = []
    positions.append(np.array([p0[0], p0[1]]))

    while calc_module(err) > MIN_ERROR:
        new_error = get_error(pf, current_pos)
        v = get_velocity(err, new_error, max_v, ts)

        current_pos = get_pos(current_pos, pf, v, ts)

        err = new_error
        positions.append(current_pos)

    return np.array(positions)

def plot(data):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(-2, 12)
    ax.set_ylim(-2, 12)
    # ax.scatter(data[:][0], data[:][1], c='red', marker='x')
    i = 0
    for elm in data:
        i += 1
        if (i % 25 == 0 or i == 0):
            ax.scatter(elm[0], elm[1], c='red', marker='x')
    plt.grid(True)
    plt.show()


def calc_module(arr):
    return math.sqrt(arr[0]**2 + arr[1]**2)


if __name__ == '__main__':
    p0 = np.array([0, 0])
    pf = np.array([10, 10])
    ta = 0
    v =  1
    ts = 2
    pos = controller(p0, pf, ta, v, ts)

    ani = FuncAnimation(fig, animate, np.arange(0, len(pos), 3), fargs=(data,pos,True), interval=1, repeat=False)
    
    #show now
    plt.draw()
    plt.show()
    
    input()