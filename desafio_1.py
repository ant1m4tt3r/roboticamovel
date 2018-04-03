# pylint: disable=C0321, C0103, C0111, E1101, W0621, I0011
import math
import numpy as np
import matplotlib.pyplot as plt

KP = 0.01
KI = 0.01
MIN_ERROR = 0.01

integ = 0


# Normaliza
def normalizer(arr):
    norm = np.linalg.norm(arr)
    if norm == 0:
        return arr
    return arr / norm

def integrator(old_integ, T, err, old_error):
    integ = old_integ + KI * T * calc_module(err + old_error) / 2
    return integ

def proportional(err):
    return KP * calc_module(err)

# Integrador
def get_pos(current_pos, final_pos, u, ts):
    desloc = u * ts
    pos = current_pos + desloc
    return pos

def get_error(final_pos, current_pos):
    return final_pos - current_pos

# Obtem a velocidade
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
        # plot(current_pos)

    print(len(positions))
    positions = np.array(positions)
    plot(positions)

def plot(data):
    print(data)
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


def distance(t1, t2):
    return ((t2[0] - t1[0])**2 + (t2[1] - t1[1])**2)**(0.5)


def angle(t1, t2):
    return math.atan2(t2[1] - t1[1], t2[0] - t1[0])


p0 = np.array([0, 0])
pf = np.array([10, 10])
ta = 0
v =  1
ts = 0.2
controller(p0, pf, ta, v, ts)
