from numpy import array
import numpy as np
np.set_printoptions(precision=4)

from math import sin, cos



def G(q0, q1, q2):
    return array([[0], [-28.1878497635456*sin(q1)*sin(q2) + 28.1878497635456*cos(q1)*cos(q2) + 103.355449133001*cos(q1)], [-28.1878497635456*sin(q1)*sin(q2) + 28.1878497635456*cos(q1)*cos(q2)]])
