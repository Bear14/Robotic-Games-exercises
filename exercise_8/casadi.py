from casadi import *
from math import cos, sin

x = SX.sym('x')
y = SX.sym('y')
th = SX.sym('th')
state = vertcat(x, y, th)

uv = SX.sym('uv') #rad
uw = SX.sym('uw') #Geschwindichkeit
para = vertcat(uv, uw)

g1 = cos(th)*uv
g2 = sin(th)*uv
g3 = uw
diff = vertcat(g1, g2, g3)

dae = {'x': state, 'p': para, 'ode': diff}

if __name__ == '__main__':
    print(x)
    pass
