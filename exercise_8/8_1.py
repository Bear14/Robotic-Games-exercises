from casadi import *

if __name__ == '__main__':
    x = SX.sym('x')
    y = SX.sym('y')
    th = SX.sym('th')
    x = vertcat(x, y, th)

    uv = SX.sym('uv')
    uw = SX.sym('uw')
    p = vertcat(uv, uw)

    g1 = cos(th) * uv
    g2 = sin(th) * uv
    g3 = uw
    ode = vertcat(g1, g2, g3)
    dae = {'x': x, 'p': p, 'ode': ode}

    opts = {'t0': 0, 'tf': 2}
    F = integrator('F', 'idas', dae, opts)

    bla = (0, 0, pi)
    blub = (1, 1)

    print("xf: ", F(x0=bla, p=blub)['xf'])
