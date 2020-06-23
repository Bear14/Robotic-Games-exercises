from casadi import *
import numpy as np
N = 100  # number of control intervals
M= 10    # number of updates per control interval

opti = Opti() # Use Opti Stack

# ---- Initialize Variables ---------
X = opti.variable(4,N+1) # state
x = X[0,:]
y = X[1,:]
theta = X[2,:]
U = opti.variable(2,N)   # control trajectory (throttle)
T = opti.variable()      # final time

# ---- objective          ---------
opti.minimize(T) # race in minimal time

# ---- dynamic constraints --------
def f(x,u):
        return vertcat(cos(x[2])*x[3],
                       sin(x[2])*x[3],u[1],u[0])

dt = T/N # length of a control interval
delta_t = dt/M

# Simulate System
#----------------------------------------------
for k in range(N): # loop over control intervals
    x_curr=X[:,k]
    for j in range(M): #loop over updates in control interval
        x_curr = x_curr+ delta_t*f(x_curr,   U[:,k])
    opti.subject_to(X[:,k+1]==x_curr)


# ---- path constraints -----------
###################################
#   Add limited control here      #
###################################
#Lenkradius beschraunken
opti.subject_to(X[2,:] >= -3)
opti.subject_to(X[2,:] <= 3)
#Geschwindichkeit beschraunken
opti.subject_to(U[0,:] <= 1)
opti.subject_to(U[0,:] >= -1)

# opti.subject_to(X[0,:] >= -0.1)

#racing circuit parameter:
rx=5
ry=1
width=1
###################################
#   Add curve constraints here    #
###################################
opti.subject_to(X[0,:] >= rx cos(T))
opti.subject_to(X[1,:] >= ry cos(T))

# #inner bound X
# opti.subject_to(X[0,:] >= sqrt(rx**2*(1-X[1,:]**2/ry**2)))
# opti.subject_to(X[1,:] >= sqrt(ry**2*(1-X[0,:]**2/rx**2)))
# #outer bound
# opti.subject_to(X[0,:] <= sqrt((rx+width)**2*(1-X[1,:]**2/(ry+width)**2)))
# opti.subject_to(X[1,:] <= sqrt((ry+width)**2*(1-X[0,:]**2/(rx+width)**2)))


# ---- boundary conditions --------

opti.subject_to(X[3,0]==0)
opti.subject_to(x[0]==0)
opti.subject_to(y[0]==(ry+0.5*width))
opti.subject_to(theta[0]==0)

opti.subject_to(x[-1]==0)
opti.subject_to(y[-1]==-(ry+0.5*width))
opti.subject_to(X[3,-1]==0)

# ---- misc. constraints  ----------
opti.subject_to(T>=0) # Time must be positive

# ---- initial guesses for solver ---
opti.set_initial(U[1,:0], 0)
opti.set_initial(T, 1)

# ---- solve NLP              ------
opti.solver("ipopt") # set numerical backend
sol = opti.solve()   # actual solve

# ---- plot results        ------


from pylab import plot, step, figure, legend, show, xlim
t=np.linspace(0,2*np.pi,len(sol.value(x)))
plot(rx*np.cos(t),ry*np.sin(t),label="inner bound")
plot((rx+width)*np.cos(t),(ry+width)*np.sin(t),label="outer bound")
plot(sol.value(x),sol.value(y),label="racecar position")
xlim(left=0)
legend()
show()


time=np.linspace(0,sol.value(T),N+1)
plot(time,sol.value(X[3,:]),label="linear_velocity")
plot(time,sol.value(X[2,:]),label="angular_velocity")
step(time[1:],sol.value(U[0,:]),'k',label="optimal throtle")
step(time[1:],sol.value(U[1,:]),label="optimal steering_angle")
legend(loc="upper left")


show()
