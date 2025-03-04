# Max Hughes
# ME 341 HW 6
# March 2025

import scipy
import numpy as np
import matplotlib.pyplot as plt

c3 = 0.37*0.283
c4 = 0.17*0.283
L = 14
F = 6000

tau_max = 13600
sigma_max = 30000
delta_max = 0.25
E = 30*10**6
G = 12*10**6

obj_history = []
cons_history = []

#cost function
def obj(x):
    return (1+c3)*x[0]**2*x[1] + c4*x[2]*x[3]*(L+x[1])

# weld stress
def g1(x):

    tau_prime = F/(2**0.5*x[0]*x[1])
    M = F*(L+x[1]/2)
    R = (x[1]**2/4 + ((x[0]+x[2])/2)**2)**0.5
    J = 2**0.5*x[0]*x[1]*(x[1]**2/12+((x[2]+x[0])/2)**2)
    tau_dprime = M*R/J

    tau = (tau_prime**2 + 2*tau_prime*tau_dprime*x[1]/(2*R) + tau_dprime**2)**0.5

    return tau_max - tau


# normal stress
def g2(x):
    sigma = 6*F*L/(x[3]*x[2]**2)
    return sigma_max - sigma

# slenderness
def g3(x):
    return x[3] - x[0]

# the 4 and 5 supplied are bounds

# buckling
def g6(x):
    I = x[2]*x[3]**3/12
    alpha = G*x[2]*x[3]**3/3
    P_c = 4.013*(E*I*alpha)**0.5*(1-(x[2]/(2*L))*(E*I/alpha)**0.5)/L**2

    return P_c - F

# g7 is a bound

# deformation
def g8(x):
    delta = 4*F*L**3/(E*x[2]**2*x[3])
    return delta_max - delta

# for getting objective value over time
def add_history(x):
    min_cons = min([f(x) for f in (g1, g2, g3, g6, g8)])
    obj_history.append(obj(x))
    cons_history.append(min_cons)

constraints = ({'type': 'ineq', 'fun': g1}, {'type': 'ineq', 'fun': g2}, {'type': 'ineq', 'fun': g3}, {'type': 'ineq', 'fun': g6}, {'type': 'ineq', 'fun': g8})

# from the problem definition
bounds = ((0.125, None),(0, None),(0, None),(None, None))


# A representative set
ICs = [[2,2,2,2], [.1, 21, 1, .5], [.2, 20, 6, .1]]


#plot setup
fig, axs = plt.subplots(2, 2)

axs[0][0].set_xlabel("Iterations")
axs[0][0].set_ylabel("Objective Value")
axs[0][0].title.set_text("SLSQP")
axs[0][0].grid()

axs[1][0].set_xlabel("Iterations")
axs[1][0].set_ylabel("Minmium Constraint Value")
axs[1][0].grid()

axs[0][1].title.set_text("COBLYA")
axs[0][1].set_xlabel("Iterations")
axs[0][1].set_ylabel("Objective Value")
axs[0][1].grid()

axs[1][1].set_xlabel("Iterations")
axs[1][1].set_ylabel("Minmium Constraint Value")
axs[1][1].grid()


# run + plot SLSQP
for IC in ICs:
    print(IC)
    res = scipy.optimize.minimize(obj, IC, method='SLSQP', bounds=bounds,
               constraints=constraints, tol=1e-6, options={'maxiter':1000}, callback=add_history)
    
    axs[0][0].plot(obj_history, label="x0 = ({:.2f}, {:.2f}, {:.2f}, {:.2f})".format(*IC), linewidth=2)
    axs[1][0].plot(cons_history, label="x0 = ({:.2f}, {:.2f}, {:.2f}, {:.2f})".format(*IC), linewidth=2)

    obj_history = []
    cons_history = []
    print(res.x)
    print(res.message)
    print()

# run + plot COBYLA
for IC in ICs:
    print(IC)
    res = scipy.optimize.minimize(obj, IC, method='COBYLA', bounds=bounds,
               constraints=constraints, tol=1e-6, options={'maxiter':1000}, callback=add_history)
    
    axs[0][1].plot(obj_history, label="x0 = ({:.2f}, {:.2f}, {:.2f}, {:.2f})".format(*IC), linewidth=2)
    axs[1][1].plot(cons_history, label="x0 = ({:.2f}, {:.2f}, {:.2f}, {:.2f})".format(*IC), linewidth=2)

    cons_history = []
    obj_history = []
    print(res.x)
    print(res.message)
    print()

# show plots
axs[0][0].legend()
axs[0][1].legend()
axs[1][0].legend()
axs[1][1].legend()
plt.show()

