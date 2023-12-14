import numpy as np
from scipy.optimize import minimize
from scipy.signal import cont2discrete
import matplotlib.pyplot as plt

# Define system matrices
T_eng = 0.460
K_eng = 0.732
A_f = -1 / T_eng
B_f = K_eng / T_eng
C = np.eye(3)
T_hw = 3
Ts = 0.05

A_continuous = np.array([[0, 1, -T_hw], [0, 0, -1], [0, 0, A_f]])
B_continuous = np.array([[0], [0], [B_f]])
C_continuous = np.eye(3)
D_continuous = 0

sys_continuous = (A_continuous, B_continuous, C_continuous, D_continuous)
sys_discrete = cont2discrete(sys_continuous, Ts, method='zoh')

A, B, _, _,_ = sys_discrete

x_in = [50,5,0]
x_in = np.array(x_in)
x_in = x_in.reshape(3,1)

print(x_in)
# Define the curve (polynomial in this case)
def curve(x, coeffs):
    return np.polyval(coeffs, x)

Q = np.eye(3)
R = np.eye(1)

def gradient_penalty(y_values):
    return np.sum(np.diff(y_values)**2)

# Approximate gradient using finite differences
def grad_curve(x, coeffs, delta=1e-5):
    return (curve(x + delta, coeffs) - curve(x, coeffs)) / delta

# Numerical gradient calculation
def numerical_gradient(f, x, epsilon=1e-5):
    return (f(x + epsilon) - f(x - epsilon)) / (2 * epsilon)

# Define the cost function
Np  = 30
y = np.zeros((Np+1,1))


def cost_function(coeffs, x_in):

    costlist = 0.0
    costlist += np.squeeze(x_in.T @ Q @ x_in)
    y[0] = x_in[1]
    for t in range(Np):
        u = curve(x_in[1], coeffs)
        # print("Shape of u:", u.shape)
        u = np.array(u[0]).flatten()
        # print(f"this U {u} and  this is shape of U {u.shape}")
        # print("Shape of R:", R.shape)
        costlist += u.T @R @ u
        print(f"this is shape of a{A.shape} and B {B.shape}")
        print(f"this is x {x_in} and shape of x {x_in.shape}")
        x_in = A @ x_in + B@u
        x_in = x_in[0]
        x_in = np.array(x_in)
        x_in = x_in.reshape(3,1)
        print(f"this is x {x_in} and shape of x {x_in.shape}")
        y[t+1] = x_in[1]
        costlist += np.squeeze(x_in.T @ Q @ x_in)
        grd_x = numerical_gradient(lambda x: curve(x, coeffs), x_in[1])
        costlist += grd_x**2

    # costlist += x_in.T@Q@x_in
    # costlist += gradient_penalty(y)

    total_cost = costlist[0]
    print(f"this is costlist {costlist} and this is shape of costlist {costlist.shape}")

    return total_cost

# Function value bounds
lower_bound = -1.0
upper_bound = 1.0

# Constraints: Function values within bounds
constraint = ({'type': 'ineq', 'fun': lambda coeffs: curve(x_in[1], coeffs) - lower_bound},
              {'type': 'ineq', 'fun': lambda coeffs: upper_bound - curve(x_in[1], coeffs)},)

#Initial guess
initial_guess = np.zeros(6)

# Optimize with constraints
result = minimize(lambda coeffs: cost_function(coeffs, x_in), initial_guess, constraints=constraint, method='SLSQP')

# Extract optimized coefficients
optimized_coeffs = result.x
# u = curve(result.x , x_in)
# u = np.array(u)
# x_in = A@x_in + B@u
# initial_guess = result.x
#
# initial_guess = result.x
# result2 = minimize(lambda coeffs: cost_function(coeffs, x_in), initial_guess, constraints=constraint, method='SLSQP')
#

print("Optimized Coefficients:", optimized_coeffs)


# Generate x values for plotting
# x_values = np.linspace(-2, 2, 1000)
#
# # Calculate corresponding y values using the Laguerre function
# y_values = laguerre_function(x_values, optimized_laguerre_coeffs)
#
# # Plot the Laguerre curve
# plt.plot(x_values, y_values, label='Laguerre Curve')
# plt.xlabel('x')
# plt.ylabel('Laguerre Function Value')
# plt.title('Optimized Laguerre Curve')
# plt.legend()
# plt.grid(True)
# plt.show()



# Define Laguerre function with an initial slope of 0
# def laguerre_with_initial_slope(x, n, alpha):
#     laguerre = genlaguerre(n, alpha)
#     integral_result = np.trapz(x * laguerre(x), x)
#     constant = -1/ integral_result
#     return constant * laguerre(x)
#
# # Define cost function with Laguerre and slope penalty
# def cost_function(params, laguerre_order, laguerre_alpha, slope_penalty_weight):
#     laguerre_params = params[:-1]
#     slope_penalty = slope_penalty_weight * params[-1] ** 2  # Penalty on the initial slope
#
#     # Calculate Laguerre values
#     x_values = np.linspace(0, 1, 10 )
#     laguerre_values = laguerre_with_initial_slope(x_values, laguerre_order, laguerre_alpha)
#
#     # Your specific cost function (modify this part)
#     # For example, you can define a quadratic cost based on the difference between desired and actual Laguerre values
#     desired_values = np.sin(0.5 * np.pi * x_values)  # Example desired values
#     laguerre_cost = np.trapz((laguerre_values - desired_values) ** 2, x_values)
#
#     return laguerre_cost + slope_penalty
#
# # Example MPC optimization
# laguerre_order = 3
# laguerre_alpha = 0.85
# slope_penalty_weight = 1.0
#
# # Initial guess for Laguerre parameters
# initial_guess = np.ones(laguerre_order + 1)
#
# # Optimize the Laguerre parameters
# result = minimize(cost_function, initial_guess, args=(laguerre_order, laguerre_alpha, slope_penalty_weight))
#
# # Extract the optimized Laguerre parameters
# optimized_laguerre_params = result.x[:-1]
#
# # Generate the final Laguerre function with the optimized parameters
# final_laguerre_function = lambda x: laguerre_with_initial_slope(x, laguerre_order, laguerre_alpha)
#
# # Plot the final Laguerre function
# import matplotlib.pyplot as plt
#
# x_values = np.linspace(0, 1, 10)
# plt.plot(x_values, final_laguerre_function(x_values), label='Optimized Laguerre Function')
# plt.xlabel('x')
# plt.ylabel('Function Value')
# plt.legend()
# plt.show()
