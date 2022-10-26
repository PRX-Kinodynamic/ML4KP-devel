import numpy as np 
import matplotlib.pyplot as plt

np.random.seed(210896)

order_min = 1
order_max = 2

speed_min = 0.5
speed_max = 1.0

std_min = 0.01
std_max = 0.1

n_order = np.random.choice(np.arange(order_min,order_max+1))

# Generate n_order + 1 random (x,y) pairs in [-10,10]
x_rand = np.random.uniform(-10,10,n_order+1)
y_rand = np.random.uniform(-10,10,n_order+1)

# Fit polynomial
p = np.polyfit(x_rand,y_rand,n_order)

xs = []
ys = []

for x in np.linspace(-10,10,21):
    y = np.polyval(p,x)
    if y > -10 and y < 10:
        xs.append(x)
        ys.append(y)
xs = np.array(xs)
ys = np.array(ys)

plt.figure(figsize=(8,8))
plt.xlim(-10,10)
plt.ylim(-10,10)
plt.scatter(x_rand,y_rand,color='red',marker='.')

# Plot the polynomial
plt.plot(xs,ys,color='blue')

plt.show()
