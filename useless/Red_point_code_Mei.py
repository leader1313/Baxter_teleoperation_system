import numpy as np

h0=0.40

d0=0.035

h1=h0
theta=180

d1=h1*d0/h0

dx=d1*np.cos(theta)

dy=d1*np.sin(theta)

print(dx, dy)
