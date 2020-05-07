
import math
import numpy as np

tandelta = math.tan(delta)
angel = yaw + math.atanh((self.lr*tandelta)/self.L)
deno1 = np.tan(delta)**2 + 1
deno2 = (self.lr**2*tandelta**2)/self.L**2 + 1
deno3 = self.L * np.sqrt(deno2)

A_temp = np.array([[ 0, 0, np.cos(angel), -v*np.sin(angel)],
            [ 0, 0, np.sin(angel),  v*np.cos(angel)],
            [ 0, 0, 0, 0],
            [ 0, 0, tandelta/deno3, 0]]) * dt
A_temp = A_temp + np.eye(4)

B_temp = np.array([[ 0, -(self.lr*v*np.sin(angel)*(deno1))/(self.L*(deno2))],
            [ 0, (self.lr*v*np.cos(angel)*(deno1))/(self.L*(deno2))],
            [ 1, 0],
            [ 0, (v*(deno1))/deno3 - (self.lr**2*v*tandelta**2*(deno1))/(deno3**3)]]) * dt