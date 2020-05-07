
import math
import numpy as np

tandelta = math.tan(delta)
angel = psi + math.atanh((lr*tandelta)/L)
deno1 = tan(delta)^2 + 1
deno2 = (lr^2*tandelta^2)/L^2 + 1

A = np.array([[ 0, 0, np.cos(angel), -v*np.sin(angel)],
              [ 0, 0, np.sin(angel),  v*np.cos(angel)],
              [ 0, 0, 0, 0],
              [ 0, 0, tandelta/(L*(deno2)^(1/2)), 0]]

B = np.array([[ 0, -(lr*v*np.sin(angel)*(deno1))/(L*(deno2))],
              [ 0, (lr*v*np.cos(angel)*(deno1))/(L*(deno2))],
              [ 1, 0],
              [ 0, (v*(deno1))/(L*(deno2)^(1/2)) - (lr^2*v*tandelta^2*(deno1))/(L^3*(deno2)^(3/2))]]