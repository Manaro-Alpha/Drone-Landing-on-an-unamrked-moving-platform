from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt
import math

x0 = 0
y0 = 8
x1 = 0
y1 = 0
x2 = 9
y2 = 0
#x = []
#y = []
def getSpline(xi,yi,xf,yf):
    global x1
    global y1
    #global x
    #global y
    x1 = math.sqrt(((xi**2) + (xf**2))/2)
    y1 = math.sqrt(((yi**2) + (yf**2))/2)

    x = [xi,x1,xf]
    y = [yi,y1,yf]

    return CubicSpline(x, y, bc_type='natural')

if __name__ == '__main__':
    i = 0
    x_new = []
    y_new = []
    x0_new = []
    y0_new = []
    x1_new = []
    y1_new = []
    x2_new = []
    y2_new = []
    for i in range(0,12):
        f = getSpline(x0,y0,x2,y2)
        x = np.linspace(x0, x1, 10)
        y = f(x)
        #y1_new.append(y_new)
        #y_new.append(f(x_new))
        #print(x_new,y_new)
        #plt.figure(figsize = (10,8))
        #plt.plot(x_new, y_new, 'b')
        #plt.plot(x_new, y_new, 'ro')
        x0_new.append(x0)
        y0_new.append(y0)
        x1_new.append(x1)
        y1_new.append(y1)
        x2_new.append(x2)
        y2_new.append(y2)
        x_new.append(x)
        y_new.append(y)
        plt.plot(x0_new, y0_new, 'ro')
        plt.plot(x1_new, y1_new, 'ro')
        plt.plot(x2_new, y2_new, 'ro')
        plt.plot(x, y, 'b')
        x0 = x1
        y0 = y1
        x2 += 0.5
        i+=1
        
        
    #y_new = f(x_new)
    #plt.figure(figsize = (10,8))
    #plt.plot(x_new, y_new, 'b')
    #plt.plot(x0_new, y0_new, 'ro')
    #plt.plot(x1_new, y1_new, 'ro')
    #plt.plot(x2_new, y2_new, 'ro')
    #plt.title('Cubic Spline Interpolation')
    #plt.xlabel('x')
    #plt.ylabel('y')
    plt.show()
    #print(x_new)
    #print(y1_new)