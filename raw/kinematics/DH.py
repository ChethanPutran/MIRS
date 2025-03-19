import numpy as np

def DH(alpha, a, d, theta):
     a11 = np.cos(theta)
     a12 = -np.sin(theta)*round(np.cos(alpha),2)
     a13 = np.sin(theta)*round(np.sin(alpha),2)
     a14 = a*np.cos(theta)

     a21 = np.sin(theta)
     a22 = np.cos(theta)*round(np.cos(alpha),2)
     a23 = -np.cos(theta)*round(np.sin(alpha),2)
     a24 = a*np.sin(theta)

     a31 = 0
     a32 = round(np.sin(alpha),2)
     a33 = round(np.cos(alpha),2)
     a34 = d

     a41 = 0
     a42 = 0
     a43 = 0
     a44 = 1


     T = [[a11, a12, a13, a14],
          [a21, a22, a23, a24],
          [a31, a32, a33, a34],
          [a41, a42, a43, a44]]
     
     return T
 


