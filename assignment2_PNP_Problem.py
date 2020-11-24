import numpy as np
import tkinter as tk

TK_SILENCE_DEPRECATION=1
# frustum square pyramid points
# pyramid base square
height = 40
point1 = np.array([60,60,-height,1])
point2 = np.array([60,-60,-height,1])
point3 = np.array([-60,60,-height,1])
point4 = np.array([-60,-60,-height,1])

# paramid peak square

point5 = np.array([30,30,height,1])
point6 = np.array([30,-30,height,1])
point7 = np.array([-30,30,height,1])
point8 = np.array([-30,-30,height,1])

# camera orientation angles for rotation matrix
thetaX = -np.pi/2
thetaY = 0
thetaZ = np.pi

# transformation matrix 
Tx = -100
Ty = 120
Tz = 60
Tm = np.array([[Tx],[Ty],[Tz]])

#print("Translation Vector Output:")
#print(str(Tm.round(0)))
# rotation matrix x
Rx = np.array([[1,             0,              0],
                      [0,np.cos(thetaX),-np.sin(thetaX)],
                      [0,np.sin(thetaX), np.cos(thetaX)]
                      ])
# rotation matrix y
Ry = np.array([[np.cos(thetaY) ,0,np.sin(thetaY)],
                      [0              ,1,             0],
                      [-np.sin(thetaY),0,np.cos(thetaY)]
                      ])
# rotation matrix z
Rz = np.array([[np.cos(thetaZ),-np.sin(thetaZ),0],
                      [np.sin(thetaZ), np.cos(thetaZ),0],
                      [             0,              0,1]
                      ])
Rm = np.matmul(Rz,np.matmul(Ry,Rx))

#print("Rotation Matrix Output:")
#print(str(Rm.round(0)))

########## Construct P matrix ##########
# given extrinsic camera matrix (rotation and translation)
cam2origin = np.matmul(Rm, -Tm)
camMatrix = np.concatenate((Rm,cam2origin),axis=1)

#print expected P matrix output
#print("Cam Matrix Output:")
#print(str(camMatrix.round(0)))


camMatrix = np.concatenate((camMatrix,[[0,0,0,1]]))

# given intrinsic Camera Matrix
fx = 40
fy = 40
cx = 0
cy = 0

focalMatrix = np.array([[fx,0,cx,0],
                         [0,fy,cy,0],
                         [0,0,  1,0]
                       ])


# construct the final form of left equation
P = np.matmul(focalMatrix, camMatrix)


pointList = [point1,point2,point3,point4,point5,point6,point7,point8]
pList = []

for point in pointList:
  projectedPoint = np.matmul(P,np.transpose(point))
  projectedPoint[0] = projectedPoint[0]/projectedPoint[2]
  projectedPoint[1] = projectedPoint[1]/projectedPoint[2]
  projectedPoint[2] = projectedPoint[2]/projectedPoint[2]
  #print(projectedPoint)
  pList.append(projectedPoint)

root = tk.Tk()
myCanvas = tk.Canvas(root, width=500, height=500)
myCanvas.delete("all")

origin = np.array([250,250])
dotSize = 1

myCanvas.create_oval(origin[0]-dotSize, origin[1]-dotSize, origin[0]+dotSize, origin[1]+dotSize, fill = "red")

matforsvd = np.array([])
for idx, val in enumerate(pList):
    if idx == 0:
        viewCoor = np.array([[0,-1,val[1]],
                               [1,0,-val[0]],
                               [-val[1],val[0],0]
                               ])
        worldCoor = np.array([[pointList[idx][0],pointList[idx][1],pointList[idx][2],pointList[idx][3],0,0,0,0,0,0,0,0],
                               [0,0,0,0,pointList[idx][0],pointList[idx][1],pointList[idx][2],pointList[idx][3],0,0,0,0],
                               [0,0,0,0,0,0,0,0,pointList[idx][0],pointList[idx][1],pointList[idx][2],pointList[idx][3]]
                              ])
        matforsvd = np.matmul(viewCoor,worldCoor)
    else:
        viewCoor = np.array([[0,-1,val[1]],
                               [1,0,-val[0]],
                               [-val[1],val[0],0]
                              ])
        
        worldCoor = np.array([[pointList[idx][0],pointList[idx][1],pointList[idx][2],pointList[idx][3],0,0,0,0,0,0,0,0],
                               [0,0,0,0,pointList[idx][0],pointList[idx][1],pointList[idx][2],pointList[idx][3],0,0,0,0],
                               [0,0,0,0,0,0,0,0,pointList[idx][0],pointList[idx][1],pointList[idx][2],pointList[idx][3]]
                              ])
        
        matforsvd = np.concatenate((matforsvd,np.matmul(viewCoor,worldCoor)))
    

u, sigma, vt = np.linalg.svd(matforsvd)

# get B from last row of vt

B = np.reshape(vt[11,:],(3,4))


# calculate svd RT from P = in x ex ---> in(-1)P  = ex
calRotMatrixSVD = np.matmul(np.linalg.inv(focalMatrix[:3,:3]),B[:3,:3])

u, sigma, vt =  np.linalg.svd(calRotMatrixSVD)

# Generate Extrinsic Camera Matrix from SVD Output
calRotMatrix = np.matmul(u,vt)
calTransVector = np.matmul(np.linalg.inv(focalMatrix[:3,:3]),np.transpose(B[:,3]))/ sigma[0]
output_cam_matrix = np.append(calRotMatrix,calTransVector[:,None],1)

# Print Extrinsic Camera Matrix generated from SVD

output_cam_matrix = np.concatenate((output_cam_matrix,[[0,0,0,1]]))

# Reconstruct P Matrix from SVD Output
P = np.matmul(focalMatrix, camMatrix)

print("R|T:")
print(output_cam_matrix)

pyramidpointList = [point1,point2,point3,point4,point5,point6,point7,point8]
pyramidPList = []
for point in pyramidpointList:
    projectedPoint = np.matmul(P, np.transpose(point))
    projectedPoint[0] = projectedPoint[0]/projectedPoint[2]
    projectedPoint[1] = projectedPoint[1]/projectedPoint[2]
    projectedPoint[2] = projectedPoint[2]/projectedPoint[2]
    pyramidPList.append(projectedPoint)

dotSize = 1

for p in pyramidPList:
    myCanvas.create_oval(origin[0] +p[0]-dotSize, origin[1]+p[1]-dotSize, origin[0]+p[0]+dotSize, origin[1]+p[1]+dotSize, fill = "blue")


# construct the line
#base line
myCanvas.create_line(origin[0] +pyramidPList[0][0],origin[1] +pyramidPList[0][1],origin[0] +pyramidPList[1][0],origin[1] +pyramidPList[1][1])
myCanvas.create_line(origin[0] +pyramidPList[1][0],origin[1] +pyramidPList[1][1],origin[0] +pyramidPList[3][0],origin[1] +pyramidPList[3][1])
myCanvas.create_line(origin[0] +pyramidPList[3][0],origin[1] +pyramidPList[3][1],origin[0] +pyramidPList[2][0],origin[1] +pyramidPList[2][1])
myCanvas.create_line(origin[0] +pyramidPList[2][0],origin[1] +pyramidPList[2][1],origin[0] +pyramidPList[0][0],origin[1] +pyramidPList[0][1])
#upper line
myCanvas.create_line(origin[0] +pyramidPList[4][0],origin[1] +pyramidPList[4][1],origin[0] +pyramidPList[5][0],origin[1] +pyramidPList[5][1])
myCanvas.create_line(origin[0] +pyramidPList[6][0],origin[1] +pyramidPList[6][1],origin[0] +pyramidPList[7][0],origin[1] +pyramidPList[7][1])
myCanvas.create_line(origin[0] +pyramidPList[5][0],origin[1] +pyramidPList[5][1],origin[0] +pyramidPList[7][0],origin[1] +pyramidPList[7][1])
myCanvas.create_line(origin[0] +pyramidPList[4][0],origin[1] +pyramidPList[4][1],origin[0] +pyramidPList[6][0],origin[1] +pyramidPList[6][1])
#connect base with upper
myCanvas.create_line(origin[0] +pyramidPList[0][0],origin[1] +pyramidPList[0][1],origin[0] +pyramidPList[4][0],origin[1] +pyramidPList[4][1])
myCanvas.create_line(origin[0] +pyramidPList[1][0],origin[1] +pyramidPList[1][1],origin[0] +pyramidPList[5][0],origin[1] +pyramidPList[5][1])
myCanvas.create_line(origin[0] +pyramidPList[2][0],origin[1] +pyramidPList[2][1],origin[0] +pyramidPList[6][0],origin[1] +pyramidPList[6][1])
myCanvas.create_line(origin[0] +pyramidPList[3][0],origin[1] +pyramidPList[3][1],origin[0] +pyramidPList[7][0],origin[1] +pyramidPList[7][1])
myCanvas.pack()
root.mainloop()