import numpy as np
import tkinter as tk


height = 80

p1 = np.array([40,40,0])
p2 = np.array([40,-40,0])
p3 = np.array([-40,-40,0])
p4 = np.array([-40,40,0])
p5 = np.array([40,40,height])
p6 = np.array([40,-40,height])
p7 = np.array([-40,-40,height])
p8 = np.array([-40,40,height])

# cam position
cam_pos = np.array([100,0,40])

# direction of the camera - nomalized
cam_n1 = np.array([-1,0,0])
cam_n2 = np.array([0,0,1])
cam_n3 = np.array([0,1,0])

thetaX = 0
thetaY = 0
thetaZ = 0

# center of image plane
f = 10									
cam_plane_origin = cam_pos + cam_n1*f	
def modifyNormal():
    global cam_n1,cam_n2,cam_n3,camera_plane_origin
    
    # Camera normal
    tmp_n1 = np.array([-1, 0, 0])

    # Camera Planes
    tmp_n2 = np.array([ 0, 1, 0])
    tmp_n3 = np.array([ 0, 0, 1])
    
    rotMatrixX = np.array([[1,             0,              0],
                      [0,np.cos(thetaX),-np.sin(thetaX)],
                      [0,np.sin(thetaX), np.cos(thetaX)]
                      ])

    rotMatrixY = np.array([[np.cos(thetaY) ,0,np.sin(thetaY)],
                      [0              ,1,             0],
                      [-np.sin(thetaY),0,np.cos(thetaY)]
                      ])

    rotMatrixZ = np.array([[np.cos(thetaZ),-np.sin(thetaZ),0],
                      [np.sin(thetaZ), np.cos(thetaZ),0],
                      [             0,              0,1]
                      ])  
    normal_matrix = np.column_stack((tmp_n1,tmp_n2,tmp_n3))
    normal_matrix = np.matmul(rotMatrixZ, np.matmul(rotMatrixY, np.matmul(rotMatrixX,normal_matrix)))  
    cam_n1, cam_n2, cam_n3 = np.transpose(normal_matrix)
    camera_plane_origin = cam_pos + cam_n1*f
    myCanvas.pack()
    
# make vector length = 1
def normalizeVector(vector):
    scalingfactor = np.linalg.norm(vector)
    if scalingfactor == 0:
        return vector
    else:
        return (vector) * (1/np.linalg.norm(vector))

# find distance between plane and point
def distance(cam, point, n, p0):
    result = np.divide(np.dot(p0-point, n), np.dot(normalizeVector(cam-point),n))
    return result

# find coordinates where point vector intersect image plane
def intersect(cam, point, n, p0):
    result = point + distance(cam, point, n, p0)*normalizeVector(cam-point)
    return result

intersect1 = intersect(cam_pos, p1, cam_n1, cam_plane_origin)
# print('Intersect 1')
# print(intersect1)

point_list = [p1,p2,p3,p4,p5,p6,p7,p8]

#--------- GUI ---------#

canvasWidth  = 500
canvasHeight = 500
minN = -0.2
maxN =  0.2
root = tk.Tk()
root.attributes('-fullscreen',True)
myCanvas = tk.Canvas(root, width=canvasWidth, height=canvasHeight)
cam_pos = [60,0,40]

def scaleXplus():
    global cam_pos
    cam_pos[0] = cam_pos[0] + 3
    generateImage()
    
def scaleYplus():
    global cam_pos
    cam_pos[1] = cam_pos[1] + 3
    generateImage()
    
def scaleZplus():
    global cam_pos
    cam_pos[2] = cam_pos[2] + 3
    generateImage()
    
def scaleXminus():
    global cam_pos
    cam_pos[0] = cam_pos[0] - 3
    generateImage()
    
def scaleYminus():
    global cam_pos
    cam_pos[1] = cam_pos[1] - 3
    generateImage()
    
def scaleZminus():
    global cam_pos
    cam_pos[2] = cam_pos[2] - 3
    generateImage()

def rotateX():
    global thetaX
    if thetaX < maxN :          
        thetaX += 0.02
    else:
        thetaX = maxN+0.02
    modifyNormal()
    generateImage()
    
def rotateY():
    global thetaY
    if thetaY < maxN :
        thetaY += 0.02
    else:
        thetaY = maxN+0.02
    modifyNormal()
    generateImage()
    
def rotateZ():
    global thetaZ
    if thetaZ < maxN :
        thetaZ += 0.02
    else:
        thetaZ = maxN+0.02
    modifyNormal()
    generateImage()
def rotateX2():
    global thetaX
    if thetaX > minN :
        thetaX -= 0.02
    else:
        thetaX = minN-0.02
    modifyNormal()
    generateImage()
    
def rotateY2():
    global thetaY
    if thetaY > minN :
        thetaY -= 0.02
    else:
        thetaY = minN-0.02
    modifyNormal()
    generateImage()
    
def rotateZ2():
    global thetaZ
    if thetaZ > minN :
        thetaZ -= 0.02
    else:
        thetaZ = minN-00.02
    modifyNormal()
    generateImage()


buttonX1 = tk.Button(root,text='X increase',command = scaleXplus, fg = "dark green", bg = "white").pack()
buttonX2 = tk.Button(root,text='X decrease',command = scaleXminus, fg = "dark green", bg = "white").pack()
buttonY1 = tk.Button(root,text='Y increase',command = scaleYplus, fg = "dark green", bg = "white").pack()
buttonY2 = tk.Button(root,text='Y decrease',command = scaleYminus, fg = "dark green", bg = "white").pack()
buttonZ1 = tk.Button(root,text='Z increase',command = scaleZplus, fg = "dark green", bg = "white").pack()
buttonZ2 = tk.Button(root,text='Z decrease',command = scaleZminus, fg = "dark green", bg = "white").pack()

buttonRX1 = tk.Button(root,text='Rotate X +', command = rotateX, fg = "dark green", bg = "white").pack()
buttonRY1 = tk.Button(root,text='Rotate Y +', command = rotateY, fg = "dark green", bg = "white").pack()
buttonRZ1 = tk.Button(root,text='Rotate Z +', command = rotateZ, fg = "dark green", bg = "white").pack()
buttonRX2 = tk.Button(root,text='Rotate X -', command = rotateX2, fg = "dark green", bg = "white").pack()
buttonRY2 = tk.Button(root,text='Rotate Y -', command = rotateY2, fg = "dark green", bg = "white").pack()
buttonRZ2 = tk.Button(root,text='Rotate Z -', command = rotateZ2, fg = "dark green", bg = "white").pack()
        
def generateImage():
    global cam_pos
    global cam_plane_origin
    global cam_n1, cam_n2, cam_n3
    global point_list
    global f
    pList = []
    myCanvas.delete("all")
    origin = np.array([canvasWidth/2, canvasHeight/2])
    dotSize = 2
    
    myCanvas.create_oval(
        origin[0]-dotSize,
        origin[1]-dotSize,
        origin[0]+dotSize,
        origin[1]+dotSize,
        fill='#f00')
    
    
    for point in point_list:
        # distance 
        # distancePlane 	=  distance(cam_pos, point, cam_n1, cam_plane_origin)
        intersectPlane 	= intersect(cam_pos, point, cam_n1, cam_plane_origin)

        magnitude = np.linalg.norm(intersectPlane - cam_plane_origin)

        angleY = np.arccos(np.dot(cam_n2, normalizeVector(intersectPlane - cam_plane_origin)))
        angleX = np.arccos(np.dot(cam_n3, normalizeVector(intersectPlane - cam_plane_origin)))
        projectionLengthY = magnitude*np.cos(angleY)
        projectionLengthX = magnitude*np.cos(angleX)
        coordinate = (origin[0]+ projectionLengthX), (origin[1]+ projectionLengthY)
        pList.append(coordinate)
        myCanvas.create_oval(
            origin[0]+projectionLengthX-dotSize, 
            origin[1]+projectionLengthY-dotSize, 
            origin[0]+projectionLengthX+dotSize, 
            origin[1]+projectionLengthY+dotSize)
        
    
    myCanvas.create_line(pList[0][0],pList[0][1],pList[1][0],pList[1][1])
    myCanvas.create_line(pList[1][0],pList[1][1],pList[2][0],pList[2][1])
    myCanvas.create_line(pList[2][0],pList[2][1],pList[3][0],pList[3][1])
    myCanvas.create_line(pList[3][0],pList[3][1],pList[0][0],pList[0][1])
    
    myCanvas.create_line(pList[4][0],pList[4][1],pList[5][0],pList[5][1])
    myCanvas.create_line(pList[5][0],pList[5][1],pList[6][0],pList[6][1])
    myCanvas.create_line(pList[6][0],pList[6][1],pList[7][0],pList[7][1])
    myCanvas.create_line(pList[7][0],pList[7][1],pList[4][0],pList[4][1])
    
    myCanvas.create_line(pList[0][0],pList[0][1],pList[4][0],pList[4][1])
    myCanvas.create_line(pList[1][0],pList[1][1],pList[5][0],pList[5][1])
    myCanvas.create_line(pList[2][0],pList[2][1],pList[6][0],pList[6][1])
    myCanvas.create_line(pList[3][0],pList[3][1],pList[7][0],pList[7][1])
    myCanvas.pack()

generateImage()
root.mainloop()