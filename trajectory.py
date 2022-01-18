import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import copy

def search_neighbors(op, brmatrix, se_points, visited_matrix, se_points_original):
    x = op[0]
    y = op[1]
    neighbors = []
    neighbors.append([x-1,y-1])
    neighbors.append([x-1,y])
    neighbors.append([x-1,y+1])
    neighbors.append([x,y-1])
    neighbors.append([x,y+1])
    neighbors.append([x+1,y-1])
    neighbors.append([x+1,y])
    neighbors.append([x+1,y+1])
    #print(neighbors)
    for i in range(8):
        if brmatrix[neighbors[i][0]][neighbors[i][1]] == 1:
            if visited_matrix[neighbors[i][0]][neighbors[i][1]] == 0:
                #print("found branch")
                return [neighbors[i][0],neighbors[i][1]],0
        

    for i in range(8):
        for j in range(len(se_points_original)):
            if  (neighbors[i][0] == se_points_original[j][0]) & (neighbors[i][1] == se_points_original[j][1]):
                return [neighbors[i][0],neighbors[i][1]],1


    return [-1,-1],2
    



img = cv.imread('snoopy2.jpg',cv.IMREAD_GRAYSCALE)
#RGB_img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
img = cv.bitwise_not(img)

#canny edge detection
edges = cv.Canny(img,100,200)
#edges = cv.bitwise_not(edges)



# Displaying the final skeleton
#cv.imshow("Skeleton",skel)
#cv.waitKey(0)
#cv.destroyAllWindows()"""


skel = edges
plt.imshow(skel, cmap='gray', vmin=0, vmax=1)
plt.show()
#cv.imshow("edges",skel)
#cv.waitKey(0)
#cv.destroyAllWindows()

#size of image
n = len(skel) #height
m = len(skel[0]) #width
print(n,m)

#search for start/end points and branch points
#print(skel[0][0])
se_points = []
branch_points = []
brmatrix = []
#inittialize brmatrix
for i in range(n):
    for j in range(m):
        if j == 0:
            brmatrix.append([0])
        else:
            brmatrix[i].append(0)
#print(brmatrix[0][20])

test = []
#inittialize test
for i in range(n):
    for j in range(m):
        if j == 0:
            test.append([0])
        else:
            test[i].append(0)

for i in range(1,n-1):
    for j in range(1,m-1):
        if skel[i][j] == 255:
            #print(skel[i][j])
            #print("hi")
            neighbors = []
            neighbors.append(skel[i-1][j-1])
            neighbors.append(skel[i-1][j])
            neighbors.append(skel[i-1][j+1])
            neighbors.append(skel[i][j-1])
            neighbors.append(skel[i][j+1])
            neighbors.append(skel[i+1][j-1])
            neighbors.append(skel[i+1][j])
            neighbors.append(skel[i+1][j+1])
            amount = 0
            for k in range(8):
                if neighbors[k] == 255:
                    amount += 1
            if amount == 1:
                se_points.append([i,j])
            if amount > 1:
                branch_points.append([i,j])
                brmatrix[i][j] = 1
            """if (i == 60) & (j == 77):
                print(amount)
                print(neighbors)"""           
        


if len(se_points) == 0:
    if len(branch_points) != 0:
        se_points.append(branch_points[0])
    else:
        print("Empty picture")


#write to file to check

fp = open("check_startend.txt","w")
msg = ""
width = 2
for i in range(len(se_points)):
    """test[se_points[i][0]][se_points[i][1]] = 255
    for k in range(2*width +1):
        for j in range(2*width + 1):
            try:
                test[se_points[i][0] - width + k][se_points[i][1] - width + j] = 255
            except ValueError:
                 pass"""
    msg += str(se_points[i][0]) + " " + str(se_points[i][1]) + "\n"
fp.write(msg)
fp.close()

test = np.array(test)
#plt.imshow(test, cmap='gray', vmin=0, vmax=255)
#plt.show()

msg = ""
fp = open("check_branch.txt","w")

for i in range(len(branch_points)):
    msg += str(branch_points[i][0]) + " " + str(branch_points[i][1]) + "\n"
fp.write(msg)
fp.close()

#form trajectories
print("searching paths")
fp = open("path.txt","w")
se_points_original = copy.deepcopy(se_points)
msg = ""

drawn_matrix = []
for i in range(n):
    for j in range(m):
        if j == 0:
            drawn_matrix.append([0])
        else:
            drawn_matrix[i].append(0)


drawn_width_matrix = []
for i in range(n):
    for j in range(m):
        if j == 0:
            drawn_width_matrix.append([0])
        else:
            drawn_width_matrix[i].append(0)

loop = 0
path_number = 0
width = 1 #pen width
while ((len(se_points) != 0) & (loop < 400)):
    loop += 1
    #print("start")
    visited_matrix = []
    for i in range(n):
        for j in range(m):
            if j == 0:
                visited_matrix.append([0])
            else:
                visited_matrix[i].append(0)
    path = []
    path.append(se_points[0])
    c = 0
    next_point = se_points[0]
    visited_matrix[next_point[0]][next_point[1]] = 1
    for i in range(2*width +1):
        for j in range(2*width + 1):
            try:
                drawn_matrix[next_point[0]][next_point[1]] = 1
                drawn_width_matrix[next_point[0] - width + i][next_point[1] - width + j] = 1
            except ValueError:
                 pass
    
    while c == 0:


        next_point, c = search_neighbors(next_point,brmatrix,se_points, drawn_matrix,se_points_original)
        #print(next_point)
        if c==2:
            try:
                branch_points = [subl for subl in branch_points if ((subl[0] != next_point[0])or(subl[1] != next_point[1]))]
            except ValueError:
                pass
            break
        
        
        for i in range(2*width +1):
            for j in range(2*width + 1):
                try:
                    drawn_matrix[next_point[0]][next_point[1]] = 1
                    drawn_width_matrix[next_point[0] - width + i][next_point[1] - width + j] = 1
                except ValueError:
                    pass
                

        branch_points = [subl for subl in branch_points if ((subl[0] != next_point[0])or(subl[1] != next_point[1]))]
        #print(len(branch_points))
        #brmatrix[next_point[0]][next_point[1]] = 0
        if visited_matrix[next_point[0]][next_point[1]] == 0: 
            path.append(next_point)
            visited_matrix[next_point[0]][next_point[1]] = 1
        else:
            break
    #print(path)
    #print("done")
    if c == 1:
        branch_points = [subl for subl in branch_points if ((subl[0] != next_point[0])or(subl[1] != next_point[1]))]
        #print("1")
        #print(len(se_points))

    if len(se_points) >= 2:
        #print(next_point)
        se_points.pop(0)
        try:
            #print(next_point)
            se_points = [subl for subl in se_points if ((subl[0] != next_point[0])or(subl[1] != next_point[1]))]
            #print(se_points)
        except ValueError:
            None
    else:
        se_points.pop(0)
        #print(se_points)
    
    if len(path) > 10:
        print("success")
        print(len(path))
        path_number += 1
        #print(loop)
        for i in range(len(path)):
            #print(path[i])
            """if (i == (len(path)-1)):
                msg += str(path[i][0]) + " " + str(path[i][1]) + "\n"
            else:
                msg += str(path[i][0]) + " " + str(path[i][1]) + ","
            """
            msg += str(path[i][0]) + "," + str(path[i][1]) + "\n"
        msg = msg + "\n"


        if 10< path_number <= 20:
            visited_matrix = np.array(visited_matrix)
            for i in range(n):
                for j in range(m):
                    if visited_matrix[i][j] == 1:
                        visited_matrix[i][j] = 255
                    else:
                        visited_matrix[i][j] = 0

            print(path_number)
            plt.subplot(2,5,(path_number-10))
            plt.imshow(visited_matrix, cmap='gray', vmin=0, vmax=255)
            
            #plt.show()
            #plt.imshow(skel, cmap='gray', vmin=0, vmax=1)
            #plt.show()
            #print(path)
            #print(next_point, c)
            pass

        """drawn_matrix = np.array(drawn_matrix)
        for i in range(n):
            for j in range(m):
                if drawn_matrix[i][j] == 1:
                    drawn_matrix[i][j] = 255
                else:
                    drawn_matrix[i][j] = 0

        print(path_number)
        plt.imshow(drawn_matrix, cmap='gray', vmin=0, vmax=255)
        plt.show()
        for i in range(n):
            for j in range(m):
                if drawn_matrix[i][j] == 255:
                    drawn_matrix[i][j] = 1
                else:
                    drawn_matrix[i][j] = 0"""

    if len(se_points) == 0:
        for i in range(len(branch_points)):
            if  drawn_width_matrix[branch_points[i][0]][branch_points[i][1]] == 0:
                #print("activated")
                a = copy.deepcopy([branch_points[i][0],branch_points[i][1]])
                se_points.append([branch_points[i][0],branch_points[i][1]])
                branch_points = [subl for subl in branch_points if ((subl[0] != a[0])or(subl[1] != a[1]))]
                #print(se_points)
                break
    

plt.show()
fp.write(msg)
fp.close()
#print(brmatrix[261][334])
#print(visited_matrix[261][334])
#draw a image to check

"""for i in range(len(branch_points)):
    print(drawn_matrix[branch_points[i][0]][branch_points[i][1]])"""

drawn_matrix = np.array(drawn_matrix)
for i in range(n):
    for j in range(m):
        if drawn_matrix[i][j] == 1:
            drawn_matrix[i][j] = 255
        else:
            drawn_matrix[i][j] = 0
print(loop)
print(path_number)
plt.imshow(drawn_matrix, cmap='gray', vmin=0, vmax=255)
plt.show()
#u8 = cv.convertScaleAbs(drawn_matrix)
#cv.imshow("drawn",u8)
#cv.waitKey(0)
#cv.destroyAllWindows()










