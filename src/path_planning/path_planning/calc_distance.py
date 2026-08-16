import math
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from scipy.interpolate import PchipInterpolator
# from scipy.interpolate import splprep,splev
import numpy as np 


tree=[(1,1,1,1)]
goal=(9,9)

obstacles=[
    (4,6,3,6),
    (7,8.5,7,9),
    (1,2,6,8)
]

# distance between two points
def distance(x1,y1,x2,y2):
    # return  math.sqrt(  ((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1))  )
    return math.hypot( (x2-x1),((y2-y1)))


#distance between two points( start and goal)
def nearest_point(x,y,x_goal,y_goal):
    return  math.sqrt(  ((x_goal-x)*(x_goal-x)) + ((y_goal-y)*(y_goal-y))  )


# generate random points
def random_point_generator():
    x=round(random.uniform(0,10), 2)
    y=round(random.uniform(0,10), 2)
    return x,y


# if random points greater tehn step_size then steer and select new point as node
def steer(x,y,x_goal,y_goal,step_size):

    if best_distance<=step_size:
        collision=collision_free(x,y, x_goal,y_goal)
        return x_goal,y_goal,collision

    
    else:
        dx=x_goal-x
        dy=y_goal-y
        length=math.hypot(dx, dy)

        unit_x=dx/length
        unit_y=dy/length

        new_x=x+unit_x*step_size
        new_y=y+unit_y*step_size

        collision=collision_free(x,y, new_x,new_y)
        return new_x,new_y,collision



#check of line between two points collide with rectangle
def collision_free(x,y, x_goal,y_goal, xmin=4,xmax=6, ymin=3,ymax=7):
    # new_x,new_y=steer(x,y, x_goal,y_goal)

    for i in range(101):
        t=i/100
        line_x = x + t*( x_goal - x)
        line_y = y + t*(y_goal - y)

        for xmin,xmax,ymin,ymax in obstacles:
            if xmin<=line_x<=xmax and ymin<=line_y<=ymax:
                    return False

    return True

def point_collision(x,y):
    for xmin,xmax,ymin,ymax in obstacles:
        if xmin<=x<=xmax and ymin<=y<=ymax:
            return True
    return False

#points
points=[(1,1), (3,2) ,(5,5), (8,3)]
#target


fig, ax = plt.subplots()
ax.set_xlim(0,10)
ax.set_ylim(0,10)
line, = ax.plot([], [])  # empty line to start

for xmin,xmax,ymin,ymax in obstacles:
    rect = patches.Rectangle((xmin,ymin), xmax-xmin, ymax-ymin, color='red', alpha=0.4)
    ax.add_patch(rect)

ax.scatter(goal[0], goal[1], color='gold', marker='o', s=200, label='Goal', zorder=5)
ax.scatter(tree[0][2], tree[0][3], color='green', marker='o', s=100, label='Start', zorder=5)
ax.legend(loc='upper left')
plt.ion()

xs, ys = [], []
i=0
while(i<500):

    if random.random()<=0.1:
        target=goal
    else:
        target=random_point_generator()
    # find point nearest to the goal point -> distance , point
    best_distance=float('inf')
    nearest_point_p=None
    target_x=target[0]
    target_y=target[1]
    for _,_,x,y in tree:
        d=nearest_point(x,y,target_x,target_y)
        if d <best_distance:
            best_distance=d
            nearest_point_p=x,y


    # print(f"Nearest point{nearest_point_p} distance:{best_distance}")


    # obstacle=collision_free(3.5,2.5,5.5,4.5,4,6,3,7)
    # print(f"is obstacle:{obstacle}")

    step_size=1
    nearest_point_p_x=nearest_point_p[0]
    nearest_point_p_y=nearest_point_p[1]
    x,y,collision=steer(nearest_point_p_x, nearest_point_p_y, target_x, target_y, step_size)
    if collision:
        tree.append((nearest_point_p_x,nearest_point_p_y,x,y))
        xs+=[nearest_point_p_x,x,None]
        ys+=[nearest_point_p_y,y,None]
        line.set_data(xs,ys)
        if distance(x,y,goal[0],goal[1])<0.2:
            print("goal reached")
            break

    # plt.plot(
    #     [nearest_point_p[0],x,None],
    #     [nearest_point_p[1],y,None])

    ax.relim()
    ax.autoscale_view()
    # plt.pause(0.01)
    print(f" Points:{points} Target:{(target_x,target_y)} Nearest point:{(nearest_point_p)} Distance:{best_distance} Selected point:{(x,y)} Collision:{collision} Tree:{tree} len:{len(tree)}")
    i+=1

plt.ioff()


tree_last_x=tree[-1][2]
tree_last_y=tree[-1][3]
path=[(tree_last_x,tree_last_y)]

while not(tree_last_x==tree[0][2] and tree_last_y==tree[0][3]):
    for i,(parent_x,parent_y,point_x,point_y) in enumerate(tree):
        if point_x==tree_last_x and point_y==tree_last_y:
            parentx=tree[i][0]
            parenty=tree[i][1]
            path.append((parentx,parenty))
            tree_last_x=parentx
            tree_last_y=parenty
            break

path.reverse()

new_path=[]
# 
# for i in range(path):
main_x=path[0][0]
main_y=path[0][1]  
i=1
while(i):
    # main_x,main_y=path[0]
    if not (main_x==path[-1][0] and main_y==path[-1][1]):
        new_path.append((main_x,main_y))
    else:
        new_path.append((main_x,main_y))
        break
        
    for j in range(len(path)-1, -1, -1):
            check_x,check_y=path[j]
            check_collision_free=collision_free(main_x,main_y,check_x,check_y)
            if check_collision_free:
                # new_path.append((check_x,check_y))
                main_x=check_x
                main_y=check_y
                break

for i in range(len(path)-1):
    plt.plot(
        [path[i][0], path[i+1][0]],
        [path[i][1], path[i+1][1]],
        color='green', linewidth=3
    )

for i in range(len(new_path)-1):
    plt.plot(
        [new_path[i][0], new_path[i+1][0]],
        [new_path[i][1], new_path[i+1][1]],
        color='red', linewidth=3
    )


x, y = zip(*new_path)

# plt.plot(x, y)

# Highlight each point with a circle
plt.scatter(x, y, facecolors='none', edgecolors='green', s=200)

# plt.show()
print()
print("new path",new_path)



def perpendicular_points(Ax,Ay, Bx,By, Cx,Cy,distance=0.5):   #more the d curvier the curve gets

    #find vector for A to C
    dx = Cx - Ax
    dy = Cy - Ay

    #find perpendicular direction(perpendicular vector)
    perp_x = -dy
    perp_y = dx

    #calc length of perpendicular vector
    length = math.hypot(perp_x, perp_y)

    #find unit vector(normalize)
    unit_perp_x = perp_x / length
    unit_perp_y = perp_y / length

    distance=distance
    P_plus = (
        Bx + unit_perp_x * distance,
        By + unit_perp_y * distance
    )

    P_minus = (
        Bx - unit_perp_x * distance,
        By - unit_perp_y * distance
    )

    return P_plus,P_minus

def bezier(P1, B, P2, t):
    x = (1-t)**2 * P1[0] + 2*(1-t)*t * B[0] + t**2 * P2[0]
    y = (1-t)**2 * P1[1] + 2*(1-t)*t * B[1] + t**2 * P2[1]

    return x, y

def find_smooth_path(A,B,C,rounding=0.2, curve_points=30):

        smooth_path=[]
        # find control points p above below b 


        start_curve=A
        control=B
        end_curve=C

        smooth_path.append(start_curve)

        for j in range(1, curve_points+1):
             t=j/curve_points
             point=bezier(start_curve, control, end_curve,t)
             smooth_path.append(point)

        return smooth_path

def check_if_smooth_path_collide(smooth_path):
    for i in range(len(smooth_path)-1):
        is_collision_free=collision_free(smooth_path[i][0],smooth_path[i][1],smooth_path[i+1][0],smooth_path[i+1][1])
        if not is_collision_free:
            print("not collision free")
            return True
    return False


# smooth_path_final = []
def smooth_path(path, rounding=1.0, curve_points=30):

    if len(path) < 3:
        return path

    smooth_path_final = [path[0]]


    for i in range(1, len(path)-1):

        #takes three points at a time 
        A = path[i-1]
        B = path[i]
        C = path[i+1]

        # P_points=perpendicular_points(A[0],A[1], B[0],B[1], C[0],C[1])

        # if not p_plus:
        #     print("p plus is safe")
        # if not p_minus:
        #     print("p minus is safe")

        # Point before B
        P1 =(
            B[0] + rounding * (A[0] - B[0]),
            B[1] + rounding * (A[1] - B[1]))
        

        # Point after B
        P2 = (
            B[0] + rounding * (C[0] - B[0]),
            B[1] + rounding * (C[1] - B[1]))

        P_points=perpendicular_points(P1[0],P1[1], B[0],B[1], P2[0],P2[1])
        plt.scatter(P1[0],P1[1], facecolors='none', edgecolors='red', s=200)
        plt.scatter(P2[0], P2[1], facecolors='none', edgecolors='red', s=200)

        P_plus,P_minus=P_points
        ppx,ppy=P_plus
        pmx,pmy=P_minus

        p_plus=point_collision(ppx,ppy)
        p_minus=point_collision(pmx,pmy)
        # smooth_path_final.append(P1)
        
        #check for original middle point
        s_path=find_smooth_path(P1,B,P2)
        check_path_collision=check_if_smooth_path_collide(s_path)
        print("do b collide",check_path_collision)
        # smooth_path_final.extend(s_path)


        if check_path_collision:
                
                 # check perpendicular point
                p_plus_path=find_smooth_path(P1,P_plus,P2)
                check_pplus_collision=check_if_smooth_path_collide(p_plus_path)
                print("do pplus collide:",check_pplus_collision)
                if check_pplus_collision:
                    #check perpendicular point
                    p_minus_path=find_smooth_path(P1,P_minus,P2)
                    check_pminus_collision=check_if_smooth_path_collide(p_minus_path)
                    if check_pminus_collision:
                        print("p minus collide")
                    else:
                        print("p_minus is correct")
                        smooth_path_final.extend(p_minus_path)
                else:
                    print("p_plus is correct")
                    smooth_path_final.extend(p_plus_path)


        else:
            print("b is correct")
            smooth_path_final.extend(s_path)
        



        # P_points=perpendicular_points(A[0],A[1], B[0],B[1], C[0],C[1])
        # P_plus,P_minus=P_points
        # ppx,ppy=P_plus
        # pmx,pmy=P_minus
        # p_plus=point_collision(ppx,ppy)
        # p_minus=point_collision(pmx,pmy)
        # if not p_plus:
        #     print("p plus is safe")
        # if not p_minus:
        #     print("p minus is safe")


        # # 
        # # Connect previous point to P1
        # # last_point = smooth_path[-1]

        # smooth_path.append(P1)

        # # Generate Bézier curve P1 -> P2 using B as control point
        # for j in range(1, curve_points + 1):

        #     t = j / curve_points

        #     point = bezier(P1, B, P2, t)

        #     smooth_path.append(point)
        #     check_path_collision=check_if_smooth_path_collide(smooth_path)
        #     if not check_path_collision:



    # Add final point
    smooth_path_final.append(path[-1])

    return smooth_path_final

# p_points=perpendicular_points()
smooth_path_final = smooth_path(new_path, rounding=0.5, curve_points=30)

# x = [p[0] for p in new_path]
# y = [p[1] for p in new_path]

# tck, u = splprep([x, y], s=0, k=min(3, len(new_path)-1))
# u_fine = np.linspace(0, 1, 200)
# x_fine, y_fine = splev(u_fine, tck)

# smooth_path_final = list(zip(x_fine, y_fine))

# for i in range(len(smooth_path)-1):
#     plt.plot(
#         [smooth_path[i][0], smooth_path[i+1][0]],
#         [smooth_path[i][1], smooth_path[i+1][1]],
#         color='blue', linewidth=2
#     )
x = [p[0] for p in new_path]
y = [p[1] for p in new_path]
t = np.arange(len(new_path))

px = PchipInterpolator(t, x)
py = PchipInterpolator(t, y)

t_fine = np.linspace(0, len(new_path)-1, 200)
smooth_path_final = list(zip(px(t_fine), py(t_fine)))


for i in range(len(smooth_path_final)-1):
    plt.plot(
        [smooth_path_final[i][0], smooth_path_final[i+1][0]],
        [smooth_path_final[i][1], smooth_path_final[i+1][1]],
        color='blue', linewidth=2
    )


            

plt.show()


print("\n")
# print("tree",tree)
print("\n")
# print("path",path)






start  = [ (1,1), (2,2) ,(3,2), (4,3)]

for i,point in enumerate(start[:-1]):
    plt.plot(
        [point[0],start[i+1][0]],
         [point[1],start[i+1][1]]
    )
# plt.show()

point2 = (2,2)
point3 = (3,2)
point4 = (4,3)




# new_point_distance=distance(5,5,new_point[0],new_point[1])
# print(f"mew point distance:{new_point_distance}")
# print(f"New point{new_point}")
# print(random_point_generator())



d=distance(2,3,7,6)
# print(d)