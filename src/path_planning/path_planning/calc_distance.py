import math
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches


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

#points
points=[(1,1), (3,2) ,(5,5), (8,3)]
#target


fig, ax = plt.subplots(figsize=(12,12))
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
    plt.pause(0.1)
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

for i in range(len(path)-1):
    plt.plot(
        [path[i][0], path[i+1][0]],
        [path[i][1], path[i+1][1]],
        color='green', linewidth=3
    )
plt.show()

print("\n")
print("tree",tree)
print("\n")
print("path",path)






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