import numpy as np 
import geatpy as ea 
from robot import robot
from Map import Map

Obstacle_Map=Map()
Obstacle_Map.draw_map()
robot=robot(Obstacle_Map.start,Obstacle_Map.end,Obstacle_Map,5)
Encoding="RI"
Field=ea.crtfld(Encoding,robot.vartype,robot.ranges,robot.borders)
population=ea.Population(Encoding,Field,robot.NIND)

Algorithm=ea.soea_DE_best_1_L_templet(robot,population)
Algorithm.MAXGEN=robot.MaxGen
Algorithm.f=0.5
Algorithm.pc=0.5
Algorithm.drawing=1

[population,obj_trace,var_trace]=Algorithm.run()

best_gen=np.argmax(obj_trace[:1])
best_ObjV=obj_trace[best_gen,1]
robot.points=var_trace[best_gen].reshape(2,robot.points_num).T

print("最优决策变量：")
for i in range(var_trace.shape[1]):
    print(var_trace[best_gen,i])
print("最优代价值:{}".format(best_ObjV))
robot.drawTrace()
    