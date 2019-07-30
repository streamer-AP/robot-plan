import numpy as np 
import geatpy as ea
import cv2
class robot(ea.Problem):
    def __init__(self,start,end,Map,points_num):
        self.name="robot"
        self.start=start
        self.end=end
        self.map=Map
        self.points_num=points_num
        self.points=np.empty((points_num,2),dtype=np.int32)
        #目标维数
        self.Mdim=1
        #决策变量维数
        self.Ndim=2*points_num
        #最小化目标
        self.MinOrMax=[1]*self.Mdim
        #编码方式：
        self.encoding="BG"
        #决策变量类型：离散
        self.vartype=np.ones(self.Ndim,np.int8)
        #决策变量上边界
        self.ub=np.append(np.ones(points_num,np.int32)*(Map.height-1),np.ones(points_num,np.int32)*(Map.width-1))
        self.lb=np.zeros(2*points_num,np.int32)
        self.ubin=np.ones_like(self.ub)
        self.lbin=np.ones_like(self.lb)
        #固定起点和终点
        self.ub[0],self.ub[points_num]=start[0],start[1]
        self.ub[points_num-1],self.ub[-1]=end[0],end[1]
        self.lb[0],self.lb[points_num]=start[0],start[1]
        self.lb[points_num-1],self.lb[-1]=end[0],end[1]
        #最大迭代次数
        self.MaxGen=100
        self.gen=0
        #种群数量
        self.NIND=100*self.points_num
        ea.Problem.__init__(self, self.name, self.Mdim, self.MinOrMax, self.Ndim, 
        self.vartype, self.lb,self.ub, self.lbin, self.ubin)

    #惩罚函数，禁止路径与障碍交叉
    def punish(self,points):
        punishment=0
        for i in range(self.points_num):
            x,y=points[i][0],points[i][1]
            if self.map.map[x][y]==0:
                punishment+=1
        self.punishment=100*punishment
        return self.punishment
    
    #衡量碰撞风险
    def risk(self,points):
        riskness=0
        for i in range(self.points_num-1):
            for obstacle in self.map.obstacle:
                radius=0
                center=obstacle["center"]
                if obstacle["shape"]=="circle":
                    radius=obstacle["radius"]
                elif obstacle["shape"]=="rectangle":
                    radius=np.sqrt(obstacle["width"]**2+obstacle["height"]**2)
                distance=np.linalg.norm(np.cross(
                    points[i]-center,points[i+1]-center
                    )/np.linalg.norm(points[i]-points[i+1]))
                if distance<=radius:
                    riskness+=np.exp(-0.5*distance/radius)
                else:
                    riskness+=np.exp(-3*distance/radius)

        self.riskness=riskness        
        return self.riskness

    #计算vector1与vector2的夹角
    def angle(self,vector1,vector2):
        vector1_n2=np.linalg.norm(vector1)
        vector2_n2=np.linalg.norm(vector2)
        vector_dot=vector1.dot(vector2)
        return np.arccos(vector_dot/(vector1_n2*vector2_n2))

    #衡量路径长度
    def length(self,points):
        road_length=0
        for i in range(self.points_num-1):
            road_length+=np.linalg.norm(points[i+1]-points[i])
        self.road_length=road_length
        return self.road_length
    
    #衡量路径平滑度
    def smooth(self,points):
        road_smoothness=0
        for i in range(self.points_num-2):
            road_smoothness+=self.angle(points[i],points[i+1])
        self.road_smoothness=road_smoothness
        return self.road_smoothness
    
    #衡量路径的评价值
    def aimFunc(self,population):
       
        self.gen+=1
        self.ObjV=population.ObjV=np.empty((population.Phen.shape[0],1))
        for i in range(population.Phen.shape[0]):
            points=population.Phen[i].reshape((2,self.points_num)).T
            points=points.astype(np.int32)
            w1,w2,w3=100,1,250
            
            self.ObjV[i]=population.ObjV[i]=w1*self.smooth(points)+w2*self.length(points)+w3*self.risk(points)+self.punish(points)
            if i%200==0:
            #抽样显示
                print("smooth:{}, length:{},risk:{},punish:{}".format(self.road_smoothness,self.road_length,self.riskness,self.punishment))
        if self.gen%5==0:
            print("======================================")
            print("gen :{}/{}， best aimFunc:{}".format(self.gen,self.MaxGen,np.min(population.ObjV)))
            print("======================================")

    
    def drawTrace(self):
        image=self.map.image
        self.points=self.points.astype(np.int32)
        for i in range(len(self.points)-1):
            cv2.line(image,(self.points[i][0],self.points[i][1]),(self.points[i+1][0],self.points[i+1][1]),(255,255,0))
        for point in self.points[1:-1]:
            cv2.circle(image,(point[0],point[1]),5,(255,255,0),thickness=2)
        self.image=image
        cv2.imshow("Trace",image)
        cv2.waitKey(0)
        cv2.imwrite("Trace.jpg",self.image)



