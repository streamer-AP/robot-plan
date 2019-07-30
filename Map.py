import cv2
import json
from map_config import map_config
import numpy as np
class Map(map_config):
    def __init__(self):
        print(self.obstacle)

    def draw_map(self):
        #绘制地图，地图障碍物（黑色实心），起点（蓝色空心点），终点（绿色实心点）
        self.image=np.ones((self.height,self.width,3),dtype=np.uint8)*255
        for shape in self.obstacle:
            if shape["shape"]=="rectangle":
                pt1=(shape["center"][0]-shape["width"]//2,shape["center"][0]-shape["height"]//2)
                pt2=(shape["center"][0]+shape["width"]//2,shape["center"][0]+shape["height"]//2)
                cv2.rectangle(self.image,pt1,pt2,0,thickness=-1)
            elif shape["shape"]=="circle":
                cv2.circle(self.image,shape["center"],shape["radius"],0,thickness=-1)
        
        self.map=cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        _,self.map=cv2.threshold(self.map,125,255,cv2.THRESH_BINARY)
        cv2.circle(self.image,self.start,5,(255,0,0),thickness=2)
        cv2.circle(self.image,self.end,5,(0,255,0),thickness=2) 

if __name__ == "__main__":
    Map_=Map()
    Map_.draw_map()



    