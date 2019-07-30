# 路径规划算法

用于带障碍的路径规划仿真及可视化，引入了geatpy 进化算法作为示例，可引入其它方法。

## 依赖
使用conda 进行版本管理
```sh
conda create -n robot python=3.6
conda activate robot
pip install geatpy
pip install numpy
pip install opencv_python
```
## 使用方法

修改map_config.py 中的障碍地图，width, height 控制地图的总大小，obstacle 为可添加的障碍物,start,end为需要进行规划的起点和终点。
robot.py 中为问题求解器，其中主要定义对规划路径的评价函数，包括碰撞风险，路径长度，路径平滑度，w1,w2,w3为其权重
main.py 中可选择进化算法模板，种群数量，

``` python
python main.py
```

生成的路径示例：

![路径](https://github.com/streamer-AP/robot-plan/blob/master/Trace.jpg)
