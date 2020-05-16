ROS Learning

## ROS简介

 ROS系统是起源于2007年斯坦福大学人工智能实验室的项目与机器人技术公司Willow Garage的个人机器人项目（Personal Robots Program）之间的合作，2008年之后就由Willow Garage来进行推动。

 ROS的首要设计目标是在机器人研发领域提高代码复用率。ROS是一种分布式处理框架（又名Nodes）。

**（1）点对点设计**

 **（2）多语言支持**

（3）精简与集成

（4）工具包丰富

（5）免费且开源

### ROS框架

一种是核心（main），一种是库（universe）

节点、消息、主题、服务

包、堆



rviz、tf、gazebo

rviz具备很强的扩展性，可以用来制作成机器人控制的上位机



移动机器人的案例：

turtlebot：

ROS探索总结：四到八;三十；

https://www.guyuehome.com/237

https://www.guyuehome.com/243

https://www.guyuehome.com/248

胡春旭研究生时的作品：

https://www.guyuehome.com/1856



机械臂：

https://www.guyuehome.com/455



>  最后，来发表一些对ROS的看法。ROS虽然好用，但是目前的ROS1还只适合于研发，如果没有强大的研发能力，建议还是不要轻易应用于产品。我们在开发过程中，遇到过以下一些坑：
>
> 1. roscore会突然挂掉，node会突然挂掉，rviz会突然挂掉，ROS的一切都会挂，系统就go die。。。 
>
> 2. moveit没办法实现连续运动，也没有点动等基本操作，需要去看moveit的底层代码然后重新组合去实现。 
>
> 3. ROS没有实时功能，需要自己搭建实时核，开发实时任务 
>
> 4. ROS资源占用率较大，对计算机的性能要求较高 



ROS的一些缺陷：

缺乏构建多机器人系统的标准方法；

在Windows、MacOS、RTOS等系统上无法应用或者功能有限；

缺少实时性方面的设计；

需要良好的网络环境保证数据的完整性，而且网络没有数据加密、安全防护等功能；

ROS 1的稳定性欠佳，研究开发与上市产品之间的过渡艰难；



## urdf 模型导入



## moveit setup 配置助手

21讲 海龟





## moveit编程



### 关节空间的规划：

#### 正运动学控制的方法

流程：先确定group 再确定约束 再确定目标位置 再规划，最后执行

初始化 move_group 的API

初始化ROS节点

初始化需要使用的 **move group** 控制的机械臂中的arm group

​	`arm = moveit_commander.MoveGroupCommander('manipulator')`

设置机械臂运动的允许误差值

设置允许的最大速度和加速度（设置一个系数，即最大速度的百分比 0-1的值）

设置机械臂的**目标位置**（有两种方法：一、named target  二、value target），**正运动学**

​	named target ： 别名的方法

​	value target ： 六轴的位置数据进行描述（单位：弧度）

> 问题：很难确定六关节的角度

控制机械臂运动 ： `arm.go()`: 相当于 plan and execute 先规划再执行

> 如果遇到无法求解的情况怎么办？

关闭并退出moveit

```shell
 # 目标catkin whitelist 白名单 packages 包
 catkin_make -DCATKIN_WHITELIST_PACKAGES="c800_description"
```

#### 逆运动学控制的方法	

通过四元数描述，基于base_link 坐标系，通过**逆向运动学**

流程：

初始化 move_group 的API

初始化ROS节点

初始化需要使用的 **move group** 控制的机械臂中的arm group

​	`arm = moveit_commander.MoveGroupCommander('manipulator')`

获取**终端**link名称

设置目标位置所使用的**参考坐标系**

 当运动规划失败后允许重新规划（KDL 数值求解方法）

设置位置、姿态的允许误差

设置允许的最大速度和加速度

**描述目标位姿**

​	位置：x y z

​	姿态：orientation（x，y，z，w）



### 笛卡尔空间的规划

会有一个路径列表：`waypoints`



安排：

b站

ROS入门21讲

solidworks urdf

C++

第一步：

SimMechanics Link 导入CAD模型和数据

看一下有没有现有的

`Quardrotor_Controller_xPC_158_jwu`



ROS下一步的学习计划：

相机二维码跟踪：古月居首页

ros control：https://www.guyuehome.com/890

action指令学习：https://www.guyuehome.com/908

rviz plugin：用qt做一个插件来发布指令，做一个可视化的界面控制

https://www.guyuehome.com/945

matlab_ros、matlab可视化控制ros：

https://www.guyuehome.com/1047

有限状态机smach：三十八、三十九

感觉挺复杂的，一种状态变化可视化的东西

https://www.guyuehome.com/1069

**看到一个比较完整的机械臂教程**

https://www.guyuehome.com/1159

动态参数配置：

twist_mux 多路切换器：应对一种话题的冲突机制；可以帮助我们切换到希望接收的数据源上。

https://www.guyuehome.com/1534

Kongfu机械臂介绍：一个不错的ppt展示

https://www.guyuehome.com/1608

百度 apollo ros 是对ros通讯机制的改良

https://www.guyuehome.com/1641：这篇介绍ros通讯问题还是比较有帮助

moveit 运动学插件：运动学求解器介绍，教你如何换一个高效的求解器

https://www.guyuehome.com/1921

ROS和verp软件集合

https://www.guyuehome.com/1966

launch文件学习

https://www.guyuehome.com/2195

moveit一些高级的用法：

https://www.guyuehome.com/2933

ROS视觉教程

https://www.guyuehome.com/2988



**我觉得学习一下ROS2还是有必要的**

但是ROS2带来了改变，支持构建的系统包括Linux、windows、Mac、RTOS，甚至没有操作系统的裸机。



