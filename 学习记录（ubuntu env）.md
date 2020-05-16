## 学习记录（ubuntu env）

### 2020.3.27

#### 上午：

尝试在ubuntu上运行，matlab程序，但由于编码问题，安装了enca 一个字符集工具

解决方法如下

> https://blog.csdn.net/haoji007/article/details/79983816

```shell
cd m_file_path
enca -L zh_CN -x UTF-8 learning.m 
enca -L zh_CN -x UTF-8 *.m
```

还需要安装机器人工具箱

今天的学习计划是什么？？

matlab查看已安装的工具库

`ver`

#### 下午

现在的一些启发：

1.signal form workspace，按列排列数值，并在左方增加采样时间，和数值一一对应

1．添加To Workspace模块；

2．添加out模块；

3．直接用Scope输出。

> https://blog.csdn.net/gg0000gg/article/details/79401656

### 2020.3.28

首先在ubuntu上运行matlab程序，然后调试舵机pid参数，记录实验数据，所以要找到记录实验数据的手段。最好能够导入到matlab处理，在matlab上绘制图像

目前需要做的是控制一个舵机跟随着轨迹运动

#### 上午

目前工作情况：（1）launch启动，能够在yaml文件中配置舵机 （2）能够通过service改变舵机的目标位置，以及调试pid参数，目前调试的结果 p=0.05 d=0.05 i=0，并且可以绘制位置、速度、电流大小的变化曲线 （3）能够实现控制多个电机联动，并通过service控制每个舵机的目标位置 （4）实现轨迹话题，按照写频率生成最小加速度的轨迹 （5）实现控制器operator，启动控制器加载正确的路径文件可以实现轨迹的控制（6）通过rosbag命令，记录舵机的轨迹变化，并转换成txt文件，并在matlab中绘制 （7）通过matlab生成轨迹的yaml文件，应该可以直接给控制器进行控制，但其中轨迹规划的实际参数等存在问题 （8）能够绘制期望轨迹和实际轨迹的曲线

下一步解决轨迹规划的问题，

轨迹规划的话就涉及到速度的原因，或者直接跳过插值的部分

还有思考分布式控制的可行性，

ROS的时间问题

尝试一下自带的PID控制





1.记录一下调试pid的过程数据；

下一步工作：产生运动轨迹，输入到单个舵机上；

计划：应该是一个客户端，pub发送运动轨迹，可以参考官方提供的controller代码

trajectory_msgs::JointTrajectory 格式数据，这个格式是ros自带的

> http://wiki.ros.org/joint_trajectory_controller/

```yaml
#trajectory_msgs/JointTrajectory.msg
Header header
string[] joint_names
JointTrajectoryPoint[] points

#trajectory_msgs/JointTrajectoryPoint.msg
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start


#desired_trajectory.msg
uint8[] id
int32[] goal_position

```

yaml文件保存路径，operator pub路径 然后controller接收，接收后有个callback函数

在callback函数中 WayPoint wp；判断运动状态的flag is_moving；如果没在动的话，获取关节当前位置

```C++
typedef struct _Point
{
  double position;
  double velocity;
  double acceleration;
} WayPoint;
std::vector<WayPoint> goal;//store trajectory points
jnt_tra_->init(move_time, write_period_,pre_goal_,goal);
```

workbench中的 trajectory_generator 什么时候用到？？

关键点：3个定时器 read_timer,write_timer,publish_timer

write_timer:

is_moving 这个flag决定要不要写入数据；

同时读取多个舵机的状态

```C++
typedef struct 
{
  const char *item_name;
  uint16_t    address;
  uint8_t	    item_name_length;
  uint16_t    data_length;
} ControlItem;
```

```yaml
# DynamixelState List
DynamixelState[] dynamixel_state
# DynamixelState
# This message includes basic data of dynamixel
string name
uint8  id
int32  present_position
int32  present_velocity
int16  present_current

```

有点超调

```shell
rosservice call /changePidGain "p_gain: 0.06
i_gain: 0.001
d_gain: 0.1"

rosbag record /my_dxl_master/dynamixel_state
rosbag record /my_dxl_master/dynamixel_state /my_dxl_master/desired_tra
rosbag record /my_distribute_controller/desired_tra /my_distribute_controller/dynamixel_state

rosbag play filename.bag -l #循环播放
rostopic echo -b 2020-03-31-20-05-26.bag -p my_dxl_master/dynamixel_state > state.txt

rostopic echo -b 2020-04-06-09-31-01.bag -p my_dxl_master/desired_tra > d_tra.txt

rostopic echo -b 2020-04-09-16-55-07.bag -p /my_distribute_controller/dynamixel_state > dis_state01.txt


```

##### 3.我该怎么做

感觉执行轨迹的时间有些问题，明明设置的是10s时间，执行了63s，经验表明大概有6倍的关系

1.记录pid调试过程的数据，情况

要求是不超调，没有冲击情况

写一个matlab程序：输出轨迹yaml文件；解析格式，并绘图；

如何得到实际的轨迹图？

学一下python绘图方法；

目前记录数据的思路如下：

还要获取一下期望的轨迹；获取期望轨迹的话就是新建一个话题

发现代码还是有一些问题的

把三个舵机的轨迹在三维图中显示，忽略掉时间维度，只显示位置的误差（这个思路还是有点意思的）

ROS的时间间隔还需要弄明白



##### 4.其他的一些启示：

1.获取目标位置时，封装成函数，输入：舵机的别名，参照workbench getPresentPosition （感觉不好）

2.yaml文件就像一个树，由缩进决定层次，可以学习如何对其进行读取

3.后续需要整理的内容

​	3.1 yaml库、eigen库如何加入到包里面

​	3.2各个函数的作用，另外如果要自己开发的话应该按照什么样的步骤

​	3.3guide文件

4.在程序中位数表示很重要，不然会计算错误

##### 5.这个表达式什么意思？？

```c++
for (auto const& joint:msg->joint_names)
```

> https://blog.csdn.net/dajiadexiaocao/article/details/81776364

: 可能是赋值；这是c++11中for循环的新用法，相当于遍历joint_names,每次赋值给joint

```C++
jnt_tra_msg_->joint_names.push_back(joint);
```

> https://www.cnblogs.com/xxyue/p/9051509.html

pushback()对于数组、向量而言是将元素加到vector的最后面，即当前最后一个元素的下一个元素，新的值为val的拷贝

```C++
vector::void push_back (const value_type& val);
vector::void push_back (value_type&& val);
```

#### 6.存疑

ros launch文件的参数：arg param rosparam

### 2020.3.31-2020.4.6

记录一些实验数据，并进行绘图，考虑用python绘图

1.pid调试过程的图，目标位置以及实际位置随时间的变化图；还有此时的电流、速度的情况

2.舵机跟踪轨迹的图

3.如何获取期望的轨迹？将时间轴调为一致

#### 2020.4.1

**下午和晚上**

学习一下python的基本语法，以及matplotlib基本用法

重新调试一下舵机的pid参数并记录，需要在win中获得准确的模型数据？写一下毕业论文

学习一下git的用法，用来管理自己的仓库