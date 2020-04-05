## A General Alignment Repulsion Algorithm  for Flocking of Multi-Agent Systems 

> https://math.meta.stackexchange.com/questions/5020/mathjax-basic-tutorial-and-quick-reference

### 生词

****

>alignment 对准	repulsion 排斥	flock 群 	repel 使后退 	aforementioned 前面提到的	synchronize 同步化indispensable  必不可少的	scenario 场景	mimic 模仿	proximity 接近	interval 间隔  nonempty 非空 interval 间隔	intensity 强度  adjoint 伴随的  i. e.  也就是

### 文章的有用信息

****

***Reynolds*** 描述群体行为的方法：

1）Separation ： 群体成员避免碰撞

2）Alignment ： 与周围群体速度保持一致

3）Cohesion（凝聚） ： 与其他成员 stay close

对应三种能力： cohesion ability；alignment ability ；separating ability；

1.该文认为，聚合行为是一种效应，而不是沟通交流造成的 2. cohesion rely on long-distance communication，但是实际场景中，个体可能并不具备广阔的视野 3.实际应用的场景，偏向短距离的通讯能力

**该文主要回答一下问题 **

1）群体能够在失去cohesion ability的情况下，成群；

2）一种碰撞避免的方法

3）a novel alignment/repulsion algorithm for a stable and uncrowded flock

**Mathematical describer**
$$
\dot x_i=v_i,\dot v_i=u_i,i\in N,N:=\{ 1,2,...,n \}
$$
























