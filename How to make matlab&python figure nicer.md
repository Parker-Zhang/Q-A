## How to make matlab&python figure nicer?

### Ex1

> https://www.zhihu.com/search?type=content&q=matlab%E5%9B%BE%E6%9B%B4%E5%A5%BD%E7%9C%8B

```matlab
clc, clear, close all 
x = linspace(0, 3*pi, 40);
y1 = sin(x); y2 = 0.1*x-0.5; 
y3 = cos(x+pi/2); 
figure % 黑色实线, 圆圈标记, 线宽1.1, 标记内部填充颜色从网上找好看的配色 
plot(x, y1, 'ok-', 'linewidth', 1.1, 'markerfacecolor', [36, 169, 225]/255) 
hold on 
plot(x, y2, 'ok-', 'linewidth', 1.1, 'markerfacecolor', [29, 191, 151]/255) 
% 设置坐标轴范围 
axis([0, 3*pi, -1.2 1.2]) 
% 坐标轴边框线宽1.1, 坐标轴字体与大小为Times New Roman和16 
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time (s)') 
ylabel('Displacement (m)') 
legend('sin(x)', '0.1x-0.5')
```

![v2-75c99ba58a8fa3dbe7cb96fd5837aecf_b](image\v2-75c99ba58a8fa3dbe7cb96fd5837aecf_b.jpg)

**python**

```python
from matplotlib import pyplot as plt import numpy as np x = np.linspace(0, 3*np.pi, 40) y1 = np.sin(x) y2 = 0.1*x-0.5 plt.figure(figsize=(12.5, 10)) 
# 线条颜色black, 线宽2, 标记大小13, 标记填充颜色从网上找16进制好看的颜色 
plt.plot(x, y1, '-o', color='black', markersize=13, markerfacecolor='#44cef6', linewidth=2) 
plt.plot(x, y2, '-o', color='black', markersize=13, markerfacecolor='#e29c45', linewidth=2) 
# 字体设置: 字体名称Times New Roman, 字体大小34 
font_format = {'family':'Times New Roman', 'size':34} 
plt.xlabel('Time (s)', font_format) 
plt.ylabel('Displacement (m)', font_format) 
# 设置坐标轴 x范围0~3*pi, y范围-1.2~1.2 
plt.axis([0, 3*np.pi, -1.2, 1.2]) 
# 横纵坐标上的字体大小与类型(不是xlabel, 是xticks) 
plt.xticks(fontproperties='Times New Roman', size=34) plt.yticks(fontproperties='Times New Roman', size=34) 
# 整个图像与展示框的相对位置 
plt.subplots_adjust(left=0.19,right=0.94, bottom=0.13) 
# 调整上下左右四个边框的线宽为2 
ax=plt.gca() 
ax.spines['bottom'].set_linewidth(2) 
ax.spines['left'].set_linewidth(2) 
ax.spines['right'].set_linewidth(2) 
ax.spines['top'].set_linewidth(2) 
plt.show()
```

```matlab
xlabel('\fontname{宋体}基准值\fontname{Times New Roman}(mm/\fontname{宋体}日\fontname{Times New Roman})');

%改变坐标系在画布的位置
set (gca,'position',[0.1,0.12,0.7,0.8] );
```

