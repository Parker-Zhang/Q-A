## Python for science

### 1. python version

> https://www.cnblogs.com/ruiyang-/p/10162581.html

Ubuntu 18.04 python 版本切换方法

查看当前python 版本，自己当前的默认版本是python2.7，有python3.6版本

```shell
python -V	#查看默认python的版本  or python --version
python -v	#进入一个类似编辑器的东西，exit()退出
ls /usr/bin/python*	#查看哪些python二进制文件可用

#使用python xx.py运行程序时，加上版本号

#基于用户修改python的版本
sudo gedit .zshrc
#add cmd like 
alias python='/usr/bin/python3.6'
#source
source .zshrc
```

### 2. install

数据分析使用到的安装包：

` Numpy，Scipy，Matplotlib，IPython` 

官网地址：

> NumPy官网：https://numpy.org/
>
> matplotlib官网：https://matplotlib.org/
>
> SciPy官网：https://scipy.org/
>
> guide:	https://scipy-lectures.org/
>
> https://numpy.org/devdocs/user/quickstart.html
>
> python教程：https://docs.python.org/3/tutorial/

installation:

**warning : only for older version,like python 2.7**

```shell
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
```



### 3. learning

#### 3.1 生词

> generic 通用的	Modules 模组	interpolation 插补	syntax 句法	tuples 元组



#### 3.2 Numerical types,Containers,Assignment operator

数据结构以及基本操作：

> https://scipy-lectures.org/intro/language/basic_types.html#numerical-types

```python
# string formating
'An integer: %i;a float:%f;another string:%s' %(1,0.1,'string')
# list : []  dictionaries {}, have values & keys
```













