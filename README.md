# games101
Games101的作业
作业地址 http://games-cn.org/forums/topic/allhw/

作业需要 Eigen3、opencv，这里使用的是Eigen3.3.9 及Opencv 4.5.2版本

Opencv官网 https://opencv.org/
Eigen官网 https://eigen.tuxfamily.org/

# 环境配置
## 安装软件
安装VS2019
## 配置环境
创建C++空工程，工程右键属性选择VC++目录，在包含目录一项，编辑头文件的位置。库目录一项，编辑静态链接库的位置。
选择链接器->输入->附加依赖项，把opencv的lib库名字输入进去(\*d.lib此类文件是debug使用的，不含d的为release使用的)
opencv的release和debug不可以同时加进去，调试时会报错！（坑！）

配置至此，已经可以生成工程了，但是还无法调试，如果调试会报某某dll找不到。

为了可以调试 在属性中找到调试，在环境一栏中编辑path=..\lib...\bin(这是一个例子)
这个路径是 动态链接库的位置所在，至此就可以进行调试了。
