# Introduction

# 当前任务
- [ ] 创建一个类，这个类要有一个`subscriber`来收取`/tf`这个topic上面的信息，然后存入私有变量，之后它有一个成员函数定时从私有变量中解算杆的长度，那么，在这个类中就先需要完成解算杆长度的任务。

# 思路

1. 首先，我先把`platform_up.STL`和`platform_down.STL`都加入到小车的模型中，然后在二者的球铰的中心处都加上一个很小的方块，或者说就是加上一个`frame`，用来表示这个点的坐标+旋转
2. 其中`up_link`是`littleY_link`的`child`，通过`fixed joint`连接，也就是说`up_link`跟`litleZ_link`也是固连的,这样`wx_link`跟`up_link`之间存在着绕着`up_link`Z轴的旋转，给逆解增加了难度
3. 而`down_link`是`car_link`的`child`，通过`fixed joint`连接。
4. 这样，在给定`wx_link`的位姿时，`up_link`会增加一个绕`litleZ_link`的旋转，这个转轴的旋转角度在优化的时候也需要有所考虑

# TIPS
1. In RVIZ, `rviz/DisplayTypes/Axes` has three axises
   1. red - x 
   2. green - y 
   3. blue -z