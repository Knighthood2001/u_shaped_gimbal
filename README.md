# u_shaped_gimbal
## 说明
<p align="center">
  <img src="img/origin1.png" width="50%">
  <img src="img/origin2.png" width="40%">
</p>

这是我从原始云台中，去除了相机、玻璃片后的模型。

<img src="img/change.png" width="50%" />

然后将其通过 `sw2urdf` 转换成 URDF 文件，方便大家后续能够使用。

接下来你就可以在  
[**https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/**](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/)  
中拖入这一整个文件夹进行查看，并可以拉动左下角的拉杆进行观察。

![alt text](img/show_urdf.png)

## ROS 运行

- 在rviz中可视化
```shell
roslaunch u_shaped_gimbal display.launch
```
![alt text](img/show_rviz.png)

- 在gazebo中仿真

```shell
roslaunch u_shaped_gimbal gazebo.launch
```

![alt text](img/show_gazebo.png)

目前由于没有进行坐标系的转换，所以模型是倒地的。

通过设置
```xml
  <!-- 虚拟根节点 -->
  <link name="root_link"/>
  <!-- root_link 到 base_link 的固定关节 -->
  <joint name="root_to_base" type="fixed">
    <parent link="root_link"/>
    <child link="base_link"/>
    <!-- 绕 X 轴 +90°，再绕 Z 轴 +90°，帮助云台立起来 -->
    <origin xyz="0 0 0" rpy="1.5708 0 1.5708"/>
  </joint>
```
可以使得云台立起来。

![alt text](img/gazebo_stand.png)
## 最后
如果需要原始模型，可以通过网盘下载

链接：[**https://pan.quark.cn/s/35a948faf0dc**](https://pan.quark.cn/s/35a948faf0dc)

里面包含原始模型，绑定点、轴、坐标系，使用sw2urdf的设置模型过程的模型，方便大家自己动手。