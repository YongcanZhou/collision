# Robot_OCC

## 依赖库

1. [Qt 5.12.12](https://download.qt.io/official_releases/qt/) 

2. [Open CASCADE Technology 7.4.0](https://dev.opencascade.org/release/previous#node-29988)   

   - 安装完需要重新对Open CASCADE Technology编译，cmake时候不选择using VTk

3. [PCL 1.12.1](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.12.1) 

   - VTK 9.1 需要重新源码编译 + cmake时候勾选QT相关选项  

     [windows debug boost](https://github.com/PointCloudLibrary/pcl/issues/5205)

     [VTK 9.1 添加QT相关lib](https://mangoroom.cn/cpp/pcl-vtk9-x-viewer-hosted-on-qt-widget.html)

4. [hpp-fcl](https://github.com/leitianjian/hpp-fcl/tree/145b08e1bf98daba5d8ab4ace248c7e99a1e5faf) 

5. [aris](https://github.com/py0330/aris)

6. [aris_sim](https://github.com/leitianjian/ArisSim) 

## 平台

Visual studio：install VS QT tools

## 使用

导入机器人实现按预定轨迹运动，同时检测碰撞





