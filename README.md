# Robot contact force calculate 

## Dependent library

1. [Qt 5.12.12](https://download.qt.io/official_releases/qt/) 

2. [Open CASCADE Technology 7.4.0](https://dev.opencascade.org/release/previous#node-29988)   

   - cmake with not using VTk

3. [PCL 1.12.1](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.12.1) 

   - VTK 9.1  recmake with QT options

     [windows debug boost](https://github.com/PointCloudLibrary/pcl/issues/5205)

     [VTK 9.1 add QT lib](https://mangoroom.cn/cpp/pcl-vtk9-x-viewer-hosted-on-qt-widget.html)

4. [hpp-fcl](https://github.com/leitianjian/hpp-fcl/tree/145b08e1bf98daba5d8ab4ace248c7e99a1e5faf) 

5. [aris](https://github.com/py0330/aris)

6. [sire](https://github.com/leitianjian/sire) 

## Platform

Visual studio：install VS QT tools

## Usage

- Import the robot to realize the movement according to the predetermined trajectory
- detect the collision at the same time 
- calculate the contact force

## demo

- ball_rebound when restitution coefficient = 0.8

  ![image](https://github.com/YongcanZhou/collision/blob/main/model/demo/ball_rebound_0.8.gif)

  https://github.com/YongcanZhou/collision/blob/main/model/demo/ball_rebound_0.8.mp4

- ball_rebound when restitution coefficient = 1

  ![image](https://github.com/YongcanZhou/collision/blob/main/model/demo/ball_rebound_1.gif)

  https://github.com/YongcanZhou/collision/blob/main/model/demo/ball_rebound_1.mp4

- without collision detection

  ![image](https://github.com/YongcanZhou/collision/blob/main/model/demo/without_collision.gif)

  https://github.com/YongcanZhou/collision/blob/main/model/demo/no_collision_detection.mpeg4.aac.mp4

- collision detection

  ![image](https://github.com/YongcanZhou/collision/blob/main/model/demo/collision.gif)

  https://github.com/YongcanZhou/collision/blob/main/model/demo/collision_detection.mpeg4.aac.mp4

- CAM

  https://github.com/YongcanZhou/collision/blob/main/model/demo/CAM.mpeg4.aac.mp4

- CAD

  https://github.com/YongcanZhou/collision/blob/main/model/demo/CAD.mp4

  https://github.com/YongcanZhou/collision/blob/a5e365ad18dc2c6cc435ed9ab8ac56a7fabb5f65/model/demo/CAD.mp4

  https://github.com/YongcanZhou/collision/blob/a5e365ad18dc2c6cc435ed9ab8ac56a7fabb5f65/model/demo/CAD

- Polish

  https://github.com/YongcanZhou/collision/blob/main/model/demo/polish.mpeg4.aac.mp4









