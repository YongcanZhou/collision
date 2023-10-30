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
  

https://github.com/YongcanZhou/collision/assets/59829915/3853b102-e7c4-46a8-86be-9f868f660f7f

- ball_rebound when restitution coefficient = 1
  

https://github.com/YongcanZhou/collision/assets/59829915/2cd5c0ba-a2c9-43d2-bbc9-2a4f0ff03d58

- without collision detection
  

https://github.com/YongcanZhou/collision/assets/59829915/ac6f27d9-fe6b-4c1c-a3c9-7281de7b3a90

- collision detection
  

https://github.com/YongcanZhou/collision/assets/59829915/884dd0d3-a216-404d-bec4-692d6105e011

- CAM
  

https://github.com/YongcanZhou/collision/assets/59829915/ed496b67-17ce-4210-9296-c7ed22921d0e

- CAD :[OCCTSketcher](https://github.com/YongcanZhou/OCCTSketcher)
  

https://github.com/YongcanZhou/collision/assets/59829915/c693e9f2-e650-48a0-8978-baefe2290ccd

- Polish
  

https://github.com/YongcanZhou/collision/assets/59829915/a1bca1cb-bbd0-44e6-ab77-82cfc6eb7f2e







