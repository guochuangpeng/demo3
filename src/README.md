# docking_and_landing  

#### Description
{**When you're done, you can delete the content in this README and update the file with details for others getting started with your repository**}

#### Software Architecture
Software architecture description

#### Installation

1.  put these packages in src of your workspace
2.  catkin build (delete build and devel folder if you want rebuild your workspace after change code)
3.  xxxx

#### Instructions

1.  use "./docking3D.sh" to execute the docking task with esti and output both work in 10HZ
2.  xxxx
3.  xxxx

#### Contribution

1.  Fork the repository
2.  Create Feat_xxx branch
3.  Commit your code
4.  Create Pull Request


#### Gitee Feature

1.  You can use Readme\_XXX.md to support different languages, such as Readme\_en.md, Readme\_zh.md
2.  Gitee blog [blog.gitee.com](https://blog.gitee.com)
3.  Explore open source project [https://gitee.com/explore](https://gitee.com/explore)
4.  The most valuable open source project [GVP](https://gitee.com/gvp)
5.  The manual of Gitee [https://gitee.com/help](https://gitee.com/help)
6.  The most popular members  [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)

#### Tips
2021.01.16 use RelativeLocalization10HZ.cpp to realize the docking can work, 
           but have a little problem ：  it can not out 0 velocity when distance less than safe distance ,
           because forget publish velocity to mavros after assignment 0 velocity. 
           (corrected, but power is low so that have no time to verify)
           
2021.03.01 use RelativeLocalization2D.cpp uav_movement2D.cpp  to realize the docking can work, 
           attention:
             static target ,2D can work, but!!! move toward south-western after hover and stop after 1 meter
2021.03.02 use RelativeLocalization2D.cpp RelativeLocalization3D.cpp uav_movement2D.cpp uav_movement3D.cpp to realize the docking can work, 
           attention :
             firt test: static target ,2D can work
             second test: move target ,2D can work ,but!!! move toward south-western after hover and no stop
             third test: static target ,3D can work, faster than 2D  tips: the height is too low,nearly touch the land 
