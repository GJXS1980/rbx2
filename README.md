#   启动
```bash
#   启动turtlebot模型
roslaunch rbx2_tasks fake_turtlebot.launch 

#   打开rviz
rosrun rviz rviz -d `rospack find rbx2_tasks`/nav_tasks.rviz

#   启动smach可视化界面
rosrun smach_viewer smach_viewer.py
```

#   启动例程
```bash
rosrun rbx2_tasks patrol_smach_concurrence.py 
```
