#!/usr/bin/env python
#! -*- coding: utf-8 -*-

""" clean_house_smach.py - Version 1.0 2013-04-20

    Control a robot to move from "room to room" and "clean" each room appropriately.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import smach
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Twist
from rbx2_tasks.history_task_setup import *
import easygui
import datetime
from collections import OrderedDict

#   讲解任务的列表
task_list = {'written_words':['explanatory_text'], 'rule':['explain_the_rule'], 'life':['explain_life'], 'faith':['explain_faith']}

class WrittenWords(State):
    def __init__(self, section, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.task = 'explanatory_text'
        self.section = section
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def execute(self, userdata):
        rospy.loginfo('Explaining the text in the ' + str(self.section))
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.05
        counter = self.timer
        while counter > 0:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self.cmd_vel_pub.publish(cmd_vel_msg)
            cmd_vel_msg.linear.x *= -1
            rospy.loginfo(counter)
            counter -= 1
            rospy.sleep(1)
        
        self.cmd_vel_pub.publish(Twist())
        message = "Finished explanatoring text the " + str(self.section) + "!"
        rospy.loginfo(message)
        easygui.msgbox(message, title="Succeeded")
        
        update_task_list(self.section, self.task)

        return 'succeeded'

class Rule(State):
    def __init__(self, section, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.task = 'explain_the_rule'
        self.section = section
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
   
    def execute(self, userdata):
        rospy.loginfo('Explaining the rule in the ' + str(self.section))
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.05
        cmd_vel_msg.angular.z = 1.2
        counter = self.timer
        while counter > 0:
            self.cmd_vel_pub.publish(cmd_vel_msg)
            cmd_vel_msg.linear.x *= -1
            rospy.loginfo(counter)
            counter -= 1
            rospy.sleep(1)
            
        self.cmd_vel_pub.publish(Twist())
        message = "Done explaining the rule the " + str(self.section) + "!"
        rospy.loginfo(message)
        easygui.msgbox(message, title="Succeeded")
        
        update_task_list(self.section, self.task)
             
        return 'succeeded'

class Life(State):
    def __init__(self, section, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.task = 'explain_life'
        self.section = section
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def execute(self, userdata):
        rospy.loginfo('Explain the life...')
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.4
        cmd_vel_msg.angular.z = 0.2
        counter = self.timer
        while counter > 0:
            self.cmd_vel_pub.publish(cmd_vel_msg)
            cmd_vel_msg.linear.x *= -1
            if counter % 2 == 5:
                cmd_vel_msg.angular.z *= -1
            rospy.loginfo(counter)
            counter -= 1
            rospy.sleep(0.2)
    
        self.cmd_vel_pub.publish(Twist())
        message = "Done explaining the life!"
        rospy.loginfo(message)
        easygui.msgbox(message, title="Succeeded")
        
        update_task_list(self.section, self.task)
            
        return 'succeeded'

class Faith(State):
    def __init__(self, section, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.task = 'explain_faith'
        self.section = section
        self.timer = timer
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def execute(self, userdata):
        rospy.loginfo('Explain the faith...')
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.4
        cmd_vel_msg.angular.z = 0.2
        counter = self.timer
        while counter > 0:
            self.cmd_vel_pub.publish(cmd_vel_msg)
            cmd_vel_msg.linear.x *= -1
            if counter % 2 == 5:
                cmd_vel_msg.angular.z *= -1
            rospy.loginfo(counter)
            counter -= 1
            rospy.sleep(0.2)
    
        self.cmd_vel_pub.publish(Twist())
        message = "Done explaining the faith!"
        rospy.loginfo(message)
        easygui.msgbox(message, title="Succeeded")
        
        update_task_list(self.section, self.task)
            
        return 'succeeded'

def update_task_list(section, task):
    task_list[section].remove(task)
    if len(task_list[section]) == 0:
        del task_list[section]

class main():
    def __init__(self):
        rospy.init_node('explain_history', anonymous=False)
        
        # 设置机器人关闭函数(stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # 初始化一些参数和变量
        setup_task_environment(self)
        
        # 将room locations转换成SMACH move_base action states
        nav_states = {}
        
        for room in self.room_locations.iterkeys():         
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = self.room_locations[room]
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb, 
                                                exec_timeout=rospy.Duration(15.0),
                                                server_wait_timeout=rospy.Duration(10.0))
            nav_states[room] = move_base_state

        ''' Create individual state machines for assigning tasks to each room '''

        # 为written words子任务创建一个状态机
        sm_written_words = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # 然后添加子任务
        with sm_written_words:
            StateMachine.add('EXPLAIN_HISTORY', WrittenWords('written_words', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # 为rule子任务创建一个状态机
        sm_rule = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # 然后添加子任务
        with sm_rule:
            StateMachine.add('EXPLAIN_HISTORY', Rule('rule', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # 为life子任务创建一个状态机
        sm_life = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # 然后添加子任务
        with sm_life:
            StateMachine.add('EXPLAIN_HISTORY', Life('life', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # 为faith子任务创建一个状态机
        sm_faith = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        # 然后添加子任务
        with sm_faith:
            StateMachine.add('EXPLAIN_HISTORY', Faith('faith', 5), transitions={'succeeded':'','aborted':'','preempted':''})

        # 初始化整个状态机
        sm_history_smach = StateMachine(outcomes=['succeeded','aborted','preempted'])
            
        # 从nav状态机和room cleaning状态机构建clean house状态机
        with sm_history_smach:            
            StateMachine.add('START', nav_states['explanatory_text'], transitions={'succeeded':'WRITTEN_WORDS','aborted':'WRITTEN_WORDS','preempted':'WRITTEN_WORDS'})
            
            ''' Add the living room subtask(s) '''
            StateMachine.add('WRITTEN_WORDS', nav_states['explanatory_text'], transitions={'succeeded':'WRITTEN_WORDS_TASKS','aborted':'RULE','preempted':'RULE'})
            
            # 当任务完成时, 继续进行kitchen任务
            StateMachine.add('WRITTEN_WORDS_TASKS', sm_written_words, transitions={'succeeded':'RULE','aborted':'RULE','preempted':'RULE'})
            
            ''' Add the kitchen subtask(s) '''
            StateMachine.add('RULE', nav_states['explain_the_rule'], transitions={'succeeded':'RULE_TASKS','aborted':'LIFE','preempted':'LIFE'})
            
            # 当任务完成时, 继续进行bathroom任务
            StateMachine.add('RULE_TASKS', sm_rule, transitions={'succeeded':'LIFE','aborted':'LIFE','preempted':'LIFE'})
            
            ''' Add the bathroom subtask(s) '''
            StateMachine.add('LIFE', nav_states['explain_life'], transitions={'succeeded':'LIFE_TASKS','aborted':'FAITH','preempted':'FAITH'})
            
            # 当任务完成时, 继续进行hallway任务
            StateMachine.add('LIFE_TASKS', sm_life, transitions={'succeeded':'FAITH','aborted':'FAITH','preempted':'FAITH'})         
            
            ''' Add the hallway subtask(s) '''
            StateMachine.add('FAITH', nav_states['explain_faith'], transitions={'succeeded':'FAITH_TASKS','aborted':'','preempted':''})
            
            # 当任务完成时, stop
            StateMachine.add('FAITH_TASKS', sm_faith, transitions={'succeeded':'','aborted':'','preempted':''})         
                        
        # 创建并开始SMACH introspection server
        intro_server = IntrospectionServer('explain_history', sm_history_smach, '/SM_ROOT')
        intro_server.start()
        
        # 运行状态机
        sm_outcome = sm_history_smach.execute()
                
        if len(task_list) > 1:
            message = "Ooops! Not all chores were completed."
            message += "The following rooms need to be revisited: "
            message += str(task_list)
        else:
            message = "All chores complete!"
            
        rospy.loginfo(message)
        easygui.msgbox(message, title="Finished Cleaning")
        
        intro_server.stop()
            
    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            pass
            
    def cleaning_task_cb(self, userdata):
        rooms_to_clean.remove(userdata.room)
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        #sm_nav.request_preempt()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("House clearning test finished.")
