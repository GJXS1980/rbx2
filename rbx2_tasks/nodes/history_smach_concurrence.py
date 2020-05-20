#!/usr/bin/env python
#! -*- coding: utf-8 -*-

""" patrol_smach_concurrence.py - Version 1.0 2013-04-12

    Control a robot using SMACH to patrol around a square a specified number of times
    while monitoring battery levels using the Concurrence container.

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
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
from rbx2_tasks.history_task_setup import *

#   讲解任务的列表
task_list = {'written_words':['explanatory_text'], 'rule':['explain_the_rule'], 'life':['explain_life'], 'faith':['explain_faith']}

class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass
    
    def execute(self, userdata):
        rospy.loginfo("Shutting down the state machine")
        return 'succeeded'

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
        # easygui.msgbox(message, title="Succeeded")
        
        # update_task_list(self.section, self.task)

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
        # easygui.msgbox(message, title="Succeeded")
        
        # update_task_list(self.section, self.task)
             
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
        # easygui.msgbox(message, title="Succeeded")
        
        # update_task_list(self.section, self.task)
            
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
        # easygui.msgbox(message, title="Succeeded")
        
        # update_task_list(self.section, self.task)
            
        return 'succeeded'

def update_task_list(section, task):
    task_list[section].remove(task)
    if len(task_list[section]) == 0:
        del task_list[section]        
 

class History_Smach():
    def __init__(self):
        rospy.init_node('explain_history_concurrence', anonymous=False)
        
        # 设置关闭机器人函数(stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # 初始化一些参数和变量
        setup_task_environment(self)
        
        # 跟踪到达目标位置的成功率
        self.n_succeeded = 0
        self.n_aborted = 0
        self.n_preempted = 0
        
        # 保存上一个或者当前的导航目标点的变量
        self.last_nav_state = None
        
        # 指示是否正在充电的标志
        self.recharging = False
        
        # 保存导航目标点的列表
        nav_states = {}
        
        # 把waypoints变成状态机的状态
        for waypoint in self.room_locations.iterkeys():           
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = self.room_locations[waypoint]
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(10.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
            # nav_states.append(move_base_state)
            nav_states[waypoint] = move_base_state
            
        # 为扩展底座（docking station）创建一个MoveBaseAction state
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.pose = self.docking_station_pose
        nav_docking_station = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                             exec_timeout=rospy.Duration(20.0),
                                             server_wait_timeout=rospy.Duration(10.0))

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

        # 初始化导航的状态机
        self.sm_nav = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        # 使用transitions将导航的状态添加到状态机
        with self.sm_nav:
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
       

        # 在sm_nav状态机中注册一个回调函数以启动状态转换（state transitions）
        self.sm_nav.register_transition_cb(self.nav_transition_cb, cb_args=[])

        # 初始化充电的状态机
        self.sm_recharge = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        with self.sm_recharge:
            StateMachine.add('NAV_DOCKING_STATION', nav_docking_station, transitions={'succeeded':'RECHARGE_BATTERY'})
            StateMachine.add('RECHARGE_BATTERY', ServiceState('battery_simulator/set_battery_level', SetBatteryLevel, 100, response_cb=self.recharge_cb), 
                             transitions={'succeeded':''})        

        # 使用并发容器（Concurrence container）创建nav_patrol状态机
        self.nav_patrol = Concurrence(outcomes=['succeeded', 'recharge', 'stop'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)
        
        # 将sm_nav machine和battery MonitorState添加到nav_patrol状态机里面         
        with self.nav_patrol:
           Concurrence.add('SM_NAV', self.sm_nav)
           Concurrence.add('MONITOR_BATTERY', MonitorState("battery_level", Float32, self.battery_cb))
        
        # 创建顶层状态机
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        
        # 将nav_patrol,sm_recharge和Stop添加到sm_top状态机
        with self.sm_top:
            StateMachine.add('PATROL', self.nav_patrol, transitions={'succeeded':'PATROL', 'recharge':'RECHARGE', 'stop':'STOP'}) 
            StateMachine.add('RECHARGE', self.sm_recharge, transitions={'succeeded':'PATROL'})
            StateMachine.add('STOP', Stop(), transitions={'succeeded':''})

        # 创建并开始SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()
        
        # 运行状态机
        sm_outcome = self.sm_top.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()
    
    def nav_transition_cb(self, userdata, active_states, *cb_args):
        self.last_nav_state = active_states
        
    # 当任何子状态终止时调用
    def concurrence_child_termination_cb(self, outcome_map):
        # 如果当前导航任务完成, return True
        if outcome_map['SM_NAV'] == 'succeeded':
            return True
        # 如果MonitorState状态变成False则储存当前的导航目标点并充电
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            rospy.loginfo("LOW BATTERY! NEED TO RECHARGE...")
            if self.last_nav_state is not None:
                self.sm_nav.set_initial_state(self.last_nav_state, UserData())
            return True
        else:
            return False
    
    # 当任何子状态终止时调用
    def concurrence_outcome_cb(self, outcome_map):
        # 如果电池电量低于设定的阈值,返回'recharge' outcome
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            return 'recharge'
        # 否则,如果最后一个导航目标点成功,返回'succeeded' 或者 'stop'
        elif outcome_map['SM_NAV'] == 'succeeded':
            self.patrol_count += 1
            rospy.loginfo("FINISHED PATROL LOOP: " + str(self.patrol_count))
            # 如果没有完成所有的巡逻，重新开始导航
            if self.n_patrols == -1 or self.patrol_count < self.n_patrols:
                # self.sm_nav.set_initial_state(['NAV_STATE_0'], UserData())
                return 'succeeded'
            # 否则,完成所有导航并返回 'stop'
            else:
                # self.sm_nav.set_initial_state(['NAV_STATE_4'], UserData())
                return 'stop'
        # 如果其他操作失败了，重新充电
        else:
            return 'recharge'
        
    def battery_cb(self, userdata, msg):
        if msg.data < self.low_battery_threshold:
            self.recharging = True
            return False
        else:
            self.recharging = False
            return True
        
    def recharge_cb(self, userdata, response):
        return 'succeeded'
        
    def move_base_result_cb(self, userdata, status, result):
        if not self.recharging:
            if status == actionlib.GoalStatus.SUCCEEDED:
                self.n_succeeded += 1
            elif status == actionlib.GoalStatus.ABORTED:
                self.n_aborted += 1
            elif status == actionlib.GoalStatus.PREEMPTED:
                self.n_preempted += 1
    
            try:
                rospy.loginfo("Success rate: " + str(100.0 * self.n_succeeded / (self.n_succeeded + self.n_aborted  + self.n_preempted)))
            except:
                pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        self.sm_nav.request_preempt()
        
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        History_Smach()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")
