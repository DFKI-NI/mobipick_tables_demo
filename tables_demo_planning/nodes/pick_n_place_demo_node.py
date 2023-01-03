#!/usr/bin/env python3


# Software License Agreement (BSD License)
#
#  Copyright (c) 2022, DFKI GmbH
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors: Alexander Sung


from typing import Optional, Set
import rospy
import unified_planning
from std_msgs.msg import String
from unified_planning.model import Object, Problem
from unified_planning.model.metrics import MinimizeSequentialPlanLength
from unified_planning.shortcuts import Not
from tables_demo_planning.mobipick_components import APIRobot, ArmPose, EnvironmentRepresentation, Item
from tables_demo_planning.demo_domain import Domain
from tables_demo_planning.subplan_visualization import SubPlanVisualization

"""Pick and Place application in the Mobipick domain"""


class PickAndPlaceRobot(APIRobot):
    def __init__(self, namespace: str, env: 'PickAndPlaceEnv') -> None:
        APIRobot.__init__(self, namespace)
        self.env = env

    def get_item(self) -> Item:
        """Return Item.power_drill if robot arm has an object attached, else Item.nothing."""
        self.arm.execute("HasAttachedObjects")
        return Item.power_drill if self.arm.get_result().result else Item.nothing

    def pick_power_drill(self) -> bool:
        """Pick up power drill."""
        self.arm.execute("CaptureObject")
        self.arm_pose = self.get_arm_pose()
        self.arm.execute("PickUpObject")
        self.arm_pose = self.get_arm_pose()
        if not self.arm.get_result().result:
            return False

        self.item = Item.power_drill
        return True

    def place_power_drill(self) -> bool:
        """Place power drill onto table."""
        self.arm.execute("PlaceObject")
        self.arm_pose = ArmPose.place
        self.item = Item.nothing
        return True

    def hand_over(self) -> bool:
        """Hand over item and observe force torque feedback."""
        self.arm.execute("MoveArmToHandover")
        self.arm_pose = self.get_arm_pose()
        self.env.item_offered = True
        if not self.arm.observe_force_torque(5.0, 25.0):
            return False

        self.arm.execute("ReleaseGripper")
        return True


class PickAndPlaceEnv(EnvironmentRepresentation[PickAndPlaceRobot]):
    def __init__(self) -> None:
        EnvironmentRepresentation.__init__(self, PickAndPlaceRobot("mobipick", self))
        self.item_offered = False

    def get_item_offered(self) -> bool:
        """Return fluent value whether item has been offered."""
        return self.item_offered


class PickAndPlaceDomain(Domain[PickAndPlaceEnv]):
    SCENARIO_POSE_NAMES = (
        Domain.BASE_HANDOVER_POSE_NAME,
        Domain.BASE_HOME_POSE_NAME,
        Domain.BASE_TABLE_2_POSE_NAME,
        Domain.BASE_TABLE_3_POSE_NAME,
    )

    def __init__(self) -> None:
        Domain.__init__(self, PickAndPlaceEnv())
        self.env.robot.add_waypoints(
            {pose_name: pose for pose_name, pose in self.api_poses.items() if pose_name in self.SCENARIO_POSE_NAMES}
        )
        self.env.robot.initialize(self.api_poses[self.BASE_HOME_POSE_NAME], *self.env.robot.get())
        self.set_fluent_functions([self.get_robot_at])
        self.item_offered = self.create_fluent_from_function(self.env.get_item_offered)

        self.set_api_actions(
            (PickAndPlaceRobot.move_base, PickAndPlaceRobot.move_base_with_item, PickAndPlaceRobot.move_arm)
        )
        self.pick, (_,) = self.create_action_from_function(PickAndPlaceRobot.pick_power_drill)
        self.pick.add_precondition(self.robot_at(self.base_table_2_pose))
        self.pick.add_precondition(self.robot_arm_at(self.arm_pose_home))
        self.pick.add_precondition(self.robot_has(self.nothing))
        self.pick.add_effect(self.robot_arm_at(self.arm_pose_home), False)
        self.pick.add_effect(self.robot_arm_at(self.arm_pose_interaction), True)
        self.pick.add_effect(self.robot_has(self.nothing), False)
        self.pick.add_effect(self.robot_has(self.power_drill), True)
        self.place, (_,) = self.create_action_from_function(PickAndPlaceRobot.place_power_drill)
        self.place.add_precondition(self.robot_at(self.base_table_3_pose))
        self.place.add_precondition(self.robot_arm_at(self.arm_pose_transport))
        self.place.add_precondition(self.robot_has(self.power_drill))
        self.place.add_effect(self.robot_arm_at(self.arm_pose_transport), False)
        self.place.add_effect(self.robot_arm_at(self.arm_pose_interaction), True)
        self.place.add_effect(self.robot_has(self.power_drill), False)
        self.place.add_effect(self.robot_has(self.nothing), True)
        self.hand_over, (_,) = self.create_action_from_function(PickAndPlaceRobot.hand_over)
        self.hand_over.add_precondition(self.robot_at(self.base_handover_pose))
        self.hand_over.add_precondition(self.robot_arm_at(self.arm_pose_transport))
        self.hand_over.add_precondition(Not(self.robot_has(self.nothing)))
        self.hand_over.add_effect(self.robot_arm_at(self.arm_pose_transport), False)
        self.hand_over.add_effect(self.robot_arm_at(self.arm_pose_interaction), True)
        for item in self.items:
            self.hand_over.add_effect(self.robot_has(item), item == self.nothing)
        self.hand_over.add_effect(self.item_offered, True)
        self.visualization: Optional[SubPlanVisualization] = None
        self.method_labels.update(
            {
                self.pick: lambda _: "Pick up power drill",
                self.place: lambda _: "Place power drill on table",
                self.hand_over: lambda _: "Hand over power drill to person",
            }
        )
        self.espeak_pub = rospy.Publisher("/espeak_node/speak_line", String, queue_size=1)

    def get_robot_at(self, pose: Object) -> bool:
        base_pose_name = self.env.robot.base.get_pose_name()
        base_pose = self.objects.get(base_pose_name, self.unknown_pose)
        return pose == base_pose

    def initialize_problem(self) -> Problem:
        """Initialize current problem."""
        actions = [self.move_base, self.move_base_with_item, self.move_arm, self.pick, self.place]
        if not self.env.item_offered:
            actions.append(self.hand_over)
        return self.define_mobipick_problem(
            actions=actions,
            poses=[self.objects[pose_name] for pose_name in self.SCENARIO_POSE_NAMES],
            items=[self.power_drill],
            locations=[],
        )

    def set_goals(self, problem: Problem) -> None:
        """Set the goals of the pick and place demo."""
        problem.add_goal(self.robot_at(self.base_home_pose))
        problem.add_goal(self.robot_has(self.nothing))
        problem.add_goal(self.item_offered)

    def run(self) -> None:
        """Run the pick and place demo."""
        self.visualization = SubPlanVisualization()
        executed_actions: Set[str] = set()
        error_count = 0
        active = True
        while active:
            # Create problem based on current state.
            problem = self.initialize_problem()
            problem.add_quality_metric(MinimizeSequentialPlanLength())
            self.set_initial_values(problem)
            self.set_goals(problem)

            # Plan
            actions = self.solve(problem)
            if not actions:
                print("Execution ended because no plan could be found.")
                self.visualization.add_node("Mission impossible", "red")
                return

            print("> Plan:")
            print('\n'.join(map(str, actions)))
            action_names = [
                f"{len(executed_actions) + number} {self.label(action)}"
                for number, action in enumerate(actions, start=1)
            ]
            self.visualization.set_actions(action_names, preserve_actions=executed_actions)
            # ... and execute.
            print("> Execution:")
            for action in actions:
                function, parameters = self.get_executable_action(action)
                action_name = f"{len(executed_actions) + 1} {self.label(action)}"
                print(action)
                self.visualization.execute(action_name)
                self.espeak_pub.publish(self.label(action))
                result = function(*parameters)
                executed_actions.add(action_name)
                if rospy.is_shutdown():
                    return

                if result is None or result:
                    self.visualization.succeed(action_name)
                else:
                    print("-- Action failed! Need to replan.")
                    error_count += 1
                    self.visualization.fail(action_name)
                    self.espeak_pub.publish("Action failed.")
                    if error_count >= 3:
                        print("Execution ended after too many failures.")
                        self.espeak_pub.publish("Mission impossible!")
                        self.visualization.add_node("Mission impossible", "red")
                        return

                    # Abort execution and loop to planning.
                    break
            else:
                active = False
                print("Demo complete.")
                self.espeak_pub.publish("Demo complete.")
                self.visualization.add_node("Demo complete", "green")


if __name__ == '__main__':
    unified_planning.shortcuts.get_env().credits_stream = None
    try:
        PickAndPlaceDomain().run()
    except rospy.ROSInterruptException:
        pass
