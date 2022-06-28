#!/usr/bin/env python3
from typing import Optional
import unified_planning
from unified_planning.model.problem import Problem
from unified_planning.shortcuts import Equals
from tables_demo_planning.demo_domain import ArmPose, Domain, Item, Robot
from tables_demo_planning.plan_visualization import PlanVisualization

"""Execution of the pick & place demo."""


class PickAndPlaceRobot(Robot):
    def get_initial_item(self) -> Item:
        self.arm.execute("HasAttachedObjects")
        return Item.power_drill if self.arm.get_result().result else Item.nothing

    def pick_power_drill(self) -> bool:
        self.arm.execute("CaptureObject")
        self.arm_pose = self.get_arm_pose()
        self.arm.execute("PickUpObject")
        self.arm_pose = self.get_arm_pose()
        if not self.arm.get_result().result:
            return False

        self.item = Item.power_drill
        return True

    def place_power_drill(self) -> bool:
        self.arm.execute("PlaceObject")
        self.arm_pose = ArmPose.place
        self.item = Item.nothing
        return True


class PickAndPlace(Domain):
    def __init__(self) -> None:
        super().__init__(PickAndPlaceRobot("mobipick"))
        self.pick, (robot,) = self.create_action(PickAndPlaceRobot, PickAndPlaceRobot.pick_power_drill)
        self.pick.add_precondition(Equals(self.robot_has(robot), self.nothing))
        self.pick.add_precondition(Equals(self.robot_at(robot), self.base_pick_pose))
        self.pick.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_home))
        self.pick.add_effect(self.robot_has(robot), self.power_drill)
        self.pick.add_effect(self.robot_arm_at(robot), self.arm_pose_interaction)
        self.place, (robot,) = self.create_action(PickAndPlaceRobot, PickAndPlaceRobot.place_power_drill)
        self.place.add_precondition(Equals(self.robot_has(robot), self.power_drill))
        self.place.add_precondition(Equals(self.robot_at(robot), self.base_place_pose))
        self.place.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_transport))
        self.place.add_effect(self.robot_has(robot), self.nothing)
        self.place.add_effect(self.robot_arm_at(robot), self.arm_pose_interaction)
        self.visualization: Optional[PlanVisualization] = None

    def initialize_problem(self) -> Problem:
        actions = [self.move_base, self.move_base_with_item, self.move_arm, self.pick, self.place]
        if not self.api_robot.item_offered:
            actions.append(self.hand_over)
        return self.define_problem(
            fluents=(self.robot_at, self.robot_arm_at, self.robot_has, self.robot_offered),
            items=(self.nothing, self.power_drill),
            locations=[],
            actions=actions,
        )

    def set_goals(self, problem: Problem) -> None:
        problem.add_goal(self.robot_offered(self.robot))
        problem.add_goal(Equals(self.robot_has(self.robot), self.nothing))
        problem.add_goal(Equals(self.robot_at(self.robot), self.base_home_pose))

    def run(self) -> None:
        active = True
        while active:
            # Create problem based on current state.
            self.problem = self.initialize_problem()
            self.set_initial_values(self.problem)
            self.set_goals(self.problem)

            # Plan
            actions = self.solve(self.problem)
            if not actions:
                print("Execution ended because no plan could be found.")
                break

            print("> Plan:")
            up_actions = [up_action for up_action, _ in actions]
            print('\n'.join(map(str, up_actions)))
            if self.visualization:
                self.visualization.set_actions(up_actions)
            else:
                self.visualization = PlanVisualization(up_actions)
            # ... and execute.
            print("> Execution:")
            for up_action, (method, parameters) in actions:
                print(up_action)
                self.visualization.execute(up_action)
                result = method(*parameters)
                if result is None or result:
                    self.visualization.succeed(up_action)
                else:
                    print("-- Action failed! Need to replan.")
                    self.visualization.fail(up_action)
                    # Abort execution and loop to planning.
                    break
            else:
                active = False
                print("Task complete.")


if __name__ == '__main__':
    unified_planning.shortcuts.get_env().credits_stream = None
    PickAndPlace().run()
