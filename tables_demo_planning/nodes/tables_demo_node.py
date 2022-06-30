#!/usr/bin/env python3
from typing import Dict, Optional, Set
import unified_planning
import rospy
import actionlib
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import Pose
from pbr_msgs.msg import PickObjectAction, PickObjectGoal
from unified_planning.model.problem import Problem
from unified_planning.shortcuts import And, Equals, Not, Or
from symbolic_fact_generation import on_fact_generator
from tables_demo_planning.demo_domain import ArmPose, Domain, Item, Location, Robot
from tables_demo_planning.subplan_visualization import SubPlanVisualization

"""
Main execution node of the tables demo.
Development in progress.
"""


class TablesDemoRobot(Robot):
    def __init__(self, domain: 'TablesDemo') -> None:
        super().__init__("mobipick")
        self.domain = domain
        self.activate_pose_selector = rospy.ServiceProxy('/pose_selector_activate', SetBool)
        self.open_gripper = rospy.ServiceProxy("/mobipick/pose_teacher/open_gripper", Empty)
        self.pick_object_action_client = actionlib.SimpleActionClient('/mobipick/pick_object', PickObjectAction)
        self.pick_object_goal = PickObjectGoal()

    def pick_item(self, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, look for item at location, pick it up, then move arm to transport pose."""
        if not self.pick_object_action_client.wait_for_server(timeout=rospy.Duration(2.0)):
            rospy.logerr("Pick Object action server not available!")
            return False

        perceived_item_locations = self.perceive()
        if item not in perceived_item_locations.keys():
            rospy.logwarn(f"Cannot find {item.name} at {location.name}. Pick up FAILED!")
            return False

        self.pick_object_goal.object_name = item.name
        self.pick_object_action_client.send_goal(self.pick_object_goal)
        if not self.pick_object_action_client.wait_for_result(timeout=rospy.Duration(50.0)):
            rospy.logwarn(f"Pick up {item.name} at {location.name} FAILED due to timeout!")
            return False

        result = self.pick_object_action_client.get_result()
        if not result:
            rospy.logwarn(f"Pick up {item.name} at {location.name} FAILED!")
            return False

        self.item = item
        self.domain.believed_item_locations[item] = self.domain.on_robot
        self.arm.move("transport")
        return True

    def place_klt(self, pose: Pose, location: Location) -> bool:
        """At pose, place klt down at location."""
        self.arm.move("above")
        self.open_gripper()
        self.item = Item.nothing
        self.domain.believed_item_locations[Item.klt] = self.domain.objects[location.name]
        self.arm.move("home")
        return True

    def store_item(self, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, store item into klt at location."""
        self.arm.move("above")
        self.open_gripper()
        self.item = Item.nothing
        self.domain.believed_item_locations[item] = Location.in_klt
        self.arm.move("home")
        return True

    def search_at(self, pose: Pose, location: Location) -> bool:
        """At pose, search for item_search at location."""
        item_locations = self.perceive()
        item = self.domain.item_search
        assert item
        if item in item_locations.keys():
            print(f"Search for {item.name} SUCCESSFUL.")
            self.domain.item_search = None
        return True

    def search_tool(self, item: Item) -> bool:
        """Initiate search for item."""
        self.domain.item_search = item
        return True

    def search_klt(self) -> bool:
        """Initiate search for klt."""
        self.domain.item_search = Item.klt
        return True

    def conclude_tool_search(self, item: Item) -> bool:
        """Conclude tool search as failed. Success is determined in search_at()."""
        print(f"Search for {item.name} FAILED!")
        return False

    def conclude_klt_search(self) -> bool:
        """Conclude klt search as failed. Success is determined in search_at()."""
        print("Search for klt FAILED!")
        return False

    def perceive(self) -> Dict[Item, Location]:
        """Move arm into observation pose and return all perceived items with locations."""
        self.arm.move("observe100cm_right")
        self.arm_pose = ArmPose.observe
        rospy.wait_for_service('/pose_selector_activate')
        self.activate_pose_selector(True)
        for table in (Location.table_1, Location.table_2, Location.table_3):
            on_fact_generator.clear_facts_and_poses_for_table(table.name)
        rospy.sleep(5)
        facts = on_fact_generator.get_current_facts()
        self.activate_pose_selector(False)
        perceived_item_locations: Dict[Item, Location] = {}
        for fact in facts:
            if fact.name == "on":
                item, table = fact.values
                print(f"{item} on {table} detected.")
                if item in self.enum_values(Item) and table in self.enum_values(Location):
                    perceived_item_locations[Item(item)] = Location(table)
        self.domain.believed_item_locations.update(perceived_item_locations)
        return perceived_item_locations


class TablesDemo(Domain):
    def __init__(self) -> None:
        super().__init__(TablesDemoRobot(self))
        self.believed_item_locations: Dict[Item, Location] = {}
        self.item_search: Optional[Item] = None
        self.target_table = self.table_2

        self.pick_item, (robot, pose, location, item) = self.create_action(TablesDemoRobot, TablesDemoRobot.pick_item)
        self.pick_item.add_precondition(Equals(self.robot_at(robot), pose))
        self.pick_item.add_precondition(
            Or(Equals(self.robot_arm_at(robot), arm_pose) for arm_pose in (self.arm_pose_home, self.arm_pose_observe))
        )
        self.pick_item.add_precondition(Equals(self.robot_has(robot), self.nothing))
        self.pick_item.add_precondition(Equals(self.believe_item_at(item), location))
        self.pick_item.add_precondition(Equals(self.pose_at(pose), location))
        self.pick_item.add_precondition(Not(Equals(location, self.anywhere)))
        self.pick_item.add_precondition(Not(Equals(item, self.nothing)))
        self.pick_item.add_effect(self.robot_has(robot), item)
        self.pick_item.add_effect(self.robot_arm_at(robot), self.arm_pose_transport)
        self.pick_item.add_effect(self.believe_item_at(item), self.anywhere)
        self.place_klt, (robot, pose, location) = self.create_action(TablesDemoRobot, TablesDemoRobot.place_klt)
        self.place_klt.add_precondition(Equals(self.robot_at(robot), pose))
        self.place_klt.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_transport))
        self.place_klt.add_precondition(Equals(self.robot_has(robot), self.klt))
        self.place_klt.add_precondition(Equals(self.pose_at(pose), location))
        self.place_klt.add_effect(self.robot_has(robot), self.nothing)
        self.place_klt.add_effect(self.robot_arm_at(robot), self.arm_pose_home)
        self.place_klt.add_effect(self.believe_item_at(self.klt), location)
        self.store_item, (robot, pose, location, item) = self.create_action(TablesDemoRobot, TablesDemoRobot.store_item)
        self.store_item.add_precondition(Equals(self.robot_at(robot), pose))
        self.store_item.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_transport))
        self.store_item.add_precondition(Equals(self.robot_has(robot), item))
        self.store_item.add_precondition(Equals(self.believe_item_at(self.klt), location))
        self.store_item.add_precondition(Equals(self.pose_at(pose), location))
        self.store_item.add_precondition(Not(Equals(location, self.anywhere)))
        self.store_item.add_precondition(Not(Equals(item, self.nothing)))
        self.store_item.add_effect(self.robot_has(robot), self.nothing)
        self.store_item.add_effect(self.robot_arm_at(robot), self.arm_pose_home)
        self.store_item.add_effect(self.believe_item_at(item), self.in_klt)
        self.search_at, (robot, pose, location) = self.create_action(TablesDemoRobot, TablesDemoRobot.search_at)
        self.search_at.add_precondition(Equals(self.robot_at(robot), pose))
        self.search_at.add_precondition(
            Or(
                Equals(self.robot_arm_at(robot), arm_pose)
                for arm_pose in (self.arm_pose_home, self.arm_pose_observe, self.arm_pose_transport)
            )
        )
        self.search_at.add_precondition(Not(self.searched_at(location)))
        self.search_at.add_precondition(Or(Equals(location, table) for table in self.tables))
        self.search_at.add_precondition(Equals(self.pose_at(pose), location))
        self.search_at.add_effect(self.searched_at(location), True)
        self.search_tool, (robot, item) = self.create_action(TablesDemoRobot, TablesDemoRobot.search_tool)
        self.search_tool.add_precondition(Not(Equals(self.robot_at(robot), self.tool_search_pose)))
        self.search_tool.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_home))
        self.search_tool.add_precondition(Equals(self.robot_has(robot), self.nothing))
        self.search_tool.add_precondition(Equals(self.believe_item_at(item), self.anywhere))
        self.search_tool.add_precondition(Not(Equals(item, self.klt)))
        self.search_tool.add_effect(self.robot_at(robot), self.tool_search_pose)
        self.search_tool.add_effect(self.believe_item_at(item), self.tool_search_location)
        self.search_klt, (robot,) = self.create_action(TablesDemoRobot, TablesDemoRobot.search_klt)
        self.search_klt.add_precondition(Not(Equals(self.robot_at(robot), self.klt_search_pose)))
        self.search_klt.add_precondition(
            Or(Equals(self.robot_arm_at(robot), arm_pose) for arm_pose in (self.arm_pose_home, self.arm_pose_transport))
        )
        self.search_klt.add_precondition(Equals(self.believe_item_at(self.klt), self.anywhere))
        self.search_klt.add_effect(self.robot_at(robot), self.klt_search_pose)
        self.search_klt.add_effect(self.believe_item_at(self.klt), self.klt_search_location)
        self.conclude_tool_search, (robot, item) = self.create_action(
            TablesDemoRobot, TablesDemoRobot.conclude_tool_search
        )
        self.conclude_tool_search.add_precondition(Equals(self.believe_item_at(item), self.anywhere))
        self.conclude_tool_search.add_precondition(And(self.searched_at(table) for table in self.tables))
        self.conclude_tool_search.add_precondition(Not(Equals(item, self.klt)))
        self.conclude_tool_search.add_effect(self.believe_item_at(item), self.tool_search_location)
        self.conclude_klt_search, (robot,) = self.create_action(TablesDemoRobot, TablesDemoRobot.conclude_klt_search)
        self.conclude_klt_search.add_precondition(Equals(self.believe_item_at(self.klt), self.anywhere))
        self.conclude_klt_search.add_precondition(And(self.searched_at(table) for table in self.tables))
        self.conclude_klt_search.add_effect(self.believe_item_at(self.klt), self.klt_search_location)

        self.problem = self.define_problem(
            fluents=(self.robot_at, self.robot_arm_at, self.robot_has, self.believe_item_at, self.pose_at),
            actions=(
                self.move_base,
                self.move_base_with_item,
                self.move_arm,
                self.pick_item,
                self.place_klt,
                self.store_item,
                self.search_tool,
                self.search_klt,
            ),
        )
        self.subproblem = self.define_problem(
            fluents=(
                self.robot_at,
                self.robot_arm_at,
                self.robot_has,
                self.believe_item_at,
                self.searched_at,
                self.pose_at,
            ),
            actions=(
                self.move_base,
                self.move_base_with_item,
                self.move_arm,
                self.search_at,
                self.conclude_tool_search,
                self.conclude_klt_search,
            ),
        )

    def set_initial_values(self, problem: Problem) -> None:
        """Set initial values for UP problem based on the fluents used and the current state."""
        super().set_initial_values(problem)
        if self.believe_item_at in problem.fluents:
            for item in Item:
                problem.set_initial_value(
                    self.believe_item_at(self.objects[item.name]),
                    self.objects[self.believed_item_locations[item].name]
                    if item in self.believed_item_locations.keys()
                    else self.anywhere,
                )

    def set_goals(self) -> None:
        """Set the goals for the overall demo."""
        self.problem.clear_goals()
        self.problem.add_goal(Equals(self.believe_item_at(self.multimeter), self.in_klt))
        self.problem.add_goal(Equals(self.believe_item_at(self.relay), self.in_klt))
        self.problem.add_goal(Equals(self.believe_item_at(self.screwdriver), self.in_klt))
        self.problem.add_goal(Equals(self.believe_item_at(self.klt), self.target_table))

    def set_search_goals(self) -> None:
        """Set the goals for the item_search subproblem."""
        assert self.item_search
        self.subproblem.clear_goals()
        self.subproblem.add_goal(
            Equals(self.believe_item_at(self.klt), self.klt_search_location)
            if self.item_search == Item.klt
            else Equals(self.believe_item_at(self.objects[self.item_search.name]), self.tool_search_location)
        )

    def print_believed_item_locations(self) -> None:
        """Print at which locations the items are believed to be."""
        for item in (Item.klt, Item.multimeter, Item.relay, Item.screwdriver):
            print(f"- {item.name}:", self.believed_item_locations.get(item, Location.anywhere).name)

    def run(self) -> None:
        print(f"Scenario: Mobipick shall bring all items inside the klt to {self.target_table}.")
        print("The believed item locations are:")

        visualization = SubPlanVisualization()
        successful_actions: Set[str] = set()
        subaction_success_count = 0
        # Solve overall problem.
        self.print_believed_item_locations()
        self.set_initial_values(self.problem)
        self.set_goals()
        actions = self.solve(self.problem)
        if not actions:
            print("Execution ended because no plan could be found.")
        # Loop action execution as long as there are actions.
        while actions:
            print("> Plan:")
            up_actions = [up_action for up_action, _ in actions]
            print('\n'.join(map(str, up_actions)))
            visualization.set_actions(
                [f"{len(successful_actions) + index + 1} {up_action}" for index, up_action in enumerate(up_actions)],
                successful_actions,
            )
            print("> Execution:")
            for up_action, (method, parameters) in actions:
                action_name = f"{len(successful_actions) + 1} {up_action}"
                print(up_action)
                visualization.execute(action_name)
                # Execute action.
                result = method(*parameters)
                if rospy.is_shutdown():
                    return

                # Handle item search as an inner execution loop.
                # Rationale: It has additional stop criteria, and might continue the outer loop.
                if self.item_search:
                    # Check whether an obsolete item search invalidates the previous plan.
                    if self.believed_item_locations.get(self.item_search, self.anywhere) != self.anywhere:
                        print(f"Search for {self.item_search.name} OBSOLETE.")
                        visualization.fail(action_name)
                        self.item_search = None
                        self.print_believed_item_locations()
                        self.set_initial_values(self.problem)
                        actions = self.solve(self.problem)
                        break

                    # Search for item by creating and executing a subplan.
                    self.set_initial_values(self.subproblem)
                    self.set_search_goals()
                    subactions = self.solve(self.subproblem)
                    assert subactions, f"No solution for: {self.subproblem}"
                    print("- Search plan:")
                    up_subactions = [up_subaction for up_subaction, _ in subactions]
                    print('\n'.join(map(str, up_subactions)))
                    visualization.set_actions(
                        [
                            f"{len(successful_actions) + 1}{chr(index + 97)} {up_subaction}"
                            for index, up_subaction in enumerate(up_subactions)
                        ],
                        successful_actions,
                        action_name,
                    )
                    print("- Search execution:")
                    subaction_success_count = 0
                    for up_subaction, (submethod, subparameters) in subactions:
                        subaction_name = (
                            f"{len(successful_actions) + 1}{chr(subaction_success_count + 97)} {up_subaction}"
                        )
                        print(up_subaction)
                        visualization.execute(subaction_name)
                        # Execute search action.
                        result = submethod(*subparameters)
                        if rospy.is_shutdown():
                            return

                        if result is not None:
                            if result:
                                # Note: True result only means any subaction succeeded.
                                # Check if the search actually succeeded.
                                if self.item_search is None:
                                    visualization.succeed(subaction_name)
                                    print("- Continue with plan.")
                                    break
                            else:
                                visualization.fail(subaction_name)
                                break
                        subaction_success_count += 1
                    self.item_search = None

                if result is not None:
                    if result:
                        visualization.succeed(action_name)
                        successful_actions.add(action_name)
                    else:
                        visualization.fail(action_name)
                        self.print_believed_item_locations()
                        self.set_initial_values(self.problem)
                        actions = self.solve(self.problem)
                        break
            else:
                print("Task complete.")
                break


if __name__ == '__main__':
    unified_planning.shortcuts.get_env().credits_stream = None
    try:
        TablesDemo().run()
    except rospy.ROSInterruptException:
        pass
