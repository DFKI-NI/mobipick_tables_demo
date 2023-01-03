#!/usr/bin/env python3


# Copyright 2022 DFKI GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors:
# - Sebastian Stock, DFKI

import rospy
import unified_planning
from typing import Optional, Set
from pick_n_place_demo_node import PickAndPlaceDomain
from unified_planning.plans import Plan, ActionInstance
from unified_planning.shortcuts import OneshotPlanner, Problem
from unified_planning.engines import OptimalityGuarantee
from up_bridge.plexmo import SequentialPlanDispatcher
from up_bridge.plexmo import SequentialPlanMonitor
from unified_planning.model import UPCOWState


class PickAndPlaceOrchestrator:
    def __init__(self) -> None:
        self._domain = PickAndPlaceDomain()

    def solve_problem(self, problem: Problem, planner_name: Optional[str] = None) -> Plan:
        result = OneshotPlanner(
            name=planner_name,
            problem_kind=problem.kind,
            optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY,
        ).solve(problem)
        return result.plan

    def dispatch_cb(self, action: ActionInstance) -> bool:
        action_name = f"{len(self._executed_actions) + 1} {self._domain.label(action)}"

        self._domain.set_initial_values(self._problem)
        monitor = SequentialPlanMonitor(self._problem)
        state = UPCOWState(self._problem.initial_values)
        unsatisfied = monitor.check_preconditions(action, state)[1]
        if len(unsatisfied) > 0:
            print("Preconditions of action %s are not satisfied" % action_name)
            print("Unsatisfied preconditions: %s" % str(unsatisfied))
            print("State is: ")
            for k, v in self._problem.initial_values.items():
                print(f"  {str(k)} := {str(v)}\n")
            # return False   # commented out until issue with arm state is resolved

        print("Dispatching " + action_name)
        executable_action, parameters = self._domain.get_executable_action(action)
        result = executable_action(*parameters)
        self._executed_actions.add(action_name)
        if result is None or result:
            return True
        else:
            return False

    def generate_and_execute_plan(self) -> None:
        self._executed_actions: Set[object] = set()

        # generate problem and plan
        self._problem = self._domain.initialize_problem()
        self._domain.set_initial_values(self._problem)
        self._domain.set_goals(self._problem)  # TODO take goal from external action
        # TODO use bridge.solve() once it returns plan
        print(self._problem)
        plan = self.solve_problem(self._problem)

        if not plan:
            print("Could not find a plan. Exiting.")
            return

        print("> Plan:")
        print("\n".join(map(str, plan.actions)))

        # execute the plan
        print("> Execution:")
        dispatcher = SequentialPlanDispatcher()
        dispatcher.set_dispatch_callback(self.dispatch_cb)
        dispatcher_result = dispatcher.execute_plan(plan)
        print(">Dispatcher finished with result " + str(dispatcher_result))


if __name__ == "__main__":
    unified_planning.shortcuts.get_env().credits_stream = None
    try:
        PickAndPlaceOrchestrator().generate_and_execute_plan()
    except rospy.ROSInterruptException:
        pass
