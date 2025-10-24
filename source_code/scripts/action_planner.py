#!/usr/bin/env python3

import rclpy, os, sys, subprocess, re, time, json
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
         
from ament_index_python.packages import get_package_share_directory
package_share_path = get_package_share_directory("route_executor2")
scripts_path = os.path.join(package_share_path, 'scripts')
sys.path.append(scripts_path)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from action_planner_memory import ActionPlannerMemory
from action_planner_executor import ActionPlannerExecutor
from harpia_msgs.action import ExecutePlan
from harpia_msgs.srv import StrInOut
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

class ActionPlanner(LifecycleNode):

    def __init__(self):
        super().__init__('action_planner')

        self.declare_parameter("pddl_domain", "")
        self.domain_file = self.get_parameter("pddl_domain").get_parameter_value().string_value

        self.solver = "TFD"
        # self.solver = "OPTIC"

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring action_planner node...")

        
        try:
            self.get_logger().info("Configuring node...")
            self.memory = ActionPlannerMemory()
            if not self.memory.init(self.domain_file, self.get_logger()):
                return TransitionCallbackReturn.ERROR
            
            self.plan_executor = ActionPlannerExecutor(self, self.memory)
        
        except Exception as e:
            self.get_logger().error(f"Error during configuration: {e}")
            return TransitionCallbackReturn.ERROR   

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node...")

        try:

            self.action_cb = ReentrantCallbackGroup()
            self.execute_plan_action_server = ActionServer(
                self,
                ExecutePlan,
                'action_planner/execute_plan',
                execute_callback   = self.exe_plan_execute,
                goal_callback      = self.exe_plan_goal_request,
                cancel_callback    = self.exe_plan_cancel_request,
                callback_group     = self.action_cb
            )
            
            self.update_parameter_service = self.create_service(
                StrInOut,
                "plansys_interface/update_parameters",
                self.update_parameters_callback
            )
            
        except Exception as e:
            self.get_logger().error(f"Error during activation: {e}")
            return TransitionCallbackReturn.ERROR
        
        self._can_receive_execute_plan_goal = True
        self._plan_finished_success = (False, False) # first is if finished, second is if successful
        
        # # show the battery amount every second
        # self.last_battery_that_was_shown = None
        # def show_battery_callback():
        #     battery_amount = self.memory.get_function_value("battery_amount", [])
        #     if battery_amount is None:
        #         self.get_logger().info(f"bat: None")
        #         return
        #     battery_amount = round(float(battery_amount))
        #     if battery_amount != self.last_battery_that_was_shown:
        #         self.get_logger().info(f"bat: {battery_amount:.0f}%")
        #         self.last_battery_that_was_shown = battery_amount
        # self.create_timer(1, show_battery_callback)

        # self.start_bat_sim()

        return TransitionCallbackReturn.SUCCESS
    
    def generate_plan_custom_solver(self):
        """
        Write domain/problem PDDL to files under <package_share>/solver,
        call the external generate_plan.sh with the chosen solver, parse
        the output into a plan list, and send it to plan_executor.
        """

        project_name = "route_executor2"
        share_path = get_package_share_directory(project_name)
        solver_dir = os.path.join(share_path, "solver")
        domain_path = os.path.join(solver_dir, "domain.pddl")
        problem_path = os.path.join(solver_dir, "problem.pddl")

        domain_pddl_text, problem_pddl_text = self.memory.get_pddl()

        #print domain and problem PDDL
        # self.get_logger().info("Domain PDDL:")
        # self.get_logger().info(domain_pddl_text)
        # self.get_logger().info("Problem PDDL:")
        # self.get_logger().info(problem_pddl_text)

        try:
            with open(domain_path, "w") as f:
                f.write(domain_pddl_text)
        except Exception as e:
            self.get_logger().error(f"Failed to write domain file: {e}")
            return False

        try:
            with open(problem_path, "w") as f:
                f.write(problem_pddl_text)
        except Exception as e:
            self.get_logger().error(f"Failed to write problem file: {e}")
            return False

        # write domain and problem to output folder
        to_share = f'/install/{project_name}/share/{project_name}'
        if to_share in share_path:
            main_project_path = share_path.replace(to_share, '')
        else:
            main_project_path = share_path
        try:
            asd = os.path.join(main_project_path, 'output/domain.pddl')
            with open(asd, "w") as f:
                f.write(domain_pddl_text)
        except Exception as e:
            self.get_logger().error(f"Failed to write domain file: {e}")
            return False
        try:
            asd = os.path.join(main_project_path, 'output/problem.pddl')
            with open(asd, "w") as f:
                f.write(problem_pddl_text)
        except Exception as e:
            self.get_logger().error(f"Failed to write problem file: {e}")
            return False
        
        solver_subdir = os.path.join(solver_dir, self.solver)
        generate_script = os.path.join(solver_subdir, "generate_plan.sh")
        cmd = [generate_script, domain_path, problem_path]

        # self.get_logger().info(f"Calling external solver: {' '.join(cmd)}")
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False  # We’ll check returncode ourselves
            )
        except FileNotFoundError:
            self.get_logger().error(f"Solver script not found: {generate_script}")
            return False
        except Exception as e:
            self.get_logger().error(f"Error when running solver: {e}")
            return False

        if result.returncode != 0:
            self.get_logger().error(f"Solver returned non-zero exit code {result.returncode}:\n{result.stderr}")
            return False

        regex_pattern = r"^(\d+\.\d+):\s\(([^)]+)\)\s\[(\d+\.\d+)\]$"
        plan = []

        for line in result.stdout.splitlines():
            self.get_logger().debug(f"Parsing line: {line.strip()}")
            match = re.match(regex_pattern, line.strip())

            if match:
                start_time = float(match.group(1))
                action_parameters = match.group(2).split()
                action_name = action_parameters.pop(0)
                duration = float(match.group(3))

                plan.append((action_name, action_parameters))

        self.get_logger().info(f"Generated plan with {len(plan)} actions.")
        return plan

    def exe_plan_goal_request(self, goal_request):

        if not self._can_receive_execute_plan_goal: # this check exists because the next check [self.plan_executor.is_executing()] only works after self.plan_executor.execute_plan() is called
            self.get_logger().warn("Cannot receive new goals at the moment, rejecting goal.")
            return GoalResponse.REJECT
        
        if self.plan_executor.is_executing():
            self.get_logger().warn("Already executing a plan, rejecting new goal.")
            return GoalResponse.REJECT
        
        self.plan = self.generate_plan_custom_solver()
        # self.get_logger().info(f"Generated plan: {self.plan}")

        if not self.plan:
            self.get_logger().error("Failed to generate a valid plan, rejecting goal.")
            return GoalResponse.REJECT
        
        self.get_logger().info("Plan generated successfully, accepting goal.")
        self._can_receive_execute_plan_goal = False
        return GoalResponse.ACCEPT 
        
    def exe_plan_cancel_request(self, goal_handle):
        # return CancelResponse.REJECT
        if self._is_cancelling:
            self.get_logger().warn("Already cancelling a plan execution, rejecting cancel request.")
            return CancelResponse.REJECT
        self.get_logger().info("Accepting cancel request")
        self._is_cancelling = True
        return CancelResponse.ACCEPT
    

    def exe_plan_execute(self, goal_handle):
        
        def on_success():
            self.get_logger().info("Plan execution completed successfully.")
            self._plan_finished_success = (True, True)

        def on_failure():
            self.get_logger().error("Plan execution failed.")
            self._plan_finished_success = (True, False)

        def on_finish_cancel():
            self._plan_finished_success = (True, False)
            self._finished_cancel = True
            # goal_handle.canceled()

        def check_if_plan_is_cancelling():
            return self._is_cancelling
        
        self._can_receive_execute_plan_goal = True
        self._plan_finished_success = (False, False)
        self._is_cancelling = False
        self._finished_cancel = False

        self.plan_executor.execute_plan(
            self.plan,
            check_if_plan_is_cancelling,
            on_success,
            on_failure,
            on_finish_cancel)
        

        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Plan execution is canceling, waiting...')
                while not self._finished_cancel:
                    time.sleep(0.1)
                goal_handle.canceled() 
                result = ExecutePlan.Result()
                result.success = self._plan_finished_success[1]
                return result


            feedback = ExecutePlan.Feedback()
            feedback.step = 0
            feedback.nofsteps = 0
            goal_handle.publish_feedback(feedback)

            if self._plan_finished_success[0]:
                break
            time.sleep(0.1)

        self.get_logger().info("Plan execution finished.")
        result = ExecutePlan.Result()
        result.success = self._plan_finished_success[1]
        goal_handle.succeed()
        return result
    

    def update_parameters_callback(self, request, response):
        """
        Callback for the 'plansys_interface/update_parameters' service.
        """

        try:
            json_commands = json.loads(request.message)
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
            response.success = False
            response.message = "Failed to parse JSON"
            return response
        
        if not isinstance(json_commands, list):
            self.get_logger().error("JSON commands must be a list.")
            response.success = False
            response.message = "JSON must be a list"
            return response
        
        if not hasattr(self, 'update_parameters_commands'):
            self.update_parameters_commands = {
                'add_instances': self.add_instances,
                'remove_instances': self.remove_instances,
                'add_predicates': self.add_predicates,
                'remove_predicates': self.remove_predicates,
                'add_functions': self.update_functions,
                'update_functions': self.update_functions,
                'remove_functions': self.remove_functions,
                'set_goals': self.set_goals,
                'clear_goal': self.clear_goal,
                'clear_knowledge': self.clear_knowledge,
                'log_pddl': self.log_pddl,
            }

        # self.get_logger().info(f"Received commands: {json_commands}")

        try:

            for command in json_commands:
                try:
                    command_type = command.get("type")
                    values = command.get("values")
                except Exception as e:
                    self.get_logger().error(f"Invalid command format: {e}")
                    response.success = False
                    response.message = "Invalid command format"
                    return response

                if not command_type in self.update_parameters_commands:
                    self.get_logger().error(f"Command type '{command_type}' not recognized.")
                    response.success = False
                    response.message = f"Command type '{command_type}' not recognized."
                    return response

                if not self.update_parameters_commands[command_type](values):
                    self.get_logger().error(f"Command '{command_type}' failed.")
                    response.success = False
                    response.message = f"Command '{command_type}' failed."
                    return response
        except Exception as e:
            self.get_logger().error(f"Error processing commands: {e}")
            response.success = False
            response.message = f"Error processing commands: {e}"
            return response
        
        response.success = True
        response.message = ""
        return response
    
    def add_instances(self, instances):
        for instance in instances:
            name, type = instance.split(" ")
            if not self.memory.add_instance(type, name):
                self.get_logger().error(f"Failed")
                return False
        return True
        
    def remove_instances(self, instances):
        raise NotImplementedError("Remove instances is not implemented yet.")

    def add_predicates(self, predicates):
        for predicate in predicates:
            instance_names = predicate.split(" ")
            predicate_name = instance_names.pop(0)
            # self.get_logger().info(f"Adding predicate: {predicate_name} with instances {instance_names}")
            if not self.memory.add_predicate(predicate_name, instance_names):
                return False
        return True
    def remove_predicates(self, predicates):
        for predicate in predicates:
            instance_names = predicate.split(" ")
            predicate_name = instance_names.pop(0)
            # self.get_logger().info(f"Removing predicate: {predicate_name} with instances {instance_names}")
            if not self.memory.remove_predicate(predicate_name, instance_names):
                return False
        return True
            
    # def add_functions(self, functions): # we do not need this anymore
    #     raise NotImplementedError("Remove functions is not implemented yet.")
    def remove_functions(self, functions):
        raise NotImplementedError("Remove functions is not implemented yet.")
    def update_functions(self, functions):
        for function in functions:
            # self.get_logger().info(f"Updating function: {function}")
            match = re.match(r"\(([^)]+)\)\s+([+-]?\d+(?:\.\d+)?)", function)
            if not match:
                self.get_logger().error(f"Error while parsing function: '{function}'")
                return False
            func_args = match.group(1).split()
            value = float(match.group(2))
            func_name = func_args.pop(0)
            # self.get_logger().info(f"Updating function: {func_name} with args {func_args} to value {value}")
            if not self.memory.set_function(func_name, func_args, value):
                self.get_logger().error(f"Error while setting function: '({func_name} "+' '.join(func_args)+f") {value}'")
                return False
        return True

    def set_goals(self, goals):
        self.memory.clear_goals()
        for goal in goals:
            instance_names = goal.split(" ")
            goal_name = instance_names.pop(0)
            # self.get_logger().info(f"Setting goal: {goal_name} with instances {instance_names}")
            if not self.memory.add_goal(goal_name, instance_names):
                return False
        return True
    def clear_goal(self, nothing):
        self.memory.clear_goals()
        self.get_logger().info("Cleared all goals.")
        return True
    def clear_knowledge(self, nothing):
        self.memory.clear_memory()
        self.get_logger().info("Cleared all knowledge.")
        return True
    


    def log_pddl(self, nothing):
        domain_pddl, problem_pddl = self.memory.get_pddl()
        # self.get_logger().info("Current PDDL Domain:")
        # self.get_logger().info(domain_pddl)
        # self.get_logger().info("Current PDDL Problem:")
        # self.get_logger().info(problem_pddl)
        return True
    
    def start_bat_sim(self):
        # return
        delay_time = 10
        runout_time = 2*(78-4-delay_time)

        timer = None
        def delay_callback():
            nonlocal timer
            self.destroy_timer(timer)

            def sim_bat_callback():
                battery_amount = self.memory.get_function_value("battery_amount", [])
                battery_amount -= 100/runout_time
                if battery_amount < 0:
                    battery_amount = 0
                self.memory.set_function("battery_amount", [], battery_amount)

            timer = self.create_timer(1, sim_bat_callback)
        timer = self.create_timer(delay_time, delay_callback)

def main(args=None):
    """
    Main function to initialize the action_planner node and handle spinning.
    """
    rclpy.init(args=args)
    action_planner = ActionPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(action_planner)
    try:
        executor.spin()
    except KeyboardInterrupt:
        action_planner.get_logger().info('KeyboardInterrupt, shutting down.\n')
    action_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()