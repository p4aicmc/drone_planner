from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from enum import Enum
from harpia_msgs.action import ActionCaller
from action_msgs.msg import GoalStatus

class ActionState(Enum):
    PENDING = 1
    IN_PROGRESS = 2
    COMPLETED = 3

class ActionPlannerExecutor:
    """
    Executor responsible for managing the lifecycle state transitions of a ROS 2 node.

    Parameters
    ----------
    node : rclpy.node.Node
        Parent node responsible for creating clients and logging.
    node_name : str
        Name of the target node whose lifecycle state will be managed.
    memory : object
        Singleton memory instance shared between components.

    Attributes
    ----------
    _parent_node : rclpy.node.Node
        Reference to the parent node.
    node_name : str
        Target node name for lifecycle management.
    _memory_singleton : object
        Shared memory instance.
    _action_node_client : rclpy.client.Client
        Client to call the ChangeState service for lifecycle transitions.
    _transition_label_to_id : dict
        Dictionary mapping transition labels to their respective transition IDs.
    """

    def __init__(self, node, memory):
        self._parent_node = node
        # self.node_name = node_name
        self.memory = memory
        self.get_logger().info("ActionPlannerExecutor initialized")

        self.current_plan = None

        # # Create a client to call the ChangeState service for the target node
        # self._action_node_client = self._parent_node.create_client(
        #     ChangeState,
        #     f'/{node_name}/change_state'
        # )

        # # Mapping between human-readable transition labels and their corresponding IDs
        # self._transition_label_to_id = {
        #     'configure': Transition.TRANSITION_CONFIGURE,
        #     'cleanup': Transition.TRANSITION_CLEANUP,
        #     'activate': Transition.TRANSITION_ACTIVATE,
        #     'deactivate': Transition.TRANSITION_DEACTIVATE,
        # }
        

    # def call_change_state(self, transition, callback):
    #     """
    #     Request a lifecycle state transition for the managed node.

    #     Parameters
    #     ----------
    #     transition : str
    #         The label of the desired transition. Must be a valid key in `_transition_label_to_id`.
    #         Examples: 'configure', 'activate', 'deactivate', 'cleanup'.
    #     callback : callable
    #         Callback function to be executed when the service call is completed.

    #     Returns
    #     -------
    #     None
    #     """
    #     if not self._action_node_client.service_is_ready():
    #         self._parent_node.get_logger().error(
    #             f'Could not change state of node {self.node_name}. Service not available.'
    #         )
    #         return

    #     # Create and populate the ChangeState request
    #     request = ChangeState.Request()
    #     request.transition.id = self._transition_label_to_id[transition]
    #     request.transition.label = transition

    #     # Send the request asynchronously and register the callback
    #     future = self._action_node_client.call_async(request)
    #     future.add_done_callback(callback)

    def get_logger(self):
        # if self._parent_node.get_logger() is None:
        #     raise ValueError("Logger not initialized, use the function .init()")
        return self._parent_node.get_logger()
    
    def print_action(self, action):
        self.get_logger().info(f"action: {action['name']} {action['args']} ({action['state']})")

    def is_executing(self):
        return self.current_plan is not None
    
    def execute_plan(self, plan, check_if_plan_is_cancelling, on_success=None, on_failure=None, on_finish_cancel=None):
        self.on_plan_success = on_success
        self.on_plan_failure = on_failure
        self.on_finish_cancel = on_finish_cancel
        self.check_if_plan_is_cancelling = check_if_plan_is_cancelling
        self.get_logger().info("execute plan:\n"+'\n'.join([ f"{action[0]} {' '.join(action[1])}" for action in plan]))

        if self.current_plan is not None:
            self.get_logger().warn("plan already in progress")
            return
        
        self.current_plan = []

        for action in plan:
            if self.memory._get_action_by_name(action[0]) is None:
                self.get_logger().error(f"action {action[0]} from plan not found on domain")
                return
            
            new_action = {}
            new_action['name'] = action[0]
            new_action['args'] = action[1]
            new_action['state'] = ActionState.PENDING

            self.current_plan.append(new_action)

        self.check_actions_condition_timer = self._parent_node.create_timer(1, self.check_in_progress_action_conditions)
        # [self.print_action(action) for action in self.current_plan]
        self.check_if_can_start_action()
        # [self.print_action(action) for action in self.current_plan]


    def check_if_can_start_action(self):
        # self.get_logger().info("checking if can start action")
        # self.get_logger().info("actions before:")
        # [self.print_action(action) for action in self.current_plan]

        if self.current_plan is None:
            self.get_logger().error("no plan in progress 1")
            return False
        
        is_cancelling = self.check_if_plan_is_cancelling()
        started_any_action = False

        all_completed = True
        is_there_in_progress = False

        
        for action in self.current_plan:

            # self.get_logger().info(f"checking action {action['name']} {action['args']} ({action['state']})")

            if action['state'] == ActionState.PENDING:
                all_completed = False

                if is_cancelling:
                    # self.get_logger().info("plan is cancelling, skipping action start")
                    continue

                if self.memory.check_conditions_action(action['name'], action['args'], ["at start", "over all"]):
                    # self.get_logger().info(f"action {action['name']} can be started")
                    if not self.start_action(action):
                        self.get_logger().error(f"could not start action {action['name']}")
                        return False
                    started_any_action = True
                    break
                else:
                    # self.get_logger().info(f"action {action['name']} cannot be started, conditions not met")
                    # if the action cannot be started, the plan is failed
                    self.handle_plan_failure()
                    return False
            elif action['state'] == ActionState.IN_PROGRESS:
                all_completed = False
                is_there_in_progress = True
            elif action['state'] == ActionState.COMPLETED:
                # is_there_completed = True
                pass

        # self.get_logger().info("actions after:")
        # [self.print_action(action) for action in self.current_plan]

        if is_cancelling:
            self.get_logger().info("plan is cancelling...")
            if not is_there_in_progress:
                self.handle_plan_canceled()

                return False
            else:
                self.get_logger().info("but still there are actions in progress")

        if not started_any_action:
            # if entered here, could not continue with the plan

            if is_there_in_progress:
                self.get_logger().info("there is an action in progress, waiting for it to finish")
                
            elif all_completed:
                self.get_logger().info("all actions completed, plan succeeded")
                self.handle_plan_success()
            else:
                self.get_logger().info("no actions can be started, plan failed")
                self.handle_plan_failure()


        return started_any_action
            
    def start_action(self, action):        
        action['state'] = ActionState.IN_PROGRESS
        self.memory.apply_effects(action['name'], action['args'], ["at start"])
        # self.get_logger().info(f"action {action['name']} started")

        # call action 
        goal_msg = ActionCaller.Goal(parameters=action['args'])
        # self.get_logger().info(f"Sending goal: {action['name']} {goal_msg.parameters}")
        

        def result_callback(future):

            result = future.result().result
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                # Action completed (was not canceled)

                if result.success:
                    self.get_logger().info(f"action {action['name']} completed")    
                    self.memory.apply_effects(action['name'], action['args'], ["at end"])
                    action['state'] = ActionState.COMPLETED
                    self.check_if_can_start_action()
                
                else:
                    self.get_logger().error(f'Action finished, but failed')
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Goal was canceled by client')
            else:
                self.get_logger().error(f'Goal ended with status code: {status}')


        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected by server')
                return

            _goal_handle = goal_handle

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_callback)

        def feedback_callback(fb_msg):
            # self.get_logger().info(f"Feedback: {fb_msg.feedback.status:.2f}")
            pass

        action_client = ActionClient(
            self._parent_node,
            ActionCaller,
            '/action/'+action['name'],
        )
        if not action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available')
            # since canot start the action, the plan is failed
            self.handle_plan_failure()
            return False

        future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback,
        )
        future.add_done_callback(goal_response_callback)

        return True

    def check_in_progress_action_conditions(self):
        # self.get_logger().info("checking in progress action conditions")
        if self.current_plan is None:
            self.get_logger().error("no plan in progress 2")
            return
        
        for action in self.current_plan:
            if action['state'] == ActionState.IN_PROGRESS:
                if not self.memory.check_conditions_action(action['name'], action['args'], "over all"):
                    self.get_logger().error(f"action {action['name']} do not meet over all conditions")
                    self.handle_plan_failure()

    def handle_plan_failure(self):
        self.get_logger().error("Plan failed")
        if self.on_plan_failure is not None:
            self.on_plan_failure()
        self.current_plan = None

        self._parent_node.destroy_timer(self.check_actions_condition_timer)

    def handle_plan_success(self):
        self.get_logger().info("Plan succeeded")
        if self.on_plan_success is not None:
            self.on_plan_success()
        self.current_plan = None

        self._parent_node.destroy_timer(self.check_actions_condition_timer)

    def handle_plan_canceled(self):
        self.get_logger().info("Plan cancelled")
        if self.on_finish_cancel is not None:
            self.on_finish_cancel()
        self.current_plan = None

        self._parent_node.destroy_timer(self.check_actions_condition_timer)