---
sidebar_position: 5
title: "Practical Exercises: Autonomous Humanoid"
description: "Hands-on exercises to implement complete autonomous humanoid systems"
---

# Practical Exercises: Autonomous Humanoid

## Exercise 1: Complete VLA System Integration

### Objective
Build a complete Vision-Language-Action system that can process voice commands and execute autonomous tasks in simulation.

### Prerequisites
- Completed Modules 1-3
- OpenAI API key
- ROS 2 environment
- Simulation platform (Gazebo, Isaac Sim, or similar)

### Steps

1. **Set up the project structure**
   ```bash
   mkdir ~/vla_capstone_project
   cd ~/vla_capstone_project

   # Create virtual environment
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate

   # Install required packages
   pip install openai rclpy py_trees
   ```

2. **Create the main VLA orchestrator**
   ```python
   # vla_orchestrator.py
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from geometry_msgs.msg import PoseStamped
   import threading
   import queue
   import time

   class VLAOrchestrator(Node):
       def __init__(self):
           super().__init__('vla_orchestrator')

           # Publishers
           self.status_pub = self.create_publisher(String, 'vla_status', 10)
           self.command_pub = self.create_publisher(String, 'robot_command', 10)

           # Subscribers
           self.voice_sub = self.create_subscription(
               String, 'voice_command', self.voice_command_callback, 10
           )
           self.text_sub = self.create_subscription(
               String, 'text_command', self.text_command_callback, 10
           )

           # Internal state
           self.command_queue = queue.Queue()
           self.active = True
           self.robot_pose = [0.0, 0.0, 0.0]

           # Start processing thread
           self.processing_thread = threading.Thread(target=self.process_commands)
           self.processing_thread.start()

           self.get_logger().info('VLA Orchestrator initialized')

       def voice_command_callback(self, msg):
           """Handle voice commands"""
           self.get_logger().info(f'Voice command received: {msg.data}')
           self.process_command({'type': 'voice', 'text': msg.data})

       def text_command_callback(self, msg):
           """Handle text commands"""
           self.get_logger().info(f'Text command received: {msg.data}')
           self.process_command({'type': 'text', 'text': msg.data})

       def process_command(self, command_data):
           """Process a command through the VLA pipeline"""
           try:
               # In a real system, this would:
               # 1. Process natural language (intent classification)
               # 2. Generate task plan (LLM-based planning)
               # 3. Execute plan (ROS 2 behaviors)

               command_text = command_data['text']
               self.get_logger().info(f'Processing: {command_text}')

               # Simulate processing time
               time.sleep(1.0)

               # Determine command type and execute
               if 'go to' in command_text.lower():
                   self.execute_navigation_command(command_text)
               elif 'pick up' in command_text.lower() or 'grasp' in command_text.lower():
                   self.execute_manipulation_command(command_text)
               elif 'find' in command_text.lower() or 'detect' in command_text.lower():
                   self.execute_perception_command(command_text)
               else:
                   self.execute_generic_command(command_text)

               # Publish success status
               status_msg = String()
               status_msg.data = f"command_processed: {command_text}"
               self.status_pub.publish(status_msg)

           except Exception as e:
               self.get_logger().error(f'Command processing error: {e}')
               status_msg = String()
               status_msg.data = f"command_failed: {str(e)}"
               self.status_pub.publish(status_msg)

       def execute_navigation_command(self, command):
           """Execute navigation commands"""
           # Extract destination from command (simplified)
           if 'kitchen' in command.lower():
               destination = [2.0, 1.0, 0.0]
           elif 'living room' in command.lower():
               destination = [0.0, 0.0, 0.0]
           elif 'bedroom' in command.lower():
               destination = [5.0, 3.0, 1.57]
           else:
               destination = [1.0, 1.0, 0.0]  # default

           self.get_logger().info(f'Navigating to {destination}')
           # In real system: send navigation goal to ROS 2 navigation stack

           # Simulate navigation
           time.sleep(2.0)
           self.robot_pose = destination
           self.get_logger().info(f'Navigation completed. New pose: {self.robot_pose}')

       def execute_manipulation_command(self, command):
           """Execute manipulation commands"""
           self.get_logger().info(f'Executing manipulation: {command}')
           # In real system: use manipulation stack to pick/place objects

           # Simulate manipulation
           time.sleep(3.0)
           self.get_logger().info('Manipulation completed')

       def execute_perception_command(self, command):
           """Execute perception commands"""
           self.get_logger().info(f'Executing perception: {command}')
           # In real system: use perception stack to detect objects

           # Simulate perception
           time.sleep(1.5)
           self.get_logger().info('Perception completed')

       def execute_generic_command(self, command):
           """Execute generic commands"""
           self.get_logger().info(f'Executing generic command: {command}')
           # Simulate processing
           time.sleep(1.0)
           self.get_logger().info('Generic command completed')

       def process_commands(self):
           """Process commands from queue (simplified for this example)"""
           while self.active:
               time.sleep(0.1)  # Simulate processing loop

       def shutdown(self):
           """Clean shutdown"""
           self.active = False
           self.processing_thread.join()

   def main():
       rclpy.init()
       orchestrator = VLAOrchestrator()

       try:
           rclpy.spin(orchestrator)
       except KeyboardInterrupt:
           pass
       finally:
           orchestrator.shutdown()
           orchestrator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Create a simple command sender for testing**
   ```python
   # command_sender.py
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   import time

   class CommandSender(Node):
       def __init__(self):
           super().__init__('command_sender')
           self.publisher = self.create_publisher(String, 'text_command', 10)
           self.timer = self.create_timer(5.0, self.send_commands)  # Send every 5 seconds
           self.command_index = 0

           self.test_commands = [
               "Go to the kitchen",
               "Find the red cup",
               "Pick up the cup",
               "Go to the living room",
               "Place the cup on the table",
               "Go back to the charging station"
           ]

           self.get_logger().info('Command sender initialized')

       def send_commands(self):
           """Send test commands"""
           if self.command_index < len(self.test_commands):
               cmd = self.test_commands[self.command_index]
               msg = String()
               msg.data = cmd
               self.publisher.publish(msg)
               self.get_logger().info(f'Sent command: {cmd}')
               self.command_index += 1
           else:
               self.get_logger().info('All commands sent')
               self.timer.cancel()

   def main():
       rclpy.init()
       sender = CommandSender()

       try:
           rclpy.spin(sender)
       except KeyboardInterrupt:
           pass
       finally:
           sender.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. **Run the integrated system**
   ```bash
   # Terminal 1: Start the orchestrator
   python vla_orchestrator.py

   # Terminal 2: Send test commands
   python command_sender.py
   ```

## Exercise 2: Simulation Environment Setup

### Objective
Create a realistic simulation environment with objects for VLA system testing.

### Steps

1. **Create a Gazebo world file**
   ```xml
   <!-- vla_world.world -->
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="vla_world">
       <!-- Physics configuration -->
       <physics name="1ms" type="ode">
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1</real_time_factor>
       </physics>

       <!-- Ground plane -->
       <include>
         <uri>model://ground_plane</uri>
       </include>

       <!-- Lighting -->
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Kitchen area -->
       <model name="kitchen_counter">
         <pose>2 1 0 0 0 0</pose>
         <include>
           <uri>model://box</uri>
           <name>kitchen_counter_base</name>
           <pose>0 0 0.5 0 0 0</pose>
           <scale>2 1 1</scale>
         </include>
       </model>

       <!-- Living room area -->
       <model name="sofa">
         <pose>-2 0 0 0 0 1.57</pose>
         <include>
           <uri>model://box</uri>
           <name>sofa_base</name>
           <pose>0 0 0.3 0 0 0</pose>
           <scale>2 0.8 0.6</scale>
         </include>
       </model>

       <!-- Coffee table -->
       <model name="coffee_table">
         <pose>-1 0 0 0 0 0</pose>
         <include>
           <uri>model://box</uri>
           <name>table_top</name>
           <pose>0 0 0.3 0 0 0</pose>
           <scale>1.2 0.6 0.05</scale>
         </include>
         <model name="table_leg_1">
           <pose>0.5 0.25 -0.15 0 0 0</pose>
           <include>
             <uri>model://cylinder</uri>
             <scale>0.05 0.05 0.5</scale>
           </include>
         </model>
         <model name="table_leg_2">
           <pose>-0.5 0.25 -0.15 0 0 0</pose>
           <include>
             <uri>model://cylinder</uri>
             <scale>0.05 0.05 0.5</scale>
           </include>
         </model>
         <model name="table_leg_3">
           <pose>0.5 -0.25 -0.15 0 0 0</pose>
           <include>
             <uri>model://cylinder</uri>
             <scale>0.05 0.05 0.5</scale>
           </include>
         </model>
         <model name="table_leg_4">
           <pose>-0.5 -0.25 -0.15 0 0 0</pose>
           <include>
             <uri>model://cylinder</uri>
             <scale>0.05 0.05 0.5</scale>
           </include>
         </model>
       </model>

       <!-- Objects for manipulation -->
       <model name="red_cup">
         <pose>2.1 1.1 1.0 0 0 0</pose>
         <include>
           <uri>model://cylinder</uri>
           <scale>0.08 0.08 0.1</scale>
         </include>
       </model>

       <model name="blue_book">
         <pose>-0.9 0.1 0.35 0 0 0</pose>
         <include>
           <uri>model://box</uri>
           <scale>0.2 0.15 0.02</scale>
         </include>
       </model>

       <!-- Robot (example with a simple model) -->
       <model name="simple_robot">
         <pose>0 0 0.5 0 0 0</pose>
         <link name="base_link">
           <pose>0 0 0.5 0 0 0</pose>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.5 0.5 1.0</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.5 0.5 1.0</size>
               </box>
             </geometry>
           </visual>
         </link>
       </model>
     </world>
   </sdf>
   ```

2. **Create a launch file for the simulation**
   ```xml
   <!-- vla_simulation.launch.py -->
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import PathJoinSubstitution
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory

   def generate_launch_description():
       # Launch Gazebo with the custom world
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               get_package_share_directory('gazebo_ros'),
               '/launch/gazebo.launch.py'
           ]),
           launch_arguments={
               'world': PathJoinSubstitution([
                   get_package_share_directory('your_robot_package'),
                   'worlds',
                   'vla_world.world'
               ])
           }.items()
       )

       # Launch your VLA orchestrator
       vla_orchestrator = Node(
           package='vla_package',
           executable='vla_orchestrator',
           name='vla_orchestrator'
       )

       # Launch command sender for testing
       command_sender = Node(
           package='vla_package',
           executable='command_sender',
           name='command_sender'
       )

       return LaunchDescription([
           gazebo,
           vla_orchestrator,
           command_sender
       ])
   ```

3. **Set up the simulation environment**
   ```bash
   # Create the worlds directory
   mkdir -p ~/vla_capstone_project/worlds

   # Copy the world file
   cp vla_world.world ~/vla_capstone_project/worlds/

   # Launch the simulation
   ros2 launch vla_simulation.launch.py
   ```

## Exercise 3: Autonomous Task Execution

### Objective
Implement a complete autonomous task that demonstrates the full VLA pipeline from voice command to task completion.

### Steps

1. **Create an advanced task planner**
   ```python
   # advanced_task_planner.py
   import openai
   import json
   import os
   import time

   class AdvancedTaskPlanner:
       def __init__(self, api_key):
           self.client = openai.OpenAI(api_key=api_key)

       def plan_complex_task(self, goal, context=None):
           """
           Plan a complex task using LLM reasoning
           """
           prompt = f"""
           You are an advanced robotic task planner. Plan a complex task based on the user's goal.

           Goal: "{goal}"

           Context: {context or "No additional context provided"}

           Consider:
           1. Navigation requirements
           2. Object detection needs
           3. Manipulation requirements
           4. Safety constraints
           5. Environmental conditions

           Break down the task into a sequence of executable steps with specific parameters.

           Respond in JSON format:
           {{
               "task_id": "...",
               "goal": "...",
               "plan": [
                   {{
                       "step_id": "...",
                       "description": "...",
                       "action": "...",
                       "parameters": {{
                           "location": "...",
                           "object": "...",
                           "orientation": [x, y, z, w],
                           "gripper_width": 0.0
                       }},
                       "preconditions": ["..."],
                       "expected_outcomes": ["..."],
                       "safety_checks": ["..."]
                   }}
               ],
               "estimated_duration": 0.0,
               "success_criteria": ["..."],
               "failure_modes": ["..."],
               "recovery_strategies": ["..."]
           }}
           """

           try:
               response = self.client.chat.completions.create(
                   model="gpt-4-turbo",
                   messages=[{"role": "user", "content": prompt}],
                   temperature=0.2
               )

               result_text = response.choices[0].message.content.strip()
               json_start = result_text.find('{')
               json_end = result_text.rfind('}') + 1

               if json_start != -1 and json_end != 0:
                   json_str = result_text[json_start:json_end]
                   return json.loads(json_str)
               else:
                   raise ValueError("Could not parse LLM response as JSON")

           except Exception as e:
               print(f"Advanced planning error: {e}")
               return None

       def execute_plan(self, plan, robot_interface):
           """
           Execute a planned sequence of actions
           """
           results = []

           for step in plan['plan']:
               print(f"Executing step: {step['description']}")
               result = self.execute_step(step, robot_interface)
               results.append({
                   'step': step,
                   'result': result,
                   'timestamp': time.time()
               })

               if not result['success']:
                   print(f"Step failed: {result.get('error', 'Unknown error')}")
                   # Try recovery strategies
                   recovery_result = self.attempt_recovery(step, result, robot_interface)
                   if recovery_result['success']:
                       print("Recovery successful, continuing")
                       continue
                   else:
                       print("Recovery failed, aborting task")
                       break

           return {
               'success': all(r['result']['success'] for r in results),
               'results': results,
               'plan_id': plan['task_id']
           }

       def execute_step(self, step, robot_interface):
           """
           Execute a single step of the plan
           """
           action = step['action'].lower()
           params = step['parameters']

           try:
               if action == 'navigate_to':
                   return robot_interface.navigate_to(params.get('location', [0, 0, 0]))
               elif action == 'detect_object':
                   return robot_interface.detect_object(params.get('object', 'unknown'))
               elif action == 'grasp_object':
                   return robot_interface.grasp_object(params.get('object', 'unknown'))
               elif action == 'place_object':
                   return robot_interface.place_object(params.get('location', [0, 0, 0]))
               elif action == 'move_arm':
                   return robot_interface.move_arm(params.get('joint_positions', []))
               else:
                   return {'success': False, 'error': f'Unknown action: {action}'}

           except Exception as e:
               return {'success': False, 'error': str(e)}

       def attempt_recovery(self, failed_step, failure_result, robot_interface):
           """
           Attempt to recover from a failed step
           """
           action = failed_step['action'].lower()
           failure_msg = failure_result.get('error', 'unknown')

           if action == 'navigate_to' and 'obstacle' in failure_msg.lower():
               # Try alternative navigation approach
               return robot_interface.try_alternative_navigation(failed_step['parameters'])
           elif action == 'grasp_object':
               # Try different grasp approach
               return robot_interface.try_different_grasp(failed_step['parameters'])
           else:
               # Default recovery: report failure
               return {'success': False, 'error': 'No recovery strategy available'}

   # Example robot interface (simplified)
   class RobotInterface:
       def __init__(self):
           self.position = [0, 0, 0]
           self.holding_object = None

       def navigate_to(self, location):
           """Simulate navigation to a location"""
           print(f"  Navigating to {location}")
           time.sleep(1.0)  # Simulate navigation time
           self.position = location
           return {'success': True, 'final_position': location}

       def detect_object(self, obj_type):
           """Simulate object detection"""
           print(f"  Detecting {obj_type}")
           time.sleep(0.5)
           # Simulate detection with 80% success rate
           import random
           success = random.random() > 0.2
           return {
               'success': success,
               'object_found': success,
               'object_info': {'type': obj_type, 'position': [1, 1, 1]} if success else None
           }

       def grasp_object(self, obj_type):
           """Simulate object grasping"""
           print(f"  Grasping {obj_type}")
           time.sleep(1.0)
           # Simulate grasp with 85% success rate
           import random
           success = random.random() > 0.15
           if success:
               self.holding_object = obj_type
           return {'success': success, 'object_held': obj_type if success else None}

       def place_object(self, location):
           """Simulate object placement"""
           print(f"  Placing object at {location}")
           time.sleep(0.8)
           self.holding_object = None
           return {'success': True, 'location': location}

       def move_arm(self, joint_positions):
           """Simulate arm movement"""
           print(f"  Moving arm to {joint_positions}")
           time.sleep(0.5)
           return {'success': True, 'final_positions': joint_positions}

       def try_alternative_navigation(self, params):
           """Try alternative navigation path"""
           print("  Trying alternative navigation")
           time.sleep(1.0)
           return {'success': True}

       def try_different_grasp(self, params):
           """Try different grasp approach"""
           print("  Trying different grasp approach")
           time.sleep(1.0)
           return {'success': True}
   ```

2. **Create the complete autonomous execution system**
   ```python
   # autonomous_execution_demo.py
   import os
   import json
   from advanced_task_planner import AdvancedTaskPlanner, RobotInterface

   def run_autonomous_demo():
       """
       Run a complete autonomous execution demo
       """
       # Initialize components
       api_key = os.getenv("OPENAI_API_KEY")
       if not api_key:
           print("Please set OPENAI_API_KEY environment variable")
           return

       planner = AdvancedTaskPlanner(api_key)
       robot = RobotInterface()

       # Define complex tasks to demonstrate
       demo_tasks = [
           {
               "goal": "Go to the kitchen, find the red cup on the counter, pick it up, and bring it to me in the living room",
               "context": {
                   "robot_location": [0, 0, 0],
                   "environment_map": {
                       "kitchen": [2, 1, 0],
                       "living_room": [0, 0, 0]
                   },
                   "objects": ["red_cup", "blue_book"]
               }
           },
           {
               "goal": "Navigate to the bedroom, find my book on the nightstand, and place it on the desk in the office",
               "context": {
                   "robot_location": [0, 0, 0],
                   "environment_map": {
                       "bedroom": [5, 3, 1.57],
                       "office": [-2, 4, -1.57]
                   },
                   "objects": ["book", "desk"]
               }
           }
       ]

       print("Starting Autonomous Execution Demo")
       print("=" * 50)

       for i, task in enumerate(demo_tasks):
           print(f"\nDemo {i+1}: {task['goal']}")
           print("-" * 30)

           # Plan the task
           print("Planning task...")
           plan = planner.plan_complex_task(task['goal'], task['context'])

           if not plan:
               print("Failed to generate plan")
               continue

           print(f"Plan generated with {len(plan['plan'])} steps")
           print(f"Estimated duration: {plan['estimated_duration']:.1f}s")

           # Execute the plan
           print("Executing plan...")
           execution_result = planner.execute_plan(plan, robot)

           # Report results
           print(f"\nExecution Result: {'SUCCESS' if execution_result['success'] else 'FAILED'}")
           print(f"Steps executed: {len(execution_result['results'])}")
           print(f"Robot final position: {robot.position}")
           print(f"Object held: {robot.holding_object}")

           if not execution_result['success']:
               failed_steps = [r for r in execution_result['results'] if not r['result']['success']]
               print(f"Failed steps: {len(failed_steps)}")

           print("\n" + "="*50)

       print("\nAutonomous Execution Demo Complete!")

   if __name__ == "__main__":
       run_autonomous_demo()
   ```

3. **Run the autonomous execution demo**
   ```bash
   # Set your OpenAI API key
   export OPENAI_API_KEY=your_api_key_here

   # Run the demo
   python autonomous_execution_demo.py
   ```

## Exercise 4: Performance Evaluation and Optimization

### Objective
Evaluate the performance of your autonomous system and implement optimizations.

### Steps

1. **Create a performance monitoring system**
   ```python
   # performance_monitor.py
   import time
   import statistics
   from dataclasses import dataclass
   from typing import List, Dict, Any

   @dataclass
   class PerformanceMetrics:
       execution_times: List[float]
       success_rate: float
       average_recovery_time: float
       safety_violations: int
       resource_usage: Dict[str, float]

   class PerformanceMonitor:
       def __init__(self):
           self.execution_records = []
           self.start_time = time.time()

       def record_execution(self, task_description: str, execution_time: float,
                          success: bool, recovery_time: float = 0.0):
           """Record execution performance"""
           record = {
               'task': task_description,
               'execution_time': execution_time,
               'success': success,
               'recovery_time': recovery_time,
               'timestamp': time.time()
           }
           self.execution_records.append(record)

       def calculate_metrics(self) -> PerformanceMetrics:
           """Calculate performance metrics"""
           if not self.execution_records:
               return PerformanceMetrics([], 0.0, 0.0, 0, {})

           execution_times = [r['execution_time'] for r in self.execution_records]
           successful_executions = [r for r in self.execution_records if r['success']]
           recovery_times = [r['recovery_time'] for r in self.execution_records if r['recovery_time'] > 0]

           success_rate = len(successful_executions) / len(self.execution_records)
           avg_recovery_time = statistics.mean(recovery_times) if recovery_times else 0.0

           return PerformanceMetrics(
               execution_times=execution_times,
               success_rate=success_rate,
               average_recovery_time=avg_recovery_time,
               safety_violations=0,  # Would come from safety system
               resource_usage={
                   'cpu_avg': self.get_average_cpu_usage(),
                   'memory_avg': self.get_average_memory_usage()
               }
           )

       def get_average_cpu_usage(self) -> float:
           """Get average CPU usage (simulated)"""
           import random
           return random.uniform(30.0, 70.0)

       def get_average_memory_usage(self) -> float:
           """Get average memory usage (simulated)"""
           import random
           return random.uniform(40.0, 80.0)

       def generate_report(self) -> str:
           """Generate performance report"""
           metrics = self.calculate_metrics()

           report = f"""
   Performance Report
   ==================
   Total Executions: {len(self.execution_records)}
   Success Rate: {metrics.success_rate:.2%}
   Average Execution Time: {statistics.mean(metrics.execution_times):.2f}s
   Average Recovery Time: {metrics.average_recovery_time:.2f}s
   Safety Violations: {metrics.safety_violations}
   Average CPU Usage: {metrics.resource_usage['cpu_avg']:.1f}%
   Average Memory Usage: {metrics.resource_usage['memory_avg']:.1f}%

   Recommendations:
   """
           if metrics.success_rate < 0.8:
               report += "- Success rate is below 80%, investigate failure causes\n"
           if statistics.mean(metrics.execution_times) > 10.0:
               report += "- Average execution time is high, consider optimization\n"

           return report
   ```

2. **Implement system optimization techniques**
   ```python
   # system_optimizer.py
   import threading
   import queue
   from concurrent.futures import ThreadPoolExecutor
   import time

   class SystemOptimizer:
       def __init__(self):
           self.executor = ThreadPoolExecutor(max_workers=4)
           self.cache = {}
           self.optimization_metrics = {
               'cache_hits': 0,
               'cache_misses': 0,
               'parallel_improvement': 0.0
           }

       def optimize_plan_generation(self, goals: List[str], context: Dict[str, Any]):
           """
           Optimize plan generation for multiple goals
           """
           start_time = time.time()

           # Use thread pool for parallel processing
           futures = []
           for goal in goals:
               future = self.executor.submit(self.generate_single_plan, goal, context)
               futures.append(future)

           results = []
           for future in futures:
               result = future.result()
               results.append(result)

           end_time = time.time()
           total_time = end_time - start_time

           # Calculate improvement over sequential processing
           sequential_time = len(goals) * 2.0  # Estimate 2s per plan sequentially
           improvement = (sequential_time - total_time) / sequential_time if sequential_time > 0 else 0

           self.optimization_metrics['parallel_improvement'] = improvement

           return results

       def generate_single_plan(self, goal: str, context: Dict[str, Any]):
           """
           Generate a single plan (simulated)
           """
           # Simulate plan generation
           time.sleep(1.0)  # Simulate processing time
           return {
               'goal': goal,
               'plan': [{'action': 'navigate', 'params': {'x': 1, 'y': 1}}],
               'success': True
           }

       def cache_frequent_operations(self, key: str, operation_func, *args, **kwargs):
           """
           Cache results of frequent operations
           """
           if key in self.cache:
               self.optimization_metrics['cache_hits'] += 1
               return self.cache[key]
           else:
               self.optimization_metrics['cache_misses'] += 1
               result = operation_func(*args, **kwargs)
               self.cache[key] = result
               # Limit cache size
               if len(self.cache) > 100:
                   # Remove oldest entries
                   oldest_key = next(iter(self.cache))
                   del self.cache[oldest_key]
               return result

       def optimize_resource_usage(self):
           """
           Optimize resource usage based on current load
           """
           # Adjust processing parameters based on system load
           import psutil
           cpu_percent = psutil.cpu_percent()
           memory_percent = psutil.virtual_memory().percent

           if cpu_percent > 80:
               # Reduce parallelism under high CPU load
               self.executor._max_workers = max(1, self.executor._max_workers // 2)
           elif cpu_percent < 30:
               # Increase parallelism under low CPU load
               self.executor._max_workers = min(8, self.executor._max_workers * 2)

           if memory_percent > 85:
               # Clear cache under high memory usage
               self.cache.clear()
   ```

3. **Run the complete evaluation**
   ```python
   # evaluation_demo.py
   import os
   from performance_monitor import PerformanceMonitor
   from system_optimizer import SystemOptimizer
   from advanced_task_planner import AdvancedTaskPlanner, RobotInterface

   def run_evaluation():
       """
       Run complete system evaluation
       """
       api_key = os.getenv("OPENAI_API_KEY")
       if not api_key:
           print("Please set OPENAI_API_KEY environment variable")
           return

       # Initialize components
       monitor = PerformanceMonitor()
       optimizer = SystemOptimizer()
       planner = AdvancedTaskPlanner(api_key)
       robot = RobotInterface()

       # Test tasks for evaluation
       test_tasks = [
           "Go to kitchen and return",
           "Find and pick up cup",
           "Navigate to multiple locations",
           "Complex manipulation task"
       ]

       print("Starting System Evaluation")
       print("=" * 40)

       for i, task_goal in enumerate(test_tasks):
           print(f"\nTest {i+1}: {task_goal}")

           start_time = time.time()

           # Plan the task
           plan = planner.plan_complex_task(task_goal, {})

           if plan:
               # Execute the plan
               result = planner.execute_plan(plan, robot)
               execution_time = time.time() - start_time

               # Record performance
               success = result['success']
               monitor.record_execution(task_goal, execution_time, success)

               print(f"  Execution time: {execution_time:.2f}s")
               print(f"  Success: {'Yes' if success else 'No'}")
           else:
               print("  Failed to plan task")

       # Generate performance report
       print("\n" + monitor.generate_report())

       # Test optimization
       print("\nTesting System Optimization...")
       optimizer.optimize_plan_generation(test_tasks, {})
       print(f"Parallel improvement: {optimizer.optimization_metrics['parallel_improvement']:.2%}")

   if __name__ == "__main__":
       run_evaluation()
   ```

## Solutions and Hints

### Exercise 1 Solution Highlights
- The VLA orchestrator handles voice and text commands
- Command processing simulates the full VLA pipeline
- Navigation, manipulation, and perception components are integrated
- Status reporting provides feedback on system state

### Exercise 2 Solution Highlights
- Custom Gazebo world with kitchen and living room areas
- Objects placed for manipulation tasks
- Robot model with appropriate collision and visual properties
- Launch file to start the complete simulation

### Exercise 3 Solution Highlights
- Advanced task planning with LLM-based reasoning
- Step-by-step execution with error handling
- Recovery strategies for failed actions
- Complete autonomous task execution from voice command to completion

### Exercise 4 Solution Highlights
- Performance monitoring with metrics collection
- System optimization techniques (caching, parallelization)
- Resource usage optimization
- Comprehensive evaluation and reporting

## Assessment Questions

1. How does the VLA orchestrator coordinate between vision, language, and action components?
2. What safety measures should be implemented in autonomous humanoid systems?
3. How can you optimize the performance of complex autonomous tasks?
4. What are the key challenges in integrating all VLA components?

## Extension Ideas

1. Add computer vision for real-time object detection
2. Implement multi-modal interaction (voice + gesture)
3. Create a web interface for remote operation
4. Add learning capabilities to improve performance over time
5. Implement collaborative tasks with multiple robots

## Summary

These exercises provide hands-on experience with:
- Complete VLA system integration
- Simulation environment setup
- Autonomous task execution
- Performance evaluation and optimization
- Real-world deployment considerations

Complete these exercises to gain practical experience with full autonomous humanoid systems.