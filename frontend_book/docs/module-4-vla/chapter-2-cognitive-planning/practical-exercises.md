---
sidebar_position: 5
title: "Practical Exercises: Cognitive Planning"
description: "Hands-on exercises to implement LLM-based cognitive planning for robotic systems"
---

# Practical Exercises: Cognitive Planning

## Exercise 1: Basic LLM Plan Generation

### Objective
Create a system that uses an LLM to generate task plans from natural language goals.

### Prerequisites
- OpenAI API key
- Python development environment
- Basic understanding of JSON parsing

### Steps

1. **Set up the environment**
   ```bash
   # Create a new directory for the exercise
   mkdir ~/vla_exercises/cognitive_planning
   cd ~/vla_exercises/cognitive_planning

   # Create virtual environment
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate

   # Install required packages
   pip install openai
   ```

2. **Create the basic plan generator**
   ```python
   # plan_generator.py
   import openai
   import os
   import json
   import time

   class BasicPlanGenerator:
       def __init__(self, api_key):
           self.client = openai.OpenAI(api_key=api_key)

       def generate_plan(self, goal, context=None):
           """
           Generate a plan from a natural language goal
           """
           prompt = f"""
           Generate a detailed plan to accomplish the following goal:
           "{goal}"

           Context: {context or "No additional context provided"}

           The plan should include:
           1. A sequence of steps to accomplish the goal
           2. Required actions for each step
           3. Expected outcomes for each step
           4. Potential challenges and how to address them

           Format the response as JSON with this structure:
           {{
               "goal": "...",
               "plan": [
                   {{
                       "step_number": 1,
                       "description": "...",
                       "action": "...",
                       "parameters": {{}},
                       "expected_outcome": "...",
                       "potential_challenges": ["..."]
                   }}
               ],
               "estimated_duration": 0.0,
               "success_criteria": ["..."]
           }}
           """

           try:
               response = self.client.chat.completions.create(
                   model="gpt-3.5-turbo",
                   messages=[{"role": "user", "content": prompt}],
                   temperature=0.3
               )

               result_text = response.choices[0].message.content.strip()

               # Extract JSON from response
               json_start = result_text.find('{')
               json_end = result_text.rfind('}') + 1

               if json_start != -1 and json_end != 0:
                   json_str = result_text[json_start:json_end]
                   return json.loads(json_str)
               else:
                   raise ValueError("Could not parse LLM response as JSON")

           except Exception as e:
               print(f"Plan generation error: {e}")
               return None

   # Example usage
   if __name__ == "__main__":
       api_key = os.getenv("OPENAI_API_KEY")
       if not api_key:
           print("Please set OPENAI_API_KEY environment variable")
           exit(1)

       generator = BasicPlanGenerator(api_key)

       # Test with a simple goal
       goal = "Get a glass of water from the kitchen and bring it to me"
       plan = generator.generate_plan(goal)

       if plan:
           print("Generated Plan:")
           print(json.dumps(plan, indent=2))
       else:
           print("Failed to generate plan")
   ```

3. **Create a plan validator**
   ```python
   # plan_validator.py
   class PlanValidator:
       def __init__(self):
           self.supported_actions = {
               "navigate_to", "go_to", "move_to", "pick_up", "grasp",
               "place", "find", "detect_object", "wait", "report_status"
           }

       def validate_plan(self, plan):
           """
           Validate that the plan contains supported actions and proper structure
           """
           if not plan or "plan" not in plan:
               return False, "Plan must contain 'plan' array"

           errors = []
           warnings = []

           for i, step in enumerate(plan["plan"]):
               # Check required fields
               required_fields = ["step_number", "description", "action"]
               for field in required_fields:
                   if field not in step:
                       errors.append(f"Step {i+1} missing required field: {field}")

               # Check if action is supported
               action = step.get("action", "").lower()
               if action not in self.supported_actions:
                   warnings.append(f"Step {i+1}: Unknown action '{action}' may not be supported")

               # Check parameters if present
               params = step.get("parameters", {})
               if not isinstance(params, dict):
                   errors.append(f"Step {i+1} parameters must be a dictionary")

           return len(errors) == 0, {"errors": errors, "warnings": warnings}
   ```

4. **Create a plan executor simulator**
   ```python
   # plan_executor.py
   import time
   import random

   class PlanExecutor:
       def __init__(self):
           self.robot_state = {
               "location": [0.0, 0.0, 0.0],
               "holding_object": None,
               "battery_level": 1.0
           }

       def execute_plan(self, plan):
           """
           Execute a plan step by step (simulated)
           """
           print(f"Starting execution of plan: {plan.get('goal', 'Unknown goal')}")
           results = []

           for step in plan.get("plan", []):
               print(f"\nExecuting step: {step['description']}")
               result = self.execute_step(step)

               results.append({
                   "step": step,
                   "result": result,
                   "timestamp": time.time()
               })

               if not result["success"]:
                   print(f"Step failed: {result.get('error', 'Unknown error')}")
                   return {"success": False, "results": results}

               print(f"Step completed successfully")

           return {"success": True, "results": results}

       def execute_step(self, step):
           """
           Execute a single step (simulated)
           """
           action = step.get("action", "").lower()
           params = step.get("parameters", {})

           # Simulate different actions
           if action in ["navigate_to", "go_to", "move_to"]:
               return self.simulate_navigation(params)
           elif action in ["pick_up", "grasp"]:
               return self.simulate_grasp(params)
           elif action == "place":
               return self.simulate_place(params)
           elif action in ["find", "detect_object"]:
               return self.simulate_detection(params)
           elif action == "wait":
               return self.simulate_wait(params)
           else:
               return {"success": False, "error": f"Unknown action: {action}"}

       def simulate_navigation(self, params):
           """Simulate navigation action"""
           target = params.get("target", [0, 0, 0])
           print(f"  Navigating to {target}")

           # Simulate navigation time
           time.sleep(0.5)

           # Simulate occasional failures
           if random.random() < 0.1:  # 10% failure rate
               return {"success": False, "error": "Navigation failed - path blocked"}

           # Update robot state
           self.robot_state["location"] = target
           return {"success": True, "message": f"Navigated to {target}"}

       def simulate_grasp(self, params):
           """Simulate grasping action"""
           obj = params.get("object", "unknown")
           print(f"  Grasping {obj}")

           time.sleep(0.7)

           if random.random() < 0.15:  # 15% failure rate
               return {"success": False, "error": f"Failed to grasp {obj} - object too heavy"}

           self.robot_state["holding_object"] = obj
           return {"success": True, "message": f"Successfully grasped {obj}"}

       def simulate_place(self, params):
           """Simulate placing action"""
           location = params.get("location", "unknown")
           obj = self.robot_state["holding_object"]

           if not obj:
               return {"success": False, "error": "Not holding any object to place"}

           print(f"  Placing {obj} at {location}")

           time.sleep(0.6)

           if random.random() < 0.05:  # 5% failure rate
               return {"success": False, "error": f"Failed to place {obj} - dropped"}

           self.robot_state["holding_object"] = None
           return {"success": True, "message": f"Successfully placed {obj} at {location}"}

       def simulate_detection(self, params):
           """Simulate object detection"""
           obj_type = params.get("object_type", "unknown")
           print(f"  Detecting {obj_type}")

           time.sleep(0.4)

           if random.random() < 0.2:  # 20% failure rate
               return {"success": False, "error": f"Could not find {obj_type} in view"}

           return {"success": True, "message": f"Detected {obj_type}", "detected_object": obj_type}

       def simulate_wait(self, params):
           """Simulate waiting action"""
           duration = params.get("duration", 1.0)
           print(f"  Waiting for {duration} seconds")

           time.sleep(duration)
           return {"success": True, "message": f"Waited for {duration} seconds"}
   ```

5. **Combine all components into a complete system**
   ```python
   # main_planning_system.py
   import os
   import json
   from plan_generator import BasicPlanGenerator
   from plan_validator import PlanValidator
   from plan_executor import PlanExecutor

   class CognitivePlanningSystem:
       def __init__(self):
           api_key = os.getenv("OPENAI_API_KEY")
           if not api_key:
               raise ValueError("OPENAI_API_KEY environment variable not set")

           self.generator = BasicPlanGenerator(api_key)
           self.validator = PlanValidator()
           self.executor = PlanExecutor()

       def process_goal(self, goal, context=None):
           """
           Complete pipeline: goal -> plan -> validation -> execution
           """
           print(f"Processing goal: {goal}")
           print("-" * 50)

           # Step 1: Generate plan
           print("1. Generating plan...")
           plan = self.generator.generate_plan(goal, context)
           if not plan:
               return {"success": False, "error": "Failed to generate plan"}

           print(f"   Plan generated with {len(plan.get('plan', []))} steps")

           # Step 2: Validate plan
           print("2. Validating plan...")
           is_valid, validation_result = self.validator.validate_plan(plan)

           if not is_valid:
               print(f"   Validation failed: {validation_result['errors']}")
               return {"success": False, "error": f"Plan validation failed: {validation_result['errors']}"}

           if validation_result["warnings"]:
               print(f"   Validation warnings: {validation_result['warnings']}")

           print("   Plan validated successfully")

           # Step 3: Execute plan
           print("3. Executing plan...")
           execution_result = self.executor.execute_plan(plan)

           return {
               "success": execution_result["success"],
               "plan": plan,
               "execution_results": execution_result["results"],
               "robot_state": self.executor.robot_state
           }

   def main():
       # Check for API key
       if not os.getenv("OPENAI_API_KEY"):
           print("Please set OPENAI_API_KEY environment variable")
           return

       # Create the system
       system = CognitivePlanningSystem()

       # Test goals
       test_goals = [
           "Go to the kitchen and bring me a glass of water",
           "Find the red book on the shelf and place it on the table",
           "Go to the office, pick up the pen, and bring it to the living room"
       ]

       for i, goal in enumerate(test_goals):
           print(f"\n{'='*60}")
           print(f"TEST {i+1}: {goal}")
           print(f"{'='*60}")

           result = system.process_goal(goal)

           if result["success"]:
               print(f"\n✓ SUCCESS: Goal completed!")
               print(f"Final robot state: {result['robot_state']}")
           else:
               print(f"\n✗ FAILED: {result.get('error', 'Unknown error')}")

   if __name__ == "__main__":
       main()
   ```

6. **Run the exercise**
   ```bash
   # Set your OpenAI API key
   export OPENAI_API_KEY=your_api_key_here

   # Run the cognitive planning system
   python main_planning_system.py
   ```

## Exercise 2: Context-Aware Planning

### Objective
Enhance the planning system to consider environmental context when generating plans.

### Steps

1. **Create a context manager**
   ```python
   # context_manager.py
   class ContextManager:
       def __init__(self):
           self.environment = {
               "rooms": {
                   "kitchen": {"coordinates": [2.0, 1.0, 0.0], "objects": ["fridge", "counter", "sink"]},
                   "living_room": {"coordinates": [0.0, 0.0, 0.0], "objects": ["sofa", "table", "tv"]},
                   "bedroom": {"coordinates": [5.0, 3.0, 1.57], "objects": ["bed", "dresser", "nightstand"]},
                   "office": {"coordinates": [-2.0, 4.0, -1.57], "objects": ["desk", "chair", "bookshelf"]}
               },
               "objects": {
                   "glass": {"location": "kitchen", "on": "counter"},
                   "red_book": {"location": "office", "on": "bookshelf"},
                   "pen": {"location": "office", "on": "desk"},
                   "water": {"location": "kitchen", "in": "fridge"}
               },
               "robot": {
                   "current_location": [0.0, 0.0, 0.0],
                   "holding": None
               }
           }

       def get_context_for_goal(self, goal):
           """
           Extract relevant context for a specific goal
           """
           context = {
               "environment_map": self.environment["rooms"],
               "available_objects": list(self.environment["objects"].keys()),
               "robot_state": self.environment["robot"],
               "spatial_relationships": self.get_spatial_relationships(),
               "object_locations": self.get_object_locations()
           }

           # Add goal-specific context
           if "water" in goal.lower():
               context["water_location"] = self.environment["objects"]["water"]
           if "book" in goal.lower():
               context["book_location"] = self.environment["objects"]["red_book"]

           return context

       def get_spatial_relationships(self):
           """
           Get spatial relationships between locations
           """
           return {
               "kitchen_to_living_room": {"distance": 2.2, "path_available": True},
               "office_to_living_room": {"distance": 5.4, "path_available": True},
               "kitchen_to_office": {"distance": 4.5, "path_available": True}
           }

       def get_object_locations(self):
           """
           Get locations of specific objects
           """
           locations = {}
           for obj_name, obj_info in self.environment["objects"].items():
               room = obj_info["location"]
               room_coords = self.environment["rooms"][room]["coordinates"]
               locations[obj_name] = {
                   "room": room,
                   "coordinates": room_coords
               }
           return locations

       def update_robot_location(self, new_location):
           """
           Update robot's location in the environment model
           """
           self.environment["robot"]["current_location"] = new_location

       def update_robot_holding(self, object_name):
           """
           Update what the robot is holding
           """
           self.environment["robot"]["holding"] = object_name
   ```

2. **Enhance the plan generator with context**
   ```python
   # enhanced_plan_generator.py
   import openai
   import os
   import json

   class ContextAwarePlanGenerator:
       def __init__(self, api_key):
           self.client = openai.OpenAI(api_key=api_key)

       def generate_contextual_plan(self, goal, context):
           """
           Generate a plan considering environmental context
           """
           prompt = f"""
           Generate a detailed plan to accomplish the following goal:
           "{goal}"

           Environmental Context:
           {json.dumps(context, indent=2)}

           Consider:
           1. Current robot location and what it's holding
           2. Locations of relevant objects
           3. Spatial relationships between locations
           4. Available paths and distances
           5. Environmental constraints

           The plan should be efficient and consider the current state.
           Format the response as JSON with this structure:
           {{
               "goal": "...",
               "context_considered": true,
               "plan": [
                   {{
                       "step_number": 1,
                       "description": "...",
                       "action": "...",
                       "parameters": {{
                           "target_location": "...",
                           "object": "...",
                           "coordinates": [x, y, theta]
                       }},
                       "expected_outcome": "...",
                       "efficiency_consideration": "..."
                   }}
               ],
               "estimated_duration": 0.0,
               "success_criteria": ["..."],
               "context_summary": "Brief summary of how context was used"
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
               print(f"Contextual plan generation error: {e}")
               return None
   ```

3. **Test the context-aware system**
   ```python
   # test_context_aware.py
   import os
   from enhanced_plan_generator import ContextAwarePlanGenerator
   from context_manager import ContextManager

   def test_context_aware_planning():
       if not os.getenv("OPENAI_API_KEY"):
           print("Please set OPENAI_API_KEY environment variable")
           return

       # Initialize components
       generator = ContextAwarePlanGenerator(os.getenv("OPENAI_API_KEY"))
       context_manager = ContextManager()

       # Test goals that benefit from context
       test_goals = [
           "Get the red book from the office",
           "Bring me a glass of water from the kitchen",
           "Go to the bedroom and wait there"
       ]

       for goal in test_goals:
           print(f"\n{'='*60}")
           print(f"GOAL: {goal}")
           print(f"{'='*60}")

           # Get context for the goal
           context = context_manager.get_context_for_goal(goal)
           print(f"Context used:")
           print(f"  - Available objects: {len(context['available_objects'])}")
           print(f"  - Current robot location: {context['robot_state']['current_location']}")
           print(f"  - Robot holding: {context['robot_state']['holding']}")

           # Generate contextual plan
           plan = generator.generate_contextual_plan(goal, context)

           if plan:
               print(f"\nGenerated Plan:")
               print(f"  Steps: {len(plan['plan'])}")
               print(f"  Context used: {plan.get('context_considered', False)}")
               print(f"  Summary: {plan.get('context_summary', 'No summary provided')}")

               # Show first few steps
               for step in plan['plan'][:3]:  # Show first 3 steps
                   print(f"    Step {step['step_number']}: {step['action']} - {step['description']}")
           else:
               print("Failed to generate plan")

   if __name__ == "__main__":
       test_context_aware_planning()
   ```

## Exercise 3: Behavior Mapping and Execution

### Objective
Create a system that maps plan steps to executable behaviors and handles execution.

### Steps

1. **Create behavior classes**
   ```python
   # behavior_classes.py
   import time
   import random

   class RobotBehavior:
       def __init__(self, name):
           self.name = name
           self.status = "idle"

       def execute(self, parameters):
           """
           Execute the behavior with given parameters
           Must be implemented by subclasses
           """
           raise NotImplementedError

       def get_status(self):
           return self.status

   class NavigationBehavior(RobotBehavior):
       def __init__(self):
           super().__init__("navigation")

       def execute(self, parameters):
           target = parameters.get("target", parameters.get("coordinates", [0, 0, 0]))
           print(f"  Navigating to {target}")

           # Simulate navigation
           time.sleep(0.8)

           # Simulate occasional failures
           if random.random() < 0.08:
               return {"success": False, "error": "Navigation failed - obstacle detected"}

           return {"success": True, "message": f"Navigated to {target}", "final_position": target}

   class ManipulationBehavior(RobotBehavior):
       def __init__(self):
           super().__init__("manipulation")

       def execute(self, parameters):
           action = parameters.get("action", "grasp")
           obj = parameters.get("object", "unknown")

           if action == "grasp":
               print(f"  Grasping {obj}")
               time.sleep(0.7)
               success = random.random() > 0.12  # 88% success rate
               if success:
                   return {"success": True, "message": f"Grasped {obj}", "object_held": obj}
               else:
                   return {"success": False, "error": f"Failed to grasp {obj}"}
           elif action == "place":
               print(f"  Placing {obj}")
               time.sleep(0.6)
               success = random.random() > 0.05  # 95% success rate
               if success:
                   return {"success": True, "message": f"Placed {obj}", "object_held": None}
               else:
                   return {"success": False, "error": f"Failed to place {obj} - dropped"}
           else:
               return {"success": False, "error": f"Unknown manipulation action: {action}"}

   class PerceptionBehavior(RobotBehavior):
       def __init__(self):
           super().__init__("perception")

       def execute(self, parameters):
           obj_type = parameters.get("object_type", "unknown")
           location = parameters.get("location", "current_view")

           print(f"  Detecting {obj_type} in {location}")
           time.sleep(0.5)

           # Simulate detection with some failures
           if random.random() < 0.18:  # 18% failure rate
               return {"success": False, "error": f"Could not detect {obj_type} in {location}"}

           return {
               "success": True,
               "message": f"Detected {obj_type}",
               "object_found": True,
               "object_info": {"type": obj_type, "location": location}
           }

   class WaitBehavior(RobotBehavior):
       def __init__(self):
           super().__init__("wait")

       def execute(self, parameters):
           duration = parameters.get("duration", 1.0)
           print(f"  Waiting for {duration} seconds")
           time.sleep(duration)
           return {"success": True, "message": f"Waited for {duration} seconds"}
   ```

2. **Create a behavior executor**
   ```python
   # behavior_executor.py
   from behavior_classes import NavigationBehavior, ManipulationBehavior, PerceptionBehavior, WaitBehavior

   class BehaviorExecutor:
       def __init__(self):
           self.behaviors = {
               "navigate_to": NavigationBehavior(),
               "go_to": NavigationBehavior(),
               "move_to": NavigationBehavior(),
               "pick_up": ManipulationBehavior(),
               "grasp": ManipulationBehavior(),
               "place": ManipulationBehavior(),
               "find": PerceptionBehavior(),
               "detect_object": PerceptionBehavior(),
               "wait": WaitBehavior()
           }

           self.robot_state = {
               "location": [0.0, 0.0, 0.0],
               "holding": None,
               "battery": 100.0
           }

       def execute_plan_steps(self, plan_steps):
           """
           Execute a sequence of plan steps as behaviors
           """
           results = []

           for step in plan_steps:
               action = step.get("action", "").lower()
               parameters = step.get("parameters", {})

               print(f"\nExecuting: {action} - {step['description']}")

               if action in self.behaviors:
                   result = self.execute_behavior(action, parameters)
                   results.append({
                       "step": step,
                       "result": result,
                       "timestamp": time.time()
                   })

                   # Update robot state based on result
                   self.update_robot_state(result)

                   if not result["success"]:
                       print(f"  ❌ Failed: {result.get('error', 'Unknown error')}")
                       return {"success": False, "results": results}
                   else:
                       print(f"  ✅ Success: {result.get('message', 'Completed')}")
               else:
                   result = {"success": False, "error": f"Unknown behavior: {action}"}
                   results.append({
                       "step": step,
                       "result": result,
                       "timestamp": time.time()
                   })
                   return {"success": False, "results": results}

           return {"success": True, "results": results}

       def execute_behavior(self, action, parameters):
           """
           Execute a specific behavior
           """
           behavior = self.behaviors[action]
           return behavior.execute(parameters)

       def update_robot_state(self, result):
           """
           Update robot state based on behavior result
           """
           if result.get("final_position"):
               self.robot_state["location"] = result["final_position"]

           if "object_held" in result:
               self.robot_state["holding"] = result.get("object_held")

           # Reduce battery for each action
           self.robot_state["battery"] = max(0.0, self.robot_state["battery"] - 2.0)

       def get_robot_state(self):
           return self.robot_state.copy()
   ```

3. **Create the complete cognitive planning system**
   ```python
   # complete_system.py
   import os
   import json
   from enhanced_plan_generator import ContextAwarePlanGenerator
   from context_manager import ContextManager
   from behavior_executor import BehaviorExecutor

   class CompleteCognitivePlanningSystem:
       def __init__(self):
           api_key = os.getenv("OPENAI_API_KEY")
           if not api_key:
               raise ValueError("OPENAI_API_KEY environment variable not set")

           self.generator = ContextAwarePlanGenerator(api_key)
           self.context_manager = ContextManager()
           self.executor = BehaviorExecutor()

       def execute_goal(self, goal):
           """
           Complete pipeline: goal -> contextual plan -> behavior execution
           """
           print(f"Processing goal: {goal}")
           print("="*60)

           # Step 1: Get context
           print("1. Gathering environmental context...")
           context = self.context_manager.get_context_for_goal(goal)
           print(f"   Context gathered for {len(context['available_objects'])} objects")

           # Step 2: Generate contextual plan
           print("2. Generating contextual plan...")
           plan = self.generator.generate_contextual_plan(goal, context)

           if not plan:
               return {"success": False, "error": "Failed to generate plan"}

           print(f"   Plan generated with {len(plan['plan'])} steps")

           # Step 3: Execute plan as behaviors
           print("3. Executing plan as behaviors...")
           execution_result = self.executor.execute_plan_steps(plan['plan'])

           # Step 4: Report results
           print("4. Execution completed")
           print(f"   Final robot state: {self.executor.get_robot_state()}")

           return {
               "success": execution_result["success"],
               "original_goal": goal,
               "generated_plan": plan,
               "execution_results": execution_result["results"],
               "final_robot_state": self.executor.get_robot_state()
           }

   def main():
       if not os.getenv("OPENAI_API_KEY"):
           print("Please set OPENAI_API_KEY environment variable")
           return

       system = CompleteCognitivePlanningSystem()

       test_goals = [
           "Get the red book from the office and bring it to me in the living room",
           "Go to the kitchen, find a glass, and place it on the table",
           "Navigate to the bedroom and wait there for 2 seconds"
       ]

       for i, goal in enumerate(test_goals):
           print(f"\n{'='*70}")
           print(f"EXECUTING GOAL {i+1}: {goal}")
           print(f"{'='*70}")

           result = system.execute_goal(goal)

           if result["success"]:
               print(f"\n🎯 SUCCESS: Goal completed!")
               print(f"   Robot is now at: {result['final_robot_state']['location']}")
               print(f"   Holding: {result['final_robot_state']['holding']}")
               print(f"   Battery: {result['final_robot_state']['battery']:.1f}%")
           else:
               print(f"\n❌ FAILED: {result.get('error', 'Execution failed')}")
               print(f"   Last robot state: {result.get('final_robot_state', 'Unknown')}")

           print("-" * 70)

   if __name__ == "__main__":
       main()
   ```

## Solutions and Hints

### Exercise 1 Solution Highlights
- Plan generation uses structured prompts to get consistent JSON output
- Validation ensures plans contain supported actions
- Simulation provides realistic execution feedback
- Error handling manages failures gracefully

### Exercise 2 Solution Highlights
- Context manager maintains environmental state
- Context-aware generation produces more efficient plans
- Spatial relationships optimize navigation paths
- Object locations enable intelligent task planning

### Exercise 3 Solution Highlights
- Behavior classes encapsulate different robot capabilities
- Behavior executor manages state and execution flow
- Error propagation allows for plan recovery
- Complete system integrates all components

## Assessment Questions

1. How does context-aware planning improve the efficiency of robotic task execution?
2. What are the key differences between action-based and behavior tree approaches?
3. How would you handle a situation where a planned action fails during execution?
4. What safety considerations should be included in cognitive planning systems?

## Extension Ideas

1. Add learning capabilities to improve plan efficiency over time
2. Implement multi-robot coordination for complex tasks
3. Create a web interface for monitoring and controlling the planning system
4. Add natural language feedback for plan execution status
5. Implement plan optimization based on execution history

## Summary

These exercises provide hands-on experience with:
- LLM-based plan generation from natural language goals
- Context-aware planning considering environmental state
- Behavior mapping from abstract plans to executable actions
- Error handling and recovery in plan execution
- Complete cognitive planning pipeline integration

Complete these exercises to gain practical experience with LLM-based cognitive planning for robotics.