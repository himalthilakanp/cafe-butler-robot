# Cafe Butler Robot – ROS 2 FSM Based AMR

## Overview
This project implements a ROS 2–based autonomous mobile robot (AMR) that acts as a **butler**
for a café. The robot handles food delivery between the kitchen and customer tables using a
**finite state machine (FSM)** approach.

The system is designed to handle real-world edge cases such as timeouts, multiple orders,
cancellations, and skipped deliveries in a **generic and scalable** manner.

---

## Architecture

The system is composed of three ROS 2 nodes:

### 1. Butler Task Manager
**Package:** `butler_task_manager`  
This node represents the AMR brain and FSM.

**Responsibilities:**
- Maintain robot state (FSM)
- Handle order queue
- Simulate navigation between home, kitchen, and tables
- Handle sequential and multiple deliveries
- Return safely to home position

**FSM States (simplified):**
- `IDLE_AT_HOME`
- `GOING_TO_KITCHEN`
- `AT_KITCHEN`
- `GOING_TO_TABLE_X`
- `AT_TABLE_X`
- `GOING_TO_HOME`

---

### 2. Order Interface
**Package:** `order_interface`  

Publishes food orders to the robot using ROS 2 topics.

**Topic:**
- `/orders` (`std_msgs/String`)

Example:
table1
table1,table2,table3

yaml
Copy code

---

### 3. Scenario Driver (Auto Test Node)
**Package:** `scenario_driver`  

This node automatically validates all problem scenarios mentioned in the assessment.
It behaves like a test harness and publishes orders and cancellations only when the robot
returns to `IDLE_AT_HOME`.

---

## Implemented Scenarios

All assessment cases are fully covered:

1. Single order → kitchen → table → home  
2. No confirmation → timeout → home  
3. No table confirmation → kitchen → home  
4. Cancel while going to table  
5. Multiple table delivery  
6. Skip table if no confirmation  
7. Cancel one table and continue remaining deliveries  

Each scenario is executed sequentially and deterministically.

---

## ROS Topics Used

| Topic Name        | Type              | Description                     |
|------------------|-------------------|---------------------------------|
| `/orders`        | `std_msgs/String` | Incoming food orders            |
| `/amr_status`    | `std_msgs/String` | Robot FSM status updates        |
| `/cancel_order`  | `std_msgs/String` | Order cancellation (optional)   |

---

## How to Build

```bash
cd ~/cafe_robot_ws
colcon build
source install/setup.bash
How to Run
Terminal 1 – Butler Task Manager
bash
Copy code
ros2 run butler_task_manager butler_task_manager
Terminal 2 – Scenario Driver (Auto Validation)
bash
Copy code
ros2 run scenario_driver scenario_driver
Optional: Monitor Robot Status
bash
Copy code
ros2 topic echo /amr_status
Design Highlights
FSM-based architecture

Generic queue-based order handling

Fully event-driven using ROS 2 topics

Automated scenario validation

Clean, modular, and extensible code

Designed to scale beyond 3 tables

Future Improvements
Replace simulated movement with Nav2

Use ROS 2 Actions for navigation

Add real confirmation services

Integrate visualization (RViz)

Author
Himal Thilakan P
Robotics / ROS Developer
