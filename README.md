# Turtlesim Search Methods Repository

This repository contains four different Python scripts using ROS and Turtlesim to demonstrate various search and collection strategies. Each script spawns multiple random turtles in the Turtlesim environment, and the main turtle (turtle1) collects them following a distinct method. The goal is to compare the efficiency of each method by measuring the time taken to collect all turtles.

---

## Overview of the Project

- Each script spawns a number of turtles at random positions.
- The main turtle navigates to collect each spawned turtle by moving and "killing" it.
- Each script uses a different strategy for the order in which turtles are collected.
- After multiple test runs, the scripts generate a PDF report summarizing the times.

---

# 1. Collection in Spawn Order

This script collects turtles in the exact order in which they were spawned.

### Code Explanation

```python
def crear_objetivos(self):
    self.objetivos = []
    n = int(input("\nCuantos objetos quieres poner? "))
    for i in range(n):
        x = round(random.uniform(1.0, 10.0), 2)
        y = round(random.uniform(1.0, 10.0), 2)
        nombre = f"obj{i+1}"
        self.spawn(x, y, 0.0, nombre)
        self.objetivos.append({'nombre': nombre, 'x': x, 'y': y})
```
crear_objetivos(self):
This method asks the user how many turtles to spawn (n).
It generates n turtles at random (x,y) coordinates within the Turtlesim window using random.uniform(1.0, 10.0) and rounds the values.
Each turtle is spawned by calling the ROS service self.spawn.
The spawned turtles' data is stored in the list self.objetivos with their name and position.

```python
def girar_hacia(self, x, y):
    vel = Twist()
    while not rospy.is_shutdown():
        ang = atan2(y - self.pos.y, x - self.pos.x)
        err = (ang - self.pos.theta + pi) % (2 * pi) - pi
        vel.angular.z = 4.5 * err
        vel.linear.x = 0
        self.pub.publish(vel)
        if abs(err) < 0.05:
            break
        self.rate.sleep()
    self.pub.publish(Twist())
```
girar_hacia(self, x, y):
Rotates the main turtle to face a point (x,y).
It calculates the angle between the turtle's current position and the target point using atan2.
The angular velocity is adjusted proportionally to the angle error until the turtle is facing within 0.05 radians of the target.
The turtle stops rotation once aligned.

```python
def avanzar_hacia(self, x, y):
    vel = Twist()
    while not rospy.is_shutdown():
        dist = sqrt(pow(x - self.pos.x, 2) + pow(y - self.pos.y, 2))
        ang = atan2(y - self.pos.y, x - self.pos.x)
        err = (ang - self.pos.theta + pi) % (2 * pi) - pi
        vel.linear.x = 1.8 * dist
        vel.angular.z = 6.0 * err
        self.pub.publish(vel)
        if dist < 0.3:
            break
        self.rate.sleep()
    self.pub.publish(Twist())
```
avanzar_hacia(self, x, y):
Moves the main turtle towards the coordinates (x,y).
The linear velocity is proportional to the distance remaining to the target to speed up or slow down as needed.
Simultaneously, the turtle adjusts its angular velocity to maintain direction towards the target.
Stops moving once within 0.3 units of the target.
```python
def recolectar(self):
    tiempos_una_prueba = []
    for obj in self.objetivos:
        print(f"-> yendo a {obj['nombre']}")
        t0 = time.perf_counter()
        self.girar_hacia(obj['x'], obj['y'])
        self.avanzar_hacia(obj['x'], obj['y'])
        t1 = time.perf_counter()
        tiempo = round(t1 - t0, 2)
        tiempos_una_prueba.append(tiempo)
        print(f"   listo, tardo {tiempo} s")
        self.kill(obj['nombre'])
    self.tiempos.append(tiempos_una_prueba)
```
recolectar(self):
Collects all turtles in the order stored in self.objetivos.
For each target turtle:
- Records the start time.
- Calls girar_hacia and avanzar_hacia to navigate to the target.
- Measures the time taken and appends it to tiempos_una_prueba.
- Kills the turtle to simulate collection.
- After all are collected, stores the times for that trial.

