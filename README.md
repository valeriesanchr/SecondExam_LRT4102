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


Sample Collection Times

| Trial | Total Time (s) |
| ----- | -------------- |
| 1     | 3.66           |
| 2     | 11.73          |
| 3     | 23.58          |
| 4     | 3.98           |
| 5     | 11.4           |

- Average time per turtle: 1.94 seconds

- Average time per trial: 10.87 seconds

- Total accumulated time: 54.35 seconds

# 2. Collection by nearest turtle first

This script collects turtles by always going to the closest remaining turtle.

### Code Explanation
The code is similar to the first script but differs mainly in the recolectar method:
```python
def recolectar(self):
    tiempos_una_prueba = []
    objetivos_restantes = self.objetivos.copy()

    while objetivos_restantes:
        # Find the nearest turtle to current position
        objetivo_cercano = min(
            objetivos_restantes,
            key=lambda obj: sqrt((obj['x'] - self.pos.x)**2 + (obj['y'] - self.pos.y)**2)
        )

        print(f"-> yendo a {objetivo_cercano['nombre']}")
        t0 = time.perf_counter()
        self.girar_hacia(objetivo_cercano['x'], objetivo_cercano['y'])
        self.avanzar_hacia(objetivo_cercano['x'], objetivo_cercano['y'])
        t1 = time.perf_counter()

        tiempo = round(t1 - t0, 2)
        tiempos_una_prueba.append((objetivo_cercano['nombre'], tiempo))
        print(f"   listo, tardo {tiempo} s")

        self.kill(objetivo_cercano['nombre'])
        objetivos_restantes.remove(objetivo_cercano)

    self.tiempos.append(tiempos_una_prueba)
```
recolectar(self):
Instead of following the spawn order, it dynamically chooses the closest turtle each time.
- Copies the list of targets into objetivos_restantes.
- Finds the nearest turtle to the current position using a lambda function with Euclidean distance.
- Navigates to it and records the time taken.
- Removes the collected turtle from the remaining list.
- Repeats until all turtles are collected.

Sample collection times
| Trial | Total Time (s) |
| ----- | -------------- |
| 1     | 8.7            |
| 2     | 7.44           |
| 3     | 8.53           |
| 4     | 9.74           |
| 5     | 7.96           |

- Average time per turtle: ~1.66 seconds

- Average time per trial: ~8.87 seconds

- Total accumulated time: (Sum of trials above)

# 3. Spiral Path Collector

In this method, the collector turtle explores the map in a spiral pattern. The area is implicitly divided into shrinking concentric rectangles, and the turtle follows each segment of the spiral while scanning for targets. Once it detects a turtle within a certain proximity, it stops and eliminates the target.

**Key features:**
- A single target is spawned in a random location.
- The collector turtle follows waypoints forming a spiral trajectory.
- When the turtle gets close enough to the target (within 0.35 units), it is considered collected and eliminated.
- The process is repeated five times to gather performance data.

**Sample spiral waypoints:**
```python
def generar_waypoints_espiral(self, paso=1.0):
    min_x, min_y = 1.0, 1.0
    max_x, max_y = 10.0, 10.0
    while min_x < max_x and min_y < max_y:
        yield (max_x, min_y)
        yield (max_x, max_y)
        yield (min_x, max_y)
        yield (min_x, min_y + paso)
        min_x += paso
        min_y += paso
        max_x -= paso
        max_y -= paso
```
| Trial | Time (s) |
| ----- | -------- |
| 1     | 26.14    |
| 2     | 23.36    |
| 3     | 38.33    |
| 4     | 24.25    |
| 5     | 25.93    |

Average Time: 27.6 s
Total Time: 139.01 s

**Advantages**:
- Systematic coverage of the entire map.
- Balanced and deterministic path for search.

**Disadvantages**:
- Not optimal if the target is near the starting point.
- May take longer if the target lies outside the early spiral loops.

# 4. Fixed-Path Grid Collector
In this approach, the main turtle follows a predefined zigzag grid pattern, similar to a lawnmower traversal. While moving through the path, it continuously checks for nearby turtles (within 0.5 units). If one is detected, it is collected and eliminated.

**Key features:**
- The number of target turtles is user-defined and spawned randomly.
- A full 10x10 grid path is generated, sweeping the map horizontally.
- The turtle does not deviate from the path — instead, it collects any target it encounters along the way.
- All detected targets are timestamped for individual collection time reporting.
  
```python
for i, y in enumerate(y_vals):
    if i % 2 == 0:
        for x in [1.0 + j for j in range(columnas)]:
            camino.append((x, y))
    else:
        for x in reversed([1.0 + j for j in range(columnas)]):
            camino.append((x, y))
```

Sample collection times
| Object | Time (s) |
| ------ | -------- |
| obj1   | 24.11    |
| obj2   | 91.21    |
| obj3   | 61.61    |
| obj4   | 110.01   |
| obj5   | 30.81    |

Since this method takes a lot of time to complete, we only did one trial run.

**Advantages:**
- Ensures full area coverage with simple implementation.
- Works well in cluttered or multi-target environments.

**Disadvantages:**
- Not adaptive: the turtle does not prioritize closer targets.
- Inefficient if targets are far from the traversal path early on.

| Method                          | Strengths                    | Weaknesses                          |
| --------------------------      | -----------------------------| -------------                       |
| 1. Collection in Spawn Order    | Fast for scattered targets   | May miss global optimal paths       |
| 2. Collection by nearest turtle | Simple to implement          | Highly inefficient and inconsistent |
| 3. Spiral Pattern               | Systematic, fair performance | Can be suboptimal in early passes   |
| 4. Fixed Grid Path              | Complete coverage guaranteed | Not adaptive, slow for nearby items |



