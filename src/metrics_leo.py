
#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy, random, time, os
from math import atan2, sqrt, pi
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from fpdf import FPDF


class RecolectorEspiral:
    def _init_(self):
        rospy.init_node("recolector_espiral")

        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.actualizar_pos)
        self.rate = rospy.Rate(10)
        self.pos = Pose()

        for srv in ["/spawn", "/kill", "/clear",
                    "/turtle1/teleport_absolute", "/turtle1/set_pen"]:
            rospy.wait_for_service(srv)

        self.spawn = rospy.ServiceProxy("/spawn", Spawn)
        self.kill  = rospy.ServiceProxy("/kill",  Kill)
        self.clear = rospy.ServiceProxy("/clear", Empty)
        self.tp    = rospy.ServiceProxy("/turtle1/teleport_absolute",
                                        TeleportAbsolute)
        self.pen   = rospy.ServiceProxy("/turtle1/set_pen", SetPen)

        self.objetivo = {}
        self.tiempos  = []

    def actualizar_pos(self, data):
        self.pos = data

    def reset(self):
        self.clear()
        self.pen(0, 0, 0, 0, 1)
        self.tp(1.0, 1.0, 0.0)
        self.pen(255, 255, 255, 3, 0)

    def girar_hacia(self, x, y):
        vel = Twist()
        while not rospy.is_shutdown():
            ang   = atan2(y - self.pos.y, x - self.pos.x)
            err   = (ang - self.pos.theta + pi) % (2*pi) - pi
            vel.angular.z = 4.5 * err
            vel.linear.x  = 0.0
            self.pub.publish(vel)
            if abs(err) < 0.05:
                break
            self.rate.sleep()
        self.pub.publish(Twist())

    def avanzar_hacia(self, x, y):
        vel = Twist()
        while not rospy.is_shutdown():
            dist = sqrt((x - self.pos.x)*2 + (y - self.pos.y)*2)
            ang  = atan2(y - self.pos.y, x - self.pos.x)
            err  = (ang - self.pos.theta + pi) % (2*pi) - pi

            vel.linear.x  = 1.8 * dist
            vel.angular.z = 6.0 * err
            self.pub.publish(vel)

            if self.objetivo and self.objetivo_detectado():
                self.kill(self.objetivo["nombre"])
                self.pub.publish(Twist())
                return True

            if dist < 0.10:
                break
            self.rate.sleep()

        self.pub.publish(Twist())
        return False

    def crear_objetivo(self, idx):
        x = round(random.uniform(1.5, 9.5), 2)
        y = round(random.uniform(1.5, 9.5), 2)
        nombre = f"obj{idx}"
        self.spawn(x, y, 0.0, nombre)
        self.objetivo = {"nombre": nombre, "x": x, "y": y}

    def objetivo_detectado(self):
        dx = self.objetivo["x"] - self.pos.x
        dy = self.objetivo["y"] - self.pos.y
        dist = sqrt(dx*dx + dy*dy)
        return dist <= 0.35

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

    def buscar_y_eliminar(self):
        t0 = time.perf_counter()

        for wx, wy in self.generar_waypoints_espiral():
            self.girar_hacia(wx, wy)
            if self.avanzar_hacia(wx, wy):
                break

        dt = round(time.perf_counter() - t0, 2)
        return dt

    def hacer_pdf(self):
        prom = round(sum(self.tiempos) / len(self.tiempos), 2)

        pdf = FPDF()
        pdf.add_page()
        pdf.set_font("Arial", size=12)
        pdf.cell(200, 10, "Tiempos de búsqueda en espiral", 0, 1, "C")
        pdf.ln(5)

        for i, t in enumerate(self.tiempos, 1):
            pdf.cell(200, 10, f"Prueba {i}: {t} s", ln=True)

        pdf.ln(5)
        pdf.set_font("Arial", style="B", size=12)
        pdf.cell(200, 10, f"Promedio: {prom} s", ln=True)

        ruta = os.path.expanduser("~/reporte_espiral.pdf")
        pdf.output(ruta)
        print(f"\nPDF generado en: {ruta}")

    def run(self):
        for i in range(1, 6):
            if rospy.is_shutdown():
                break
            print(f"\n--- PRUEBA {i} ---")
            self.reset()
            self.crear_objetivo(i)
            rospy.sleep(1)

            tiempo = self.buscar_y_eliminar()
            self.tiempos.append(tiempo)
            print(f"   → Objetivo eliminado en {tiempo} s")

            rospy.sleep(2)

        if self.tiempos:
            self.hacer_pdf()
        rospy.signal_shutdown("Terminado")


if _name_ == "_main_":
    try:
        bot = RecolectorEspiral()
        bot.run()
    except rospy.ROSInterruptException:
        pass
