#!/usr/bin/env python3
import rospy
import random
import time
from math import atan2, sqrt, pow, pi
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from fpdf import FPDF
import os

class Recolector:
    def __init__(self):
        rospy.init_node('recolector_bot')

        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.actualizar_pos)
        self.rate = rospy.Rate(10)
        self.pos = Pose()

        rospy.wait_for_service('/spawn')
        rospy.wait_for_service('/kill')
        rospy.wait_for_service('/clear')
        rospy.wait_for_service('/turtle1/teleport_absolute')
        rospy.wait_for_service('/turtle1/set_pen')

        self.spawn = rospy.ServiceProxy('/spawn', Spawn)
        self.kill = rospy.ServiceProxy('/kill', Kill)
        self.clear = rospy.ServiceProxy('/clear', Empty)
        self.tp = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)

        self.objetivos = []
        self.tiempos = []

    def actualizar_pos(self, data):
        self.pos = data

    def reset(self):
        self.clear()
        self.pen(0, 0, 0, 0, 1)
        self.tp(5.5, 5.5, 0)
        self.pen(255, 255, 255, 3, 0)

    def crear_objetivos(self):
        self.objetivos = []
        n = int(input("\nCuantos objetos quieres poner? "))
        for i in range(n):
            x = round(random.uniform(1.0, 10.0), 2)
            y = round(random.uniform(1.0, 10.0), 2)
            nombre = f"obj{i+1}"
            self.spawn(x, y, 0.0, nombre)
            self.objetivos.append({'nombre': nombre, 'x': x, 'y': y})

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

    def recolectar(self):
        tiempos_una_prueba = []
        objetivos_restantes = self.objetivos.copy()

        while objetivos_restantes:
            # Encontrar el objetivo más cercano
            objetivo_cercano = min(objetivos_restantes, key=lambda obj: sqrt((obj['x'] - self.pos.x)**2 + (obj['y'] - self.pos.y)**2))

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

    def hacer_pdf(self):
        total = sum(t for prueba in self.tiempos for (_, t) in prueba)
        total_items = sum(len(p) for p in self.tiempos)
        prom_item = round(total / total_items, 2)
        prom_prueba = [round(sum(t for (_, t) in p), 2) for p in self.tiempos]
        prom_global = round(sum(prom_prueba) / len(prom_prueba), 2)

        pdf = FPDF()
        pdf.add_page()
        pdf.set_font("Arial", size=12)
        pdf.cell(200, 10, txt="Resumen de Tiempos - Bot Recolector", ln=True, align='C')
        pdf.ln(10)

        for i, prueba in enumerate(self.tiempos, 1):
            pdf.set_font("Arial", style='B', size=12)
            pdf.cell(200, 10, txt=f"Prueba {i}:", ln=True)
            pdf.set_font("Arial", size=12)
            for j, (nombre, t) in enumerate(prueba, 1):
                pdf.cell(200, 10, txt=f"  {nombre}: {t} s", ln=True)
            pdf.cell(200, 10, txt=f"  Total prueba: {prom_prueba[i-1]} s", ln=True)
            pdf.ln(5)

        pdf.set_font("Arial", style='B', size=12)
        pdf.cell(200, 10, txt="Promedios finales:", ln=True)
        pdf.set_font("Arial", size=12)
        pdf.cell(200, 10, txt=f"- Promedio por objeto: {prom_item} s", ln=True)
        pdf.cell(200, 10, txt=f"- Promedio por prueba: {prom_global} s", ln=True)
        pdf.cell(200, 10, txt=f"- Tiempo total acumulado: {round(total, 2)} s", ln=True)

        ruta = os.path.expanduser("~/reporte_bot.pdf")
        pdf.output(ruta)
        print(f"\nPDF generado: {ruta}")

    def run(self):
        vueltas = 0
        while not rospy.is_shutdown() and vueltas < 5:
            print(f"\n--- INICIO DE PRUEBA {vueltas+1} ---")
            self.reset()
            self.crear_objetivos()
            rospy.sleep(2)
            self.recolectar()
            print("esperando para la siguiente...\n")
            rospy.sleep(3)
            vueltas += 1

        self.hacer_pdf()
        rospy.signal_shutdown("Listo")

if __name__ == '__main__':
    try:
        bot = Recolector()
        bot.run()
    except rospy.ROSInterruptException:
        pass

