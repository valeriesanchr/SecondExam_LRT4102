#!/usr/bin/env python3
import rospy
import random
import time
from math import atan2, sqrt, pi
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
        self.camino = []
        self.radio_recoleccion = 0.5
        self.tiempos = []
        self.recolectados = {}

    def actualizar_pos(self, data):
        self.pos = data

    def reset(self):
        self.clear()
        self.pen(0, 0, 0, 0, 1)
        self.tp(1.0, 1.0, 0)
        self.pen(255, 255, 255, 2, 0)

    def crear_objetivos(self):
        self.objetivos = []
        n = int(input("\nCuántos objetos quieres poner? "))
        for i in range(n):
            x = round(random.uniform(1.0, 10.0), 2)
            y = round(random.uniform(1.0, 10.0), 2)
            nombre = f"obj{i+1}"
            self.spawn(x, y, 0.0, nombre)
            self.objetivos.append({'nombre': nombre, 'x': x, 'y': y})

    def generar_camino(self):
        self.camino = []
        filas = 10
        columnas = 10
        y_vals = [1.0 + i for i in range(filas)]

        for i, y in enumerate(y_vals):
            if i % 2 == 0:
                for x in [1.0 + j for j in range(columnas)]:
                    self.camino.append((x, y))
            else:
                for x in reversed([1.0 + j for j in range(columnas)]):
                    self.camino.append((x, y))

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
            dist = sqrt((x - self.pos.x)**2 + (y - self.pos.y)**2)
            ang = atan2(y - self.pos.y, x - self.pos.x)
            err = (ang - self.pos.theta + pi) % (2 * pi) - pi
            vel.linear.x = 1.5 * dist
            vel.angular.z = 4.0 * err
            self.pub.publish(vel)

            self.detectar_objetivos_cercanos()

            if dist < 0.3:
                break
            self.rate.sleep()
        self.pub.publish(Twist())

    def detectar_objetivos_cercanos(self):
        for obj in self.objetivos[:]:
            dist = sqrt((obj['x'] - self.pos.x)**2 + (obj['y'] - self.pos.y)**2)
            if dist < self.radio_recoleccion:
                print(f"Recolectado: {obj['nombre']} en ({obj['x']}, {obj['y']})")
                t1 = time.perf_counter()
                tiempo = round(t1 - self.recolectados[obj['nombre']]['inicio'], 2)
                self.recolectados[obj['nombre']]['tiempo'] = tiempo
                self.tiempos.append(tiempo)
                try:
                    self.kill(obj['nombre'])
                    self.objetivos.remove(obj)
                except rospy.ServiceException:
                    pass

    def seguir_camino(self):
        for (x, y) in self.camino:
            for obj in self.objetivos:
                if obj['nombre'] not in self.recolectados:
                    self.recolectados[obj['nombre']] = {'inicio': time.perf_counter(), 'tiempo': None}
            self.girar_hacia(x, y)
            self.avanzar_hacia(x, y)
            rospy.sleep(0.1)

    def hacer_pdf(self):
        pdf = FPDF()
        pdf.add_page()
        pdf.set_font("Arial", size=12)
        pdf.cell(200, 10, txt="Resumen de Recolección - Camino Fijo", ln=True, align='C')
        pdf.ln(10)

        if not self.tiempos:
            pdf.cell(200, 10, txt="No se recolectó ningún objeto.", ln=True)
        else:
            for i, (nombre, datos) in enumerate(self.recolectados.items(), 1):
                if datos['tiempo'] is not None:
                    pdf.cell(200, 10, txt=f"{i}. {nombre}: {datos['tiempo']} s", ln=True)

            total = round(sum(self.tiempos), 2)
            prom_item = round(total / len(self.tiempos), 2)

            pdf.ln(10)
            pdf.set_font("Arial", style='B', size=12)
            pdf.cell(200, 10, txt="Resumen:", ln=True)
            pdf.set_font("Arial", size=12)
            pdf.cell(200, 10, txt=f"- Promedio por objeto: {prom_item} s", ln=True)
            pdf.cell(200, 10, txt=f"- Promedio por prueba: {total} s", ln=True)
            pdf.cell(200, 10, txt=f"- Tiempo total acumulado: {total} s", ln=True)

        ruta = os.path.expanduser("~/Desktop/reporte_prueba_1.pdf")
        pdf.output(ruta)
        print(f"\nPDF generado: {ruta}")

    def run(self):
        self.reset()
        self.crear_objetivos()
        self.generar_camino()
        rospy.sleep(2)
        self.seguir_camino()
        self.hacer_pdf()
        rospy.signal_shutdown("Camino completado")

if __name__ == '__main__':
    try:
        bot = Recolector()
        bot.run()
    except rospy.ROSInterruptException:
        pass

