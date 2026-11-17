#!/usr/bin/env python3
import re
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node as GzNode

TOPICO_GZ = '/celda/contactos'
FIN_CHOQUE_S = 0.5


def robot_y_eslabon(nombre):
    partes = nombre.split('::')
    if len(partes) < 2 or not re.fullmatch(r'robot\d+', partes[0]):
        return None
    return partes[0], partes[1]


class MonitorChoques(Node):
    def __init__(self):
        super().__init__('monitor_choques')
        self.pub = self.create_publisher(String, '/celda/choques', 10)
        self.activos = {}
        self.total = 0
        self.lock = threading.Lock()
        self.gz = GzNode()
        if not self.gz.subscribe(Contacts, TOPICO_GZ, self.contactos):
            self.get_logger().error(f'no pude suscribirme al topico de Gazebo {TOPICO_GZ}')
        self.create_timer(0.2, self.revisar)
        self.get_logger().info(f'Escuchando contactos entre robots en {TOPICO_GZ} -> /celda/choques')
    def contactos(self, msg):
        ahora = time.time()
        for c in msg.contact:
            a = robot_y_eslabon(c.collision1.name)
            b = robot_y_eslabon(c.collision2.name)
            if not a or not b or a[0] == b[0]:
                continue
            par = tuple(sorted((f'{a[0]}/{a[1]}', f'{b[0]}/{b[1]}')))
            with self.lock:
                nuevo = par not in self.activos
                self.activos[par] = ahora
            if nuevo:
                self.total += 1
                self.avisar(f'CHOQUE {par[0]} <-> {par[1]}', error=True)
    def revisar(self):
        ahora = time.time()
        with self.lock:
            fin = [p for p, t in self.activos.items() if ahora - t > FIN_CHOQUE_S]
            for p in fin:
                del self.activos[p]
        for p in fin:
            self.avisar(f'fin del choque {p[0]} <-> {p[1]}')
    def avisar(self, txt, error=False):
        if error:
            self.get_logger().error(txt)
        else:
            self.get_logger().info(txt)
        self.pub.publish(String(data=txt))


def main():
    rclpy.init()
    nodo = MonitorChoques()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.get_logger().info(f'choques entre robots detectados: {nodo.total}')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
