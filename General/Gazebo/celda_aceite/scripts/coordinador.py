#!/usr/bin/env python3
# Coordinador de la celda. Comandos por /celda/comando o UDP 5005:
# START, UNA, REAL, UNA REAL, STOP, HOME, RESET, ESTADO, MANUAL R3 e1 e2 z garra
import json
import math
import os
import re
import socket
import statistics
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

MUNDO = 'celda_aceite'
HZ = 50.0

CONTROL = {
    'robot1': [('jtc', 'arm_controller', ['joint_1', 'joint_2', 'joint_3', 'joint_4'])],
    'robot2': [('posicion', 'scara_controller', ['Revoluci_n_1', 'Revoluci_n_2', 'Corredera_4'])],
    'robot3': [('jtc', 'arm_controller', ['eslabon1_joint', 'eslabon2_joint', 'prismatico_joint']),
               ('jtc', 'gripper_controller', ['garra1_joint', 'garra2_joint'])],
    'robot4': [('posicion', 'joint1_position_controller', ['Joint_1']),
               ('posicion', 'joint2_position_controller', ['Joint_2']),
               ('velocidad', 'joint3_velocity_controller', ['Joint_3'])],
    'robot5': [('jtc', 'arm_controller', ['Join_1', 'Joint_2', 'Joint_3'])],
}
HERRAMIENTA = {'robot2': 'Cremallera2', 'robot3': 'tcp', 'robot4': 'Link_3', 'robot5': 'Link_3'}
PRISMATICAS = {'Corredera_4', 'prismatico_joint', 'Joint_3'}
ORDEN_IR_HOME = ['robot5', 'robot4', 'robot3', 'robot2', 'robot1']
GARRA = {'abrir': 0.45, 'cerrar': 0.24}
V_REV, V_PRIS = 1.1, 0.035
ROSCA_ALTO, ROSCA_VUELTAS, ROSCA_T = 0.004, 3.0, 3.0
LIMITES_MANUAL = {'robot3': [('eslabon1_joint', -2.6, 2.6), ('eslabon2_joint', -1.95, 1.95),
                             ('prismatico_joint', 0.001, 0.09), ('garra1_joint', 0.0, 0.45)]}
T_CARGAR, T_VERTER = 0.8, 1.2
TRAMOS = {'ir': range(0, 4), 'llevar': range(4, 8), 'volver': range(8, 9)}


def suave(s):
    return s * s * (3 - 2 * s)


def mat(t, r):
    x, y, z, w = r.x, r.y, r.z, r.w
    return [[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w), t.x],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w), t.y],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y), t.z],
            [0, 0, 0, 1]]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inv(A):
    R = [[A[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(R[i][k] * A[k][3] for k in range(3)) for i in range(3)]
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]], [0, 0, 0, 1]]


def traslacion(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def a_pose(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(T[0][3]), float(T[1][3]), float(T[2][3])
    m = T
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w, x, y, z = (m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w, x, y, z = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w, x, y, z = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = float(x), float(y), float(z), float(w)
    return p


class Parada(Exception):
    """se pidio HOME/RESET con la linea andando: los hilos de los robots terminan"""


class Coordinador(Node):
    def __init__(self):
        super().__init__('coordinador_celda')
        self.declare_parameter('puerto_udp', 5005)
        self.declare_parameter('velocidad', 1.0)
        self.declare_parameter('reales', '')
        self.declare_parameter('espera_listo_s', 120.0)
        r = str(self.get_parameter('reales').value).strip().lower()
        self.reales_pedidos = set(CONTROL) if r in ('todos', 'all') else {x.strip() for x in r.split(',') if x.strip()}
        self.reales = set()
        self.esp_dir, self.esp_ok, self.esp_listo = {}, set(), set()
        self.pendiente, self.n_orden, self.sock_udp = {}, {}, None
        cfgdir = os.path.join(get_package_share_directory('celda_aceite'), 'config')
        self.sec = json.load(open(os.path.join(cfgdir, 'secuencia.json')))
        cfg = yaml.safe_load(open(os.path.join(cfgdir, 'robots.yaml')))['robots']
        est = yaml.safe_load(open(os.path.join(cfgdir, 'estaciones.yaml')))
        self.n_pool = int(est.get('pool', 1))
        self.home = {ns: {j: float(v) for j, v in (r.get('home') or {}).items()} for ns, r in cfg.items()}
        conf = json.load(open(os.path.join(cfgdir, 'conflictos.json')))
        self.conflictos = {frozenset((a, b)) for a, b, *_ in conf['conflictos']}
        self.vel = float(self.get_parameter('velocidad').value)
        cb = ReentrantCallbackGroup()
        self.pubs, self.js = {}, {}
        for ns, ctrls in CONTROL.items():
            for tipo, nombre, _ in ctrls:
                topico = f'/{ns}/{nombre}/' + ('joint_trajectory' if tipo == 'jtc' else 'commands')
                tipo_msg = JointTrajectory if tipo == 'jtc' else Float64MultiArray
                self.pubs[(ns, nombre)] = self.create_publisher(tipo_msg, topico, 10)
            self.create_subscription(JointState, f'/{ns}/joint_states',
                                     lambda m, ns=ns: self.js.__setitem__(ns, dict(zip(m.name, m.position))), 10,
                                     callback_group=cb)
        self.pub_estado = self.create_publisher(String, '/celda/estado', 10)
        self.pub_prod = self.create_publisher(String, '/celda/produccion', 10)
        self.pub_band = self.create_publisher(String, '/celda/banderas', 50)
        self.pub_real = {ns: self.create_publisher(JointState, f'/{ns}_real/joint_states', 10) for ns in CONTROL}
        self.create_subscription(String, '/celda/comando', lambda m: self.comando(m.data), 10, callback_group=cb)
        self.set_pose = self.create_client(SetEntityPose, f'/world/{MUNDO}/set_pose', callback_group=cb)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.cmd = {ns: dict(q) for ns, q in self.home.items()}
        self.cmd['robot3'].update({'garra1_joint': 0.0, 'garra2_joint': 0.0})
        self.pend_lock, self.pendientes = threading.Lock(), {}
        self.cv = threading.Condition()
        self.llevando = {}
        self.objetos = {}
        self.reiniciar_objetos(mover=False)
        self.ocupado = False
        self.lock = threading.Lock()
        self.create_timer(1.0 / HZ, self.mantener, callback_group=cb)
        threading.Thread(target=self.udp, daemon=True).start()
        threading.Thread(target=self.enviar_poses, daemon=True).start()
        if self.reales_pedidos:
            self.get_logger().info('REAL exige los ESP32 de: ' + ', '.join(sorted(self.reales_pedidos)) +
                                   '. Esperando su HOLA por UDP...')
        self.get_logger().info('Listo. Manda START (linea), UNA (una botella) o STOP por /celda/comando '
                               'o por UDP (puerto %d).' % self.get_parameter('puerto_udp').value)
    def comando(self, texto):
        resp = self._comando(texto)
        self.estado(f'{texto.strip().upper()} -> {resp}')
        return resp
    def _comando(self, texto):
        t = texto.strip().upper()
        t = ' '.join(t.split())
        if t in ('START', 'INICIO', '1', 'UNA', 'REAL', 'START REAL', 'UNA REAL', 'REAL UNA'):
            if self.ocupado:
                return 'OCUPADO: la linea ya esta corriendo (STOP para pararla)'
            faltan = self.listos()
            if faltan:
                return 'NO LISTO: sin controladores activos en ' + ', '.join(faltan)
            real = 'REAL' in t
            if real:
                if self.faltan_esp():
                    return 'NO LISTO: faltan los ESP32 de ' + ', '.join(self.faltan_esp()) + ' (no mandaron HOLA)'
                if not self.esp_dir:
                    return 'NO LISTO: no hay ningun ESP32 conectado (tienen que mandar "Rn HOLA" al puerto UDP)'
            self.reales = set(self.esp_dir) | self.reales_pedidos if real else set()
            limite = 1 if 'UNA' in t else None
            threading.Thread(target=self.linea, args=(limite,), daemon=True).start()
            con = (' con robots reales: ' + ', '.join(sorted(self.reales))) if real else ' (solo animacion)'
            return ('OK: produciendo una botella' if limite else 'OK: linea en marcha (STOP para parar)') + con
        if t in ('STOP', 'PARAR'):
            if not self.ocupado:
                return 'LIBRE: la linea no esta corriendo'
            with self.cv:
                self.parar = True
                self.cv.notify_all()
            return 'OK: termino las botellas en curso y vuelvo a HOME'
        if t == 'RESET':
            if self.ocupado:
                return 'OCUPADO (manda STOP primero)'
            threading.Thread(target=self.reset, daemon=True).start()
            return 'OK: volviendo al inicio'
        if t == 'HOME':
            if self.ocupado:
                return 'OCUPADO (manda STOP primero)'
            threading.Thread(target=self.ir_home, daemon=True).start()
            return 'OK: llevando los robots a HOME'
        if t.startswith('MANUAL'):
            p = t.split()
            ns = f'robot{p[1][1:]}' if len(p) > 1 and p[1].startswith('R') else None
            if ns not in LIMITES_MANUAL:
                return 'uso: MANUAL R3 <eslabon1> <eslabon2> <prismatico> <garra>  (rad, rad, m, rad)'
            if self.ocupado:
                return 'OCUPADO: el movimiento a mano solo se puede con la linea detenida (STOP primero)'
            try:
                vals = [float(x) for x in p[2:]]
            except ValueError:
                return 'MANUAL: los valores tienen que ser numeros'
            if len(vals) != len(LIMITES_MANUAL[ns]):
                return f'MANUAL {p[1]}: se esperan {len(LIMITES_MANUAL[ns])} valores'
            self.ocupado = True
            threading.Thread(target=self.manual, args=(ns, vals), daemon=True).start()
            return f'OK: moviendo {ns} a mano'
        if t == 'ESTADO':
            return ('PRODUCIENDO' if self.ocupado else 'LIBRE') + ' | ' + self.texto_produccion()
        return f'?? comando desconocido: {texto!r} (usa START, UNA, REAL, UNA REAL, STOP, RESET, HOME o ESTADO)'
    def udp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('0.0.0.0', int(self.get_parameter('puerto_udp').value)))
        self.sock_udp = s
        while rclpy.ok():
            datos, origen = s.recvfrom(1024)
            texto = datos.decode(errors='ignore')
            if self.bandera(texto, origen):
                continue
            resp = self.comando(texto)
            self.get_logger().info(f'(por UDP desde {origen[0]})')
            s.sendto((resp + '\n').encode(), origen)
    def bandera(self, texto, origen):
        m = re.match(r'^\s*R([1-5])\s+(HOLA|OK|LISTO|POS)\b\s*(\S*)\s*(\S*)', texto, re.I)
        if not m:
            return False
        ns, tipo, tramo, n = f'robot{m.group(1)}', m.group(2).upper(), m.group(3), m.group(4)
        if tipo == 'POS':
            self.encoders(ns, texto.split()[2:])
            return True
        self.pub_band.publish(String(data=f'<- {texto.strip()}'))
        with self.cv:
            if tipo == 'HOLA':
                nuevo = self.esp_dir.get(ns) != origen
                self.esp_dir[ns] = origen
                self.sock_udp.sendto(f'R{ns[-1]} BIENVENIDO\n'.encode(), origen)
                if nuevo:
                    self.get_logger().info(f'{ns}: ESP32 conectado desde {origen[0]}:{origen[1]}')
            elif tipo == 'OK':
                self.esp_ok.add((ns, tramo, n))
            else:
                self.esp_listo.add((ns, tramo, n))
                self.get_logger().info(f'{ns}: bandera LISTO {tramo} {n}')
            self.cv.notify_all()
        return True
    def ordenar(self, ns, tramo):
        if ns not in self.reales:
            return
        n = str(self.n_orden.get(ns, 0) + 1); self.n_orden[ns] = int(n)
        self.pendiente[ns] = (tramo, n)
        msg = f'R{ns[-1]} HAZ {tramo} {n}\n'.encode()
        self.pub_band.publish(String(data=f'-> R{ns[-1]} HAZ {tramo} {n}'))
        def repetir():
            while rclpy.ok() and (ns, tramo, n) not in self.esp_ok and (ns, tramo, n) not in self.esp_listo:
                self.sock_udp.sendto(msg, self.esp_dir[ns])
                time.sleep(1.0)
        threading.Thread(target=repetir, daemon=True).start()
    def cerrar(self, ns):
        p = self.pendiente.pop(ns, None)
        if not p:
            return
        tramo, n = p
        limite = time.time() + float(self.get_parameter('espera_listo_s').value)
        with self.cv:
            while (ns, tramo, n) not in self.esp_listo:
                self.chequear_parada()
                if time.time() > limite:
                    raise RuntimeError(f'el ESP32 de {ns} no mando LISTO de "{tramo}" (orden {n})')
                self.cv.wait(0.2)
            self.esp_listo.discard((ns, tramo, n))
            self.esp_ok.discard((ns, tramo, n))
    def encoders(self, ns, valores):
        try:
            v = [float(x) for x in valores]
        except ValueError:
            return
        nombres = [j for _, _, js in CONTROL[ns] for j in js]
        if ns == 'robot3' and len(v) == len(nombres) - 1:
            v.append(v[-1])
        if len(v) != len(nombres):
            return
        m = JointState(); m.header.stamp = self.get_clock().now().to_msg()
        m.name, m.position = nombres, v
        self.pub_real[ns].publish(m)
    def manual(self, ns, valores):
        with self.lock:
            self.ocupado = True
            try:
                q = {}
                for (j, lo, hi), v in zip(LIMITES_MANUAL[ns], valores):
                    q[j] = min(hi, max(lo, v))
                if ns == 'robot3':
                    q['garra2_joint'] = q['garra1_joint']
                n = None
                if ns in self.esp_dir:
                    n = str(self.n_orden.get(ns, 0) + 1); self.n_orden[ns] = int(n)
                    vals = ' '.join(f'{q[j]:.4f}' for j, _, _ in LIMITES_MANUAL[ns])
                    msg = f'R{ns[-1]} MUEVE {vals} {n}'
                    self.pub_band.publish(String(data='-> ' + msg))
                    def repetir():
                        while rclpy.ok() and (ns, 'MUEVE', n) not in self.esp_ok and (ns, 'MUEVE', n) not in self.esp_listo:
                            self.sock_udp.sendto((msg + '\n').encode(), self.esp_dir[ns])
                            time.sleep(1.0)
                    threading.Thread(target=repetir, daemon=True).start()
                self.estado(f'MANUAL {ns}: ' + ', '.join(f'{j}={v:.3f}' for j, v in q.items()))
                self.mover(ns, q, minimo=1.0)
                if n:
                    self.pendiente[ns] = ('MUEVE', n)
                    self.cerrar(ns)
                self.estado(f'MANUAL {ns}: listo')
            except Exception as e:
                self.estado(f'ERROR en MANUAL {ns}: {e}')
            finally:
                self.ocupado = False
    def faltan_esp(self):
        return sorted(ns for ns in self.reales_pedidos if ns not in self.esp_dir)
    def estado(self, txt):
        self.get_logger().info(txt)
        self.pub_estado.publish(String(data=txt))
    def nombres(self, i):
        sufijo = '' if i == 0 else f'_{i}'
        return f'botella{sufijo}', f'botella_llena{sufijo}', f'tapa{sufijo}'
    def escondido(self, nombre):
        base = nombre.split('_')[-1]
        i = int(base) if base.isdigit() else 0
        k = 2 if nombre.startswith('tapa') else (1 if nombre.startswith('botella_llena') else 0)
        return traslacion(0.10 * i, 0.10 * k, -0.5)
    def reiniciar_objetos(self, mover=True):
        b = self.sec['objetos_inicio']['botella']; t = self.sec['objetos_inicio']['tapa']
        self.T_EB, self.T_RT = traslacion(*b), traslacion(*t)
        self.objetos = {}
        for i in range(self.n_pool):
            for n in self.nombres(i):
                self.objetos[n] = self.escondido(n)
        with self.cv:
            self.botellas_libres = list(range(self.n_pool))
            self.tapas_libres = list(range(self.n_pool))
            self.est = {'EB': None, 'PLL': None, 'PS': None, 'RT': None}
            self.carga = {}
            self.llevando = {}
            self.reponer_EB(); self.reponer_RT()
        if mover:
            for n, T in self.objetos.items():
                self.poner(n, T)
    def reponer_EB(self):
        if self.est['EB'] is None and self.botellas_libres:
            i = self.botellas_libres.pop(0)
            vacia, llena, _ = self.nombres(i)
            self.est['EB'] = {'i': i, 'visible': vacia, 'llena': llena, 'tapa': None, 'estado': 'vacia'}
            self.poner(vacia, self.T_EB)
    def reponer_RT(self):
        if self.est['RT'] is None and self.tapas_libres:
            i = self.tapas_libres.pop(0)
            tapa = self.nombres(i)[2]
            self.est['RT'] = tapa
            self.poner(tapa, self.T_RT)
    def poner(self, nombre, T):
        self.objetos[nombre] = T
        with self.pend_lock:
            self.pendientes[nombre] = T
    # un solo hilo para set_pose: varias peticiones a la vez tumban Gazebo
    def enviar_poses(self):
        while rclpy.ok():
            t0 = time.time()
            with self.pend_lock:
                lote, self.pendientes = self.pendientes, {}
            if lote and self.set_pose.service_is_ready():
                for nombre, T in lote.items():
                    req = SetEntityPose.Request()
                    req.entity = Entity(name=nombre, type=Entity.MODEL)
                    req.pose = a_pose(T)
                    fut = self.set_pose.call_async(req)
                    limite = time.time() + 0.5
                    while not fut.done() and time.time() < limite:
                        time.sleep(0.001)
            elif lote:
                with self.pend_lock:
                    for n, T in lote.items():
                        self.pendientes.setdefault(n, T)
            time.sleep(max(0.0, 1.0 / 30 - (time.time() - t0)))
    def T_herr(self, ns):
        try:
            tr = self.tf.lookup_transform('world', f'{ns}/{HERRAMIENTA[ns]}', rclpy.time.Time())
            return mat(tr.transform.translation, tr.transform.rotation)
        except Exception:
            return None
    def agarrar(self, ns, nombres):
        T = None
        for _ in range(100):
            T = self.T_herr(ns)
            if T:
                break
            time.sleep(0.02)
        if T is None:
            raise RuntimeError(f'no hay TF de {ns}/{HERRAMIENTA[ns]} (el robot no tiene controladores activos?)')
        T0 = self.objetos[nombres[0]]
        p = self.sec['agarre'][ns]['punto']
        Rt = [[T[j][i] for j in range(3)] for i in range(3)]
        Rrel = [[sum(Rt[i][k] * T0[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        Trel0 = [Rrel[0] + [p[0]], Rrel[1] + [p[1]], Rrel[2] + [p[2]], [0, 0, 0, 1]]
        grupo = [(nombres[0], Trel0)]
        T0_nueva = mul(T, Trel0)
        for n in nombres[1:]:
            grupo.append((n, mul(inv(T), mul(T0_nueva, mul(inv(T0), self.objetos[n])))))
        self.llevando[ns] = grupo
        self.seguir(ns)
    def soltar(self, ns):
        self.llevando.pop(ns, None)
    def listos(self):
        faltan = [ns for ns in CONTROL if ns not in self.js]
        faltan += [ns for ns in HERRAMIENTA if ns not in faltan and self.T_herr(ns) is None]
        return faltan
    def seguir(self, ns=None):
        for r in ([ns] if ns else list(self.llevando)):
            grupo = self.llevando.get(r)
            if not grupo:
                continue
            T = self.T_herr(r)
            if T:
                for nombre, Trel in grupo:
                    self.poner(nombre, mul(T, Trel))
    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9
    def dormir(self, dt):
        fin = self.ahora() + dt
        while self.ahora() < fin and rclpy.ok():
            time.sleep(0.002)
    def publicar(self, ns, q):
        for tipo, nombre, juntas in CONTROL[ns]:
            if tipo == 'posicion':
                m = Float64MultiArray(); m.data = [float(q[j]) for j in juntas]
                self.pubs[(ns, nombre)].publish(m)
    # a los JTC (R1, R3, R5) se les manda la trayectoria entera; un punto a 50 Hz con varios robots tumba Gazebo
    def trayectoria(self, ns, muestras):
        for tipo, nombre, juntas in CONTROL[ns]:
            if tipo != 'jtc' or not any(j in muestras[-1][1] for j in juntas):
                continue
            m = JointTrajectory(); m.joint_names = juntas
            for t, q in muestras:
                p = JointTrajectoryPoint(); p.positions = [float(q[j]) for j in juntas]
                p.time_from_start.sec = int(t); p.time_from_start.nanosec = int((t - int(t)) * 1e9)
                m.points.append(p)
            self.pubs[(ns, nombre)].publish(m)
    # eje Z del R4 en velocidad. Si toca el tope de abajo (0.105) Gazebo lo traba, por eso maximo 0.098
    def mantener(self):
        actual = self.js.get('robot4', {}).get('Joint_3')
        if actual is None:
            return
        objetivo = min(0.098, max(0.001, self.cmd['robot4'].get('Joint_3', 0.0)))
        err = objetivo - actual
        v_des = 0.0 if abs(err) < 0.0015 else max(-0.08, min(0.08, 3.0 * err))
        paso = 0.15 / HZ
        self.v_j3 = getattr(self, 'v_j3', 0.0)
        self.v_j3 += max(-paso, min(paso, v_des - self.v_j3))
        m = Float64MultiArray(); m.data = [self.v_j3]
        self.pubs[('robot4', 'joint3_velocity_controller')].publish(m)
    def mover(self, ns, destino, minimo=0.5, via=()):
        self.chequear_parada()
        q0 = dict(self.cmd[ns])
        puntos = [q0]
        for v in list(via) + [destino]:
            p = dict(puntos[-1]); p.update(v); puntos.append(p)
        juntas = list(puntos[-1])
        durs = [max((abs(b[j] - a.get(j, 0.0)) / (V_PRIS if j in PRISMATICAS else V_REV) for j in juntas),
                    default=0) for a, b in zip(puntos, puntos[1:])]
        total = sum(durs)
        dur = max(minimo, total) / self.vel
        n = max(1, int(dur * HZ))
        def postura(i):
            t = suave(i / n) * total
            k = 0
            while k < len(durs) - 1 and t > durs[k]:
                t -= durs[k]; k += 1
            s = 1.0 if durs[k] == 0 else min(1.0, t / durs[k])
            a, b = puntos[k], puntos[k + 1]
            return {j: a.get(j, 0.0) + (b[j] - a.get(j, 0.0)) * s for j in juntas}
        cada = max(1, int(HZ / 10))
        self.trayectoria(ns, [(dur * i / n, postura(i)) for i in list(range(cada, n, cada)) + [n]])
        for i in range(1, n + 1):
            q = postura(i)
            self.cmd[ns] = q
            self.publicar(ns, q)
            self.seguir(ns)
            self.dormir(1.0 / HZ)
        t0, avisado, reenvio = self.ahora(), False, self.ahora()
        while True:
            self.publicar(ns, self.cmd[ns]); self.seguir(ns); self.dormir(1.0 / HZ)
            if self.ahora() - reenvio > 2.0:
                self.trayectoria(ns, [(0.5, self.cmd[ns])]); reenvio = self.ahora()
            js = self.js.get(ns, {})
            lejos = {j: round(js[j] - q, 3) for j, q in self.cmd[ns].items()
                     if j in js and not j.startswith('garra')
                     and abs(js[j] - q) >= (0.004 if j in PRISMATICAS else 0.03)}
            if not lejos and self.ahora() - t0 > 0.15:
                break
            if self.ahora() - t0 > 8.0 and not avisado:
                self.get_logger().warn(f'{ns}: todavia no llega despues de 8 s (real - comando: {lejos})')
                avisado = True
            if self.ahora() - t0 > 20.0:
                raise RuntimeError(f'{ns} no llega a su posicion en 20 s (trabado?): real - comando = {lejos}')
    def roscar(self, z_final):
        self.estado('4 SELLADO - R4: roscando la tapa')
        grupo = self.llevando['robot4']
        z0 = self.cmd['robot4']['Joint_3']
        n = max(1, int(ROSCA_T / self.vel * HZ))
        for i in range(1, n + 1):
            s = i / n
            self.cmd['robot4']['Joint_3'] = z0 + (z_final - z0) * s
            ang = -2 * math.pi * ROSCA_VUELTAS * s
            c, sn = math.cos(ang), math.sin(ang)
            giro = [[c, -sn, 0, 0], [sn, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            self.llevando['robot4'] = [(nombre, mul(Trel, giro)) for nombre, Trel in grupo]
            self.seguir('robot4')
            self.dormir(1.0 / HZ)
        self.mover('robot4', {'Joint_3': z_final})
    def pausa(self, s, ns=None):
        t = self.ahora()
        while self.ahora() - t < s / self.vel:
            self.seguir(ns); self.dormir(1.0 / HZ)
    def chocan(self, a, b):
        return frozenset((a, b)) in self.conflictos
    # semaforo: un tramo arranca solo si ningun otro robot esta en uno incompatible (config/conflictos.json)
    def entrar(self, ns, tramo):
        mio = f'{ns}:{tramo}'
        self.cerrar(ns)
        with self.cv:
            while True:
                self.chequear_parada()
                otro = next((o for r, o in self.ocupacion.items() if r != ns and self.chocan(mio, o)), None)
                if otro is None:
                    break
                if self.esperando.get(ns) != otro:
                    self.esperando[ns] = otro
                    self.get_logger().info(f'{ns}: espera para "{tramo}" porque {otro.replace(":", " esta en ")}')
                self.cv.wait(0.2)
            self.esperando.pop(ns, None)
            self.ocupacion[ns] = mio
            self.cv.notify_all()
        self.ordenar(ns, tramo)
    def quieto(self, ns, postura):
        self.cerrar(ns)
        with self.cv:
            self.ocupacion[ns] = f'{ns}:{postura}'
            self.cv.notify_all()
    def esperar(self, condicion):
        with self.cv:
            while not condicion():
                self.chequear_parada()
                self.cv.wait(0.2)
    def chequear_parada(self):
        if getattr(self, 'abortar', False):
            raise Parada()
    def hacer(self, ns, tramo, al_agarrar=None, al_soltar=None):
        paso = self.rutas[ns]
        for k in TRAMOS[tramo]:
            nombre, q, ev, *via = paso['keyframes'][k]
            via = via[0] if via else []
            if nombre == 'volver a HOME' and ns == 'robot3':
                self.mover('robot3', {'garra1_joint': 0.0, 'garra2_joint': 0.0})
            if ns == 'robot3' and nombre == 'bajar' and ns not in self.llevando:
                self.mover('robot3', {'garra1_joint': GARRA['abrir'], 'garra2_joint': GARRA['abrir']})
            if ns == 'robot4' and nombre == 'bajar' and ns in self.llevando:
                arriba = dict(q); arriba['Joint_3'] = q['Joint_3'] - ROSCA_ALTO
                self.mover(ns, arriba)
                self.roscar(q['Joint_3'])
                continue
            self.mover(ns, q, via=via)
            if ev and ev[0] == 'agarrar':
                if ns == 'robot3':
                    self.mover('robot3', {'garra1_joint': GARRA['cerrar'], 'garra2_joint': GARRA['cerrar']})
                al_agarrar()
                self.pausa(0.3, ns)
            if ev and ev[0] == 'soltar':
                self.soltar(ns)
                if ns == 'robot3':
                    self.mover('robot3', {'garra1_joint': GARRA['abrir'], 'garra2_joint': GARRA['abrir']})
                al_soltar()
                self.pausa(0.3, ns)
    def en_camino_a(self, punto):
        hasta_pll = self.carga.get('robot2') is not None
        if punto == 'PLL':
            return hasta_pll or self.est['PLL'] is not None
        hasta_ps = hasta_pll or self.est['PLL'] is not None or self.carga.get('robot3') is not None
        if punto == 'PS':
            return hasta_ps or self.est['PS'] is not None
        return False
    def falta_tapa(self):
        return (self.carga.get('robot2') is not None or self.est['PLL'] is not None
                or self.carga.get('robot3') is not None
                or (self.est['PS'] is not None and self.est['PS']['estado'] == 'sin_tapa'))
    def linea(self, limite):
        with self.lock:
            self.ocupado = True
            self.parar, self.abortar = False, False
            self.limite, self.tomadas = limite, 0
            self.terminadas, self.t_inicio, self.t_inicio_pc = [], self.ahora(), time.time()
            self.rutas = {p['robot']: p for p in self.sec['plan']}
            self.ocupacion = {ns: f'{ns}:HOME' for ns in CONTROL}
            self.esperando = {}
            self.pendiente = {}
            self.reiniciar_objetos()
            for ns in ORDEN_IR_HOME:
                if any(abs(self.cmd[ns].get(j, 0.0) - v) > 1e-3 for j, v in self.home[ns].items()):
                    self.mover(ns, self.home[ns], minimo=1.0)
            self.estado('LINEA: en marcha' + (' (una botella)' if limite else ''))
            hilos = [threading.Thread(target=self.hilo, args=(ns, f), daemon=True)
                     for ns, f in (('robot1', self.ciclo_r1), ('robot2', self.ciclo_r2), ('robot3', self.ciclo_r3),
                                   ('robot4', self.ciclo_r4), ('robot5', self.ciclo_r5))]
            for h in hilos:
                h.start()
            for h in hilos:
                h.join()
            self.estado('FIN: linea detenida | ' + self.texto_produccion())
            self.ocupado = False
    def hilo(self, ns, ciclo):
        try:
            ciclo()
        except Parada:
            pass
        except Exception as e:
            self.estado(f'ERROR en {ns}: {e} -> se para la linea')
            with self.cv:
                self.parar = True
                self.abortar = True
                self.cv.notify_all()
    def ciclo_r2(self):
        ns = 'robot2'
        while True:
            self.esperar(lambda: self.parar or self.est['EB'] is not None)
            if self.parar:
                return
            with self.cv:
                self.tomadas += 1
                if self.limite and self.tomadas >= self.limite:
                    self.parar = True
            self.entrar(ns, 'ir')
            self.estado('2 ENFRIADO Y DOSIFICACION - R2: toma un envase vacio en EB')
            def tomar():
                with self.cv:
                    b = self.est['EB']; self.est['EB'] = None; self.carga[ns] = b
                self.agarrar(ns, [b['visible']])
            self.hacer(ns, 'ir', al_agarrar=tomar)
            self.quieto(ns, 'arriba_recoger')
            self.esperar(lambda: self.est['PLL'] is None)
            self.entrar(ns, 'llevar')
            self.estado('2 ENFRIADO Y DOSIFICACION - R2: lleva el envase a PLL')
            def dejar():
                with self.cv:
                    self.est['PLL'] = self.carga.pop(ns)
                    self.reponer_EB()
                    self.cv.notify_all()
            self.hacer(ns, 'llevar', al_soltar=dejar)
            self.entrar(ns, 'volver')
            self.hacer(ns, 'volver')
            self.quieto(ns, 'HOME')
    def ciclo_r1(self):
        ns, r1 = 'robot1', self.sec['robot1']
        en_ri = False
        while True:
            self.esperar(lambda: not self.parar or self.en_camino_a('PLL'))
            if self.parar and not self.en_camino_a('PLL'):
                break
            if not en_ri:
                self.entrar(ns, 'salir')
                self.mover(ns, r1['RI_arriba'])
                self.quieto(ns, 'RI_arriba'); en_ri = True
            self.entrar(ns, 'cargar')
            self.estado('1 PRECALENTADO - R1: carga aceite caliente en RI')
            self.mover(ns, r1['RI_dentro']); self.pausa(T_CARGAR, ns); self.mover(ns, r1['RI_arriba'])
            self.quieto(ns, 'RI_arriba')
            self.esperar(lambda: (self.est['PLL'] is not None and self.est['PLL']['estado'] == 'vacia')
                         or (self.parar and not self.en_camino_a('PLL')))
            if self.est['PLL'] is None or self.est['PLL']['estado'] != 'vacia':
                break
            self.entrar(ns, 'verter')
            self.estado('1 PRECALENTADO - R1: vierte el aceite caliente en la botella en PLL')
            self.mover(ns, r1['PLL_arriba']); self.mover(ns, r1['PLL_verter'])
            self.mover(ns, {'joint_4': self.sec.get('robot1_verter', 0.87)})
            self.pausa(T_VERTER / 2, ns)
            with self.cv:
                b = self.est['PLL']
                self.poner(b['llena'], self.objetos[b['visible']])
                self.poner(b['visible'], self.escondido(b['visible']))
                b['visible'], b['estado'] = b['llena'], 'llena'
            self.pausa(T_VERTER / 2, ns)
            self.mover(ns, {'joint_4': 0.0}); self.mover(ns, r1['PLL_arriba'])
            with self.cv:
                self.cv.notify_all()
            self.mover(ns, r1['RI_arriba'])
            self.quieto(ns, 'RI_arriba')
        if en_ri:
            self.entrar(ns, 'volver')
            self.mover(ns, self.home[ns])
        self.quieto(ns, 'HOME')
    def ciclo_r3(self):
        ns = 'robot3'
        while True:
            self.esperar(lambda: (self.est['PLL'] is not None and self.est['PLL']['estado'] == 'llena'
                                  and self.est['PS'] is None and self.ocupacion['robot1'] != 'robot1:verter')
                         or (self.parar and not self.en_camino_a('PLL')))
            if self.est['PLL'] is None or self.est['PLL']['estado'] != 'llena':
                return
            self.entrar(ns, 'ir')
            self.estado('3 TRANSPORTE A SELLADO - R3: toma la botella llena en PLL')
            def tomar():
                with self.cv:
                    b = self.est['PLL']; self.est['PLL'] = None; self.carga[ns] = b
                    self.cv.notify_all()
                self.agarrar(ns, [b['visible']])
            self.hacer(ns, 'ir', al_agarrar=tomar)
            self.quieto(ns, 'arriba_recoger')
            self.esperar(lambda: self.est['PS'] is None)
            self.entrar(ns, 'llevar')
            self.estado('3 TRANSPORTE A SELLADO - R3: lleva la botella a PS')
            def dejar():
                with self.cv:
                    self.est['PS'] = self.carga.pop(ns); self.est['PS']['estado'] = 'sin_tapa'
                    self.cv.notify_all()
            self.hacer(ns, 'llevar', al_soltar=dejar)
            self.entrar(ns, 'volver')
            self.hacer(ns, 'volver')
            self.quieto(ns, 'HOME')
    def ciclo_r4(self):
        ns = 'robot4'
        while True:
            self.esperar(lambda: (self.falta_tapa() and not (self.est['PS'] is not None
                                                            and self.est['PS']['estado'] == 'tapada'))
                         or (self.parar and not self.en_camino_a('PS')))
            if not self.falta_tapa():
                return
            self.entrar(ns, 'ir')
            self.estado('4 SELLADO - R4: toma una tapa en RT')
            def tomar():
                with self.cv:
                    t = self.est['RT']; self.est['RT'] = None; self.carga[ns] = t
                self.agarrar(ns, [t])
            self.hacer(ns, 'ir', al_agarrar=tomar)
            with self.cv:
                self.reponer_RT()
            self.quieto(ns, 'arriba_recoger')
            self.esperar(lambda: self.est['PS'] is not None and self.est['PS']['estado'] == 'sin_tapa')
            self.entrar(ns, 'llevar')
            self.estado('4 SELLADO - R4: lleva la tapa a PS')
            def dejar():
                with self.cv:
                    b = self.est['PS']; b['tapa'] = self.carga.pop(ns); b['estado'] = 'tapada'
                    self.cv.notify_all()
            self.hacer(ns, 'llevar', al_soltar=dejar)
            self.entrar(ns, 'volver')
            self.hacer(ns, 'volver')
            self.quieto(ns, 'HOME')
    def ciclo_r5(self):
        ns = 'robot5'
        while True:
            self.esperar(lambda: (self.est['PS'] is not None and self.est['PS']['estado'] == 'tapada')
                         or (self.parar and not self.en_camino_a('PS')))
            if self.est['PS'] is None or self.est['PS']['estado'] != 'tapada':
                return
            self.entrar(ns, 'ir')
            self.estado('5 EMPAQUE - R5: toma la botella sellada en PS')
            def tomar():
                with self.cv:
                    b = self.est['PS']; self.est['PS'] = None; self.carga[ns] = b
                    self.cv.notify_all()
                self.agarrar(ns, [b['visible'], b['tapa']])
            self.hacer(ns, 'ir', al_agarrar=tomar)
            self.quieto(ns, 'arriba_recoger')
            self.entrar(ns, 'llevar')
            self.estado('5 EMPAQUE - R5: lleva la botella a PF')
            def dejar():
                with self.cv:
                    b = self.carga.pop(ns)
                    for n in (b['visible'], b['tapa']):
                        self.poner(n, self.escondido(n))
                    self.botellas_libres.append(b['i'])
                    self.tapas_libres.append(int(b['tapa'].split('_')[-1]) if b['tapa'][-1].isdigit() else 0)
                    self.terminadas.append(self.ahora())
                    self.reponer_RT()
                    self.cv.notify_all()
                txt = self.texto_produccion()
                self.estado('PRODUCCION: ' + txt)
                self.pub_prod.publish(String(data=txt))
            self.hacer(ns, 'llevar', al_soltar=dejar)
            self.entrar(ns, 'volver')
            self.hacer(ns, 'volver')
            self.quieto(ns, 'HOME')
    def texto_produccion(self):
        t = getattr(self, 'terminadas', [])
        if not t:
            return '0 botellas'
        minutos = (t[-1] - self.t_inicio) / 60.0
        txt = f'{len(t)} botellas en {minutos:.2f} min = {len(t) / minutos:.2f} bot/min desde START'
        if len(t) >= 2:
            ciclo = statistics.median(b - a for a, b in zip(t, t[1:]))
            txt += f' | ritmo de la linea: 1 botella cada {ciclo:.1f} s = {60.0 / ciclo:.2f} bot/min'
        pc = time.time() - self.t_inicio_pc
        if pc > 1:
            txt += f' (tiempo simulado; la PC simulo a {(self.ahora() - self.t_inicio) / pc:.2f}x tiempo real)'
        return txt
    def reset(self):
        self.ir_home()
        self.reiniciar_objetos()
    def _a_home(self):
        self.llevando = {}
        self.abortar = False
        for ns in ORDEN_IR_HOME:
            q = self.home.get(ns)
            if not q or ns not in CONTROL:
                continue
            if ns not in self.js:
                self.estado(f'HOME: {ns} sin /joint_states (no cargado o sin controladores), se salta')
                continue
            self.estado(f'HOME: {ns}')
            self.mover(ns, q, minimo=1.0)
        self.estado('HOME: listo')
    def ir_home(self):
        with self.lock:
            self.ocupado = True
            try:
                self._a_home()
            except Exception as e:
                self.estado(f'ERROR: {e}')
            finally:
                self.ocupado = False


def main():
    rclpy.init()
    nodo = Coordinador()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(nodo)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
