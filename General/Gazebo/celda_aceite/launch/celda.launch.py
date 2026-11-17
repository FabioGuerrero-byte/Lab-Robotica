import os
import re
import subprocess
import tempfile
from math import cos, radians, sin

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'celda_aceite'
WORLD = 'celda_aceite'
# con la ganancia por defecto (0.1) los robots tardaban mucho en llegar
GANANCIA_POSICION = 0.3


def _leer_descripcion(ruta):
    if ruta.endswith('.xacro'):
        import xacro
        return xacro.process_file(ruta).toxml()
    with open(ruta, 'r') as f:
        return f.read()


def _yaml_con_namespace(origen, ns, carpeta):
    datos = yaml.safe_load(open(origen)) or {}
    salida = {(k if k.startswith('/') else f'/{ns}/{k}'): v for k, v in datos.items()}
    salida.setdefault('/**/gz_ros_control', {}).setdefault('ros__parameters', {})[
        'position_proportional_gain'] = GANANCIA_POSICION
    destino = os.path.join(carpeta, f'{ns}_controladores.yaml')
    with open(destino, 'w') as f:
        yaml.safe_dump(salida, f)
    return destino


def _parchar_plugin(xml, ns, yaml_ns):
    nuevo = f'<ros><namespace>/{ns}</namespace></ros><parameters>{yaml_ns}</parameters>'
    xml, n = re.subn(r'<parameters>.*?</parameters>', lambda _: nuevo, xml, flags=re.S)
    if n != 1:
        raise RuntimeError(f'{ns}: se esperaba 1 <parameters> del plugin gz_ros2_control y hay {n}')
    return xml


import xml.etree.ElementTree as ET
from math import atan2, hypot

import numpy as np


def _rr_tf(el):
    T = np.eye(4)
    if el is None:
        return T
    x, y, z = (float(v) for v in el.get('xyz', '0 0 0').split())
    r, p, w = (float(v) for v in el.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cw, sw = cos(r), sin(r), cos(p), sin(p), cos(w), sin(w)
    T[:3, :3] = [[cw*cp, cw*sp*sr - sw*cr, cw*sp*cr + sw*sr],
                 [sw*cp, sw*sp*sr + cw*cr, sw*sp*cr - cw*sr],
                 [-sp, cp*sr, cp*cr]]
    T[:3, 3] = [x, y, z]
    return T


def _rr_poner(el, T):
    R = T[:3, :3]
    p = atan2(-R[2, 0], hypot(R[0, 0], R[1, 0]))
    if abs(cos(p)) > 1e-9:
        r, w = atan2(R[2, 1], R[2, 2]), atan2(R[1, 0], R[0, 0])
    else:
        r, w = atan2(-R[1, 2], R[1, 1]), 0.0
    el.set('xyz', ' '.join(f'{v:.9g}' for v in T[:3, 3]))
    el.set('rpy', f'{r:.9g} {p:.9g} {w:.9g}')


def _rerootear(xml, nueva_raiz, link_mundo='world'):
    root = ET.fromstring(xml)
    joints = root.findall('joint')
    por_hijo = {j.find('child').get('link'): j for j in joints}
    def T_mundo(l):
        if l not in por_hijo:
            return np.eye(4)
        j = por_hijo[l]
        return T_mundo(j.find('parent').get('link')) @ _rr_tf(j.find('origin'))
    T_raiz = T_mundo(nueva_raiz)
    camino, l = [nueva_raiz], nueva_raiz
    while l in por_hijo:
        l = por_hijo[l].find('parent').get('link')
        camino.append(l)
    if camino[-1] != link_mundo:
        raise RuntimeError(f'la raiz del URDF es {camino[-1]}, se esperaba {link_mundo}')
    j_mundo = por_hijo[camino[-2]]
    invertidos = [por_hijo[c] for c in camino[:-2]]
    D = {}
    for j in invertidos:
        D[j.find('parent').get('link')] = _rr_tf(j.find('origin'))
    I = np.eye(4)
    for j in joints:
        if j is j_mundo:
            continue
        P, C = j.find('parent').get('link'), j.find('child').get('link')
        o = j.find('origin')
        if o is None:
            o = ET.SubElement(j, 'origin')
        if j in invertidos:
            _rr_poner(o, np.linalg.inv(D.get(C, I)))
            j.find('parent').set('link', C)
            j.find('child').set('link', P)
            ax = j.find('axis')
            if ax is None:
                ax = ET.SubElement(j, 'axis'); ax.set('xyz', '1 0 0')
            ax.set('xyz', ' '.join(f'{-float(v):.9g}' for v in ax.get('xyz').split()))
        else:
            _rr_poner(o, np.linalg.inv(D.get(P, I)) @ _rr_tf(o))
    for link in root.findall('link'):
        d = D.get(link.get('name'))
        if d is None:
            continue
        Di = np.linalg.inv(d)
        for tag in ('visual', 'collision', 'inertial'):
            for e in link.findall(tag):
                o = e.find('origin')
                if o is None:
                    o = ET.Element('origin'); e.insert(0, o)
                _rr_poner(o, Di @ _rr_tf(o))
    j_mundo.find('child').set('link', nueva_raiz)
    _rr_poner(j_mundo.find('origin') if j_mundo.find('origin') is not None
           else ET.SubElement(j_mundo, 'origin'), T_raiz)
    return ET.tostring(root, encoding='unicode')


def _sin_colisiones(xml):
    xml = re.sub(r'<collision\b[^>]*/>', '', xml)
    return re.sub(r'<collision\b.*?</collision>', '', xml, flags=re.S)


def _vel_minima(xml, vel):
    if not vel:
        return xml
    root = ET.fromstring(xml)
    for j in root.findall('joint'):
        lim = j.find('limit')
        if j.get('type') in ('revolute', 'continuous', 'prismatic') and lim is not None:
            if float(lim.get('velocity', '0')) < vel:
                lim.set('velocity', str(vel))
    return ET.tostring(root, encoding='unicode')


def _sensores_contacto(xml, ns):
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(xml)
    try:
        sdf = subprocess.run(['gz', 'sdf', '-p', f.name], capture_output=True, text=True, timeout=60).stdout
    finally:
        os.unlink(f.name)
    bloques = []
    for link, cuerpo in re.findall(r"<link name='([^']+)'>(.*?)</link>", sdf, re.S):
        for col in re.findall(r"<collision name='([^']+)'", cuerpo):
            bloques.append(
                f'<gazebo reference="{link}"><sensor name="contacto_{col}" type="contact">'
                f'<always_on>true</always_on><update_rate>30</update_rate>'
                f'<contact><collision>{col}</collision><topic>/celda/contactos</topic></contact>'
                f'</sensor></gazebo>')
    if not bloques:
        print(f'[celda_aceite] AVISO: {ns} no tiene colisiones para vigilar')
    return xml.replace('</robot>', ''.join(bloques) + '</robot>')


def _pose_inicial(xml, ns, valores):
    for j, v in (valores or {}).items():
        bloque = r'(<joint name="%s">(?:(?!</joint>).)*?)' % re.escape(j)
        xml, n = re.subn(bloque + r'(<param name="initial_value">)[^<]*(</param>)',
                         lambda m: m.group(1) + m.group(2) + str(v) + m.group(3), xml, count=1, flags=re.S)
        if n == 0:
            xml, n = re.subn(bloque + r'<state_interface name="position"\s*/>',
                             lambda m: m.group(1) + '<state_interface name="position"><param name="initial_value">'
                             + str(v) + '</param></state_interface>', xml, count=1, flags=re.S)
        if n != 1:
            raise RuntimeError(f'{ns}: no encuentro la articulacion {j} en su ros2_control')
    return xml


def _construir(context, *args, **kwargs):
    share = get_package_share_directory(PKG)
    cfg = yaml.safe_load(open(os.path.join(share, 'config', 'robots.yaml')))['robots']
    pedidos = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',') if r.strip()]
    desconocidos = [r for r in pedidos if r not in cfg]
    if desconocidos:
        raise RuntimeError(f'Robots desconocidos {desconocidos}; disponibles: {list(cfg)}')
    colisiones = LaunchConfiguration('colisiones').perform(context).lower() in ('true', '1', 'si')
    tmp = tempfile.mkdtemp(prefix='celda_aceite_')
    acciones, cadena = [], []
    for i, ns in enumerate(pedidos):
        r = cfg[ns]
        pkg_share = get_package_share_directory(r['pkg'])
        xml = _leer_descripcion(os.path.join(pkg_share, r['urdf']))
        if r.get('raiz'):
            xml = _rerootear(xml, r['raiz'])
        if not colisiones:
            xml = _sin_colisiones(xml)
        else:
            xml = _sensores_contacto(xml, ns)
        yaml_ns = _yaml_con_namespace(os.path.join(pkg_share, r['controladores_yaml']), ns, tmp)
        xml = _parchar_plugin(xml, ns, yaml_ns)
        xml = _pose_inicial(xml, ns, r.get('home'))
        xml = _vel_minima(xml, r.get('vel_minima'))
        yaw = radians(r.get('yaw_deg', 0.0))
        bx, by = r.get('base_cm', [0.0, 0.0])
        x = (r['x_cm'] - (cos(yaw) * bx - sin(yaw) * by)) / 100.0
        y = (r['y_cm'] - (sin(yaw) * bx + cos(yaw) * by)) / 100.0
        z = r.get('z_cm', 0.0) / 100.0
        rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
                   namespace=ns, output='screen',
                   parameters=[{'robot_description': ParameterValue(xml, value_type=str),
                                'frame_prefix': f'{ns}/', 'use_sim_time': True}])
        tf_estatico = Node(package='tf2_ros', executable='static_transform_publisher',
                           arguments=['--x', str(x), '--y', str(y), '--z', str(z), '--yaw', str(yaw),
                                      '--pitch', '0', '--roll', '0',
                                      '--frame-id', 'world', '--child-frame-id', f'{ns}/world'])
        spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                     arguments=['-world', WORLD, '-name', ns, '-topic', f'/{ns}/robot_description',
                                '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)])
        spawners = Node(package='controller_manager', executable='spawner', namespace=ns, output='screen',
                        arguments=r['controladores'] + ['--controller-manager', f'/{ns}/controller_manager',
                                                        '--controller-manager-timeout', '120'] + _opciones_spawner())
        acciones += [rsp, tf_estatico, spawn]
        cadena.append(spawners)
    for anterior, siguiente in zip(cadena, cadena[1:]):
        acciones.append(RegisterEventHandler(OnProcessExit(target_action=anterior, on_exit=[siguiente])))
    if cadena:
        acciones.append(cadena[0])
    return acciones


_CACHE_OPS = []


def _opciones_spawner():
    if _CACHE_OPS:
        return _CACHE_OPS[0]
    try:
        ayuda = subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', '-h'],
                               capture_output=True, text=True, timeout=30).stdout
    except Exception:
        return []
    ops = []
    if '--switch-timeout' in ayuda:
        ops += ['--switch-timeout', '60']
    if '--service-call-timeout' in ayuda:
        ops += ['--service-call-timeout', '60']
    _CACHE_OPS.append(ops)
    return ops


def generate_launch_description():
    share = get_package_share_directory(PKG)
    cfg = yaml.safe_load(open(os.path.join(share, 'config', 'robots.yaml')))['robots']
    rutas = {os.path.dirname(share), os.path.join(share, 'models')}
    for r in cfg.values():
        try:
            rutas.add(os.path.dirname(get_package_share_directory(r['pkg'])))
        except Exception:
            print(f"[celda_aceite] AVISO: no encuentro el paquete {r['pkg']} (compilaste y hiciste source?)")
    recursos = os.pathsep.join(sorted(rutas))
    mundo = os.path.join(share, 'worlds', 'celda_aceite.sdf')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                   'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 3 {mundo}', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('gui')))
    gz_sin_ventana = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                   'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-s -r -v 3 {mundo}', 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(LaunchConfiguration('gui')))
    reloj = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                            f'/world/{WORLD}/set_pose@ros_gz_interfaces/srv/SetEntityPose'])
    coordinador = Node(package='celda_aceite', executable='coordinador.py', output='screen',
                       parameters=[{'use_sim_time': True, 'reales': LaunchConfiguration('reales')}],
                       condition=IfCondition(LaunchConfiguration('coordinador')))
    interfaz = Node(package='celda_aceite', executable='interfaz_r3.py', output='screen',
                    condition=IfCondition(LaunchConfiguration('interfaz')))
    monitor = Node(package='celda_aceite', executable='monitor_choques.py', output='screen',
                   condition=IfCondition(LaunchConfiguration('colisiones')))
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='robot1,robot2,robot3,robot4,robot5',
                              description='Robots a cargar, separados por coma (ver config/robots.yaml)'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', recursos),
        # sin memoria compartida de Fast DDS: con 5 gz_ros2_control Gazebo se caia
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument('gui', default_value='true', description='false = sin ventana de Gazebo'),
        DeclareLaunchArgument('colisiones', default_value='true',
                              description='true = los robots chocan fisicamente entre si y monitor_choques.py avisa '
                                          'cada choque en el log y en /celda/choques; false = se atraviesan sin aviso'),
        DeclareLaunchArgument('coordinador', default_value='true',
                              description='true = arranca el coordinador (START por /celda/comando o UDP 5005)'),
        DeclareLaunchArgument('interfaz', default_value='true',
                              description='true = pagina web del R3 en http://<IP del PC>:8080'),
        DeclareLaunchArgument('reales', default_value='',
                              description="robots fisicos (ESP32 por WiFi) que siguen la linea: 'robot1,robot2' o "
                                          "'todos'; vacio = solo simulacion. Ver README (banderas UDP)"),
        gz, gz_sin_ventana, reloj, coordinador, monitor, interfaz,
        OpaqueFunction(function=_construir),
    ])
