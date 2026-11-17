#!/usr/bin/env python3
import os
import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
cfg = yaml.safe_load(open(os.path.join(RAIZ, 'config', 'estaciones.yaml')))


def m(cm):
    return cm / 100.0


def modelo(nombre, geom, x, y, h, color):
    r, g, b = color
    mat = f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>'
    return f'''    <model name="{nombre}">
      <static>true</static>
      <pose>{m(x)} {m(y)} {m(h)/2} 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry>{geom}</geometry></collision>
        <visual name="vis"><geometry>{geom}</geometry>{mat}</visual>
      </link>
    </model>
'''


partes = []
for nom, e in cfg['estaciones'].items():
    if e['forma'] == 'caja':
        geom = f"<box><size>{m(e['dx'])} {m(e['dy'])} {m(e['h'])}</size></box>"
    else:
        geom = f"<cylinder><radius>{m(e['diametro'])/2}</radius><length>{m(e['h'])}</length></cylinder>"
    partes.append(modelo(nom, geom, e['x'], e['y'], e['h'], e['color']))

for nom, o in (cfg.get('objetos') or {}).items():
    if 'pose_cm' in o:
        x, y, z = o['pose_cm']
    else:
        base = cfg['estaciones'][o['sobre']]
        x, y, z = base['x'], base['y'], base['h'] + o.get('encima_cm', 0.0)
    partes.append(f'''    <include>
      <uri>model://{o['modelo']}</uri>
      <name>{nom}</name>
      <pose>{m(x)} {m(y)} {m(z)} 0 0 0</pose>
    </include>
''')

for i in range(1, int(cfg.get('pool', 1))):
    for k, mod in enumerate(['botella', 'botella_llena', 'tapa']):
        partes.append(f'''    <include>
      <uri>model://{mod}</uri>
      <name>{mod}_{i}</name>
      <pose>{m(10 * i)} {m(10 * k)} -0.5 0 0 0</pose>
    </include>
''')

L = cfg['celda']['lado_cm']
borde = ''
for i, (px, py, sx, sy) in enumerate([(0, L/2, L, 1), (0, -L/2, L, 1), (L/2, 0, 1, L), (-L/2, 0, 1, L)]):
    borde += (f'        <visual name="borde{i}"><pose>{m(px)} {m(py)} 0.001 0 0 0</pose>'
              f'<geometry><box><size>{m(sx)} {m(sy)} 0.002</size></box></geometry>'
              f'<material><ambient>0.6 0.1 0.2 1</ambient><diffuse>0.6 0.1 0.2 1</diffuse></material></visual>\n')

sdf = f'''<?xml version="1.0"?>
<!-- GENERADO por tools/generar_mundo.py desde config/estaciones.yaml. No editar a mano. -->
<sdf version="1.9">
  <world name="celda_aceite">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <light type="directional" name="sol">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="suelo">
      <static>true</static>
      <link name="link">
        <collision name="col"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
        <visual name="vis"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material></visual>
{borde}      </link>
    </model>
{''.join(partes)}  </world>
</sdf>
'''
out = os.path.join(RAIZ, 'worlds', 'celda_aceite.sdf')
open(out, 'w').write(sdf)
print('escrito', out)
