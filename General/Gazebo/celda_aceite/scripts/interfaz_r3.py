#!/usr/bin/env python3
import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JUNTAS = [('eslabon1_joint', 'Eslabon 1', 'rad', -2.6, 2.6), ('eslabon2_joint', 'Eslabon 2', 'rad', -1.95, 1.95),
          ('prismatico_joint', 'Eje Z', 'm', 0.001, 0.09), ('garra1_joint', 'Garra', 'rad', 0.0, 0.45)]


class InterfazR3(Node):
    def __init__(self):
        super().__init__('interfaz_r3')
        self.declare_parameter('puerto_web', 8080)
        cfg = os.path.join(get_package_share_directory('celda_aceite'), 'config')
        home = yaml.safe_load(open(os.path.join(cfg, 'robots.yaml')))['robots']['robot3']['home']
        kf = [p for p in json.load(open(os.path.join(cfg, 'secuencia.json')))['plan'] if p['robot'] == 'robot3'][0]
        kf = {k[0]: k[1] for k in kf['keyframes']}
        self.posturas = {'HOME': [home['eslabon1_joint'], home['eslabon2_joint'], 0.0, 0.0],
                         'PLL': [kf['ir a recoger']['eslabon1_joint'], kf['ir a recoger']['eslabon2_joint'], 0.0, 0.0],
                         'PS': [kf['llevar']['eslabon1_joint'], kf['llevar']['eslabon2_joint'], 0.0, 0.0]}
        self.lock = threading.Lock()
        self.banderas, self.estados = [], []
        self.gazebo, self.real, self.t_real = {}, {}, 0.0
        self.tramo, self.linea, self.esp = '-', 'detenida', False
        self.rviz = None
        self.create_subscription(String, '/celda/banderas', self.on_bandera, 50)
        self.create_subscription(String, '/celda/estado', self.on_estado, 50)
        self.create_subscription(JointState, '/robot3/joint_states',
                                 lambda m: self.guardar(self.gazebo, m), 10)
        self.create_subscription(JointState, '/robot3_real/joint_states', self.on_real, 10)
        self.pub = self.create_publisher(String, '/celda/comando', 10)
    def guardar(self, d, m):
        with self.lock:
            d.update(zip(m.name, m.position))
    def on_real(self, m):
        self.guardar(self.real, m)
        self.t_real = time.time()
    def on_bandera(self, m):
        if 'R3 ' not in m.data:
            return
        with self.lock:
            self.banderas.append((time.strftime('%H:%M:%S'), m.data))
            self.banderas = self.banderas[-40:]
            p = m.data.split()
            if 'HOLA' in p:
                self.esp = True
            if len(p) >= 4 and p[2] in ('HAZ', 'MUEVE'):
                self.tramo = 'MUEVE (a mano)' if p[2] == 'MUEVE' else f'{p[3]} (haciendolo)'
            if len(p) >= 4 and p[2] == 'LISTO':
                self.tramo = f'{p[3]} (LISTO)'
    def on_estado(self, m):
        t = m.data
        with self.lock:
            if 'R3' in t or 'robot3' in t or t.startswith(('LINEA', 'FIN', 'ERROR', 'MANUAL', 'START', 'REAL',
                                                              'STOP', 'UNA', 'HOME', 'PRODUCCION')):
                self.estados.append((time.strftime('%H:%M:%S'), t))
                self.estados = self.estados[-15:]
            if t.startswith('LINEA: en marcha'):
                self.linea = 'en marcha'
            elif t.startswith(('FIN', 'ERROR')):
                self.linea = 'detenida'
    def mandar(self, texto):
        self.pub.publish(String(data=texto))
    def abrir_rviz(self):
        if self.rviz and self.rviz.poll() is None:
            return 'RViz ya esta abierto'
        self.rviz = subprocess.Popen(['ros2', 'launch', 'celda_aceite', 'rviz_r3.launch.py'])
        return 'abriendo RViz...'
    def json_estado(self):
        with self.lock:
            filas = []
            for j, nombre, unidad, lo, hi in JUNTAS:
                g, r = self.gazebo.get(j), self.real.get(j)
                filas.append({'junta': j, 'nombre': nombre, 'unidad': unidad, 'min': lo, 'max': hi,
                              'gazebo': g, 'real': r if time.time() - self.t_real < 3 else None})
            return {'filas': filas, 'banderas': self.banderas[::-1], 'estados': self.estados[::-1],
                    'tramo': self.tramo, 'linea': self.linea,
                    'encoders': time.time() - self.t_real < 3, 'esp': self.esp, 'posturas': self.posturas}


PAGINA = """<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Robot 3</title>
<style>
:root{--f:#f6f7f9;--c:#fff;--t:#1d2430;--s:#5b6575;--b:#dfe3ea;--a:#2563eb;--ok:#16a34a;--no:#b91c1c}
@media(prefers-color-scheme:dark){:root{--f:#12151b;--c:#1b2029;--t:#e6e9ef;--s:#9aa3b2;--b:#2c3340;--a:#60a5fa;--ok:#4ade80;--no:#f87171}}
*{box-sizing:border-box}body{margin:0;font:15px system-ui,sans-serif;background:var(--f);color:var(--t)}
header{padding:14px 18px;border-bottom:1px solid var(--b);display:flex;gap:10px;flex-wrap:wrap;align-items:center}
h1{font-size:18px;margin:0 12px 0 0}.chip{padding:3px 10px;border-radius:99px;border:1px solid var(--b);font-size:13px}
.ok{color:var(--ok);border-color:var(--ok)}.no{color:var(--no);border-color:var(--no)}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px;padding:14px}
section{background:var(--c);border:1px solid var(--b);border-radius:10px;padding:14px}h2{font-size:15px;margin:0 0 10px}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}td,th{padding:5px 4px;border-bottom:1px solid var(--b);text-align:right}
td:first-child,th:first-child{text-align:left}.mal{color:var(--no);font-weight:600}
.log{font:13px ui-monospace,monospace;max-height:300px;overflow:auto}.log div{padding:2px 0;border-bottom:1px solid var(--b)}
.s{color:var(--s)}label{display:block;margin:8px 0 2px}input[type=range]{width:100%}
button{padding:8px 12px;border-radius:7px;border:1px solid var(--b);background:var(--c);color:var(--t);cursor:pointer;margin:6px 6px 0 0}
button.p{background:var(--a);color:#fff;border-color:var(--a)}button:disabled{opacity:.45;cursor:not-allowed}
#msg{margin-top:8px;color:var(--s);min-height:1.2em}
</style></head><body>
<header><h1>Robot 3</h1><span id="esp" class="chip">ESP32</span><span id="enc" class="chip">encoders</span>
<span id="lin" class="chip">linea</span><span class="chip">tramo: <b id="tra">-</b></span></header>
<main>
<section><h2>Angulos: Gazebo vs encoders</h2><table><thead><tr><th>Junta</th><th>Gazebo</th><th>Real</th><th>Dif.</th></tr></thead>
<tbody id="ang"></tbody></table><p class="s">Grados en las juntas rotativas, mm en el eje Z. En rojo, diferencias de mas de 3 grados o 3 mm.</p></section>
<section><h2>Mover a mano</h2><div id="sl"></div>
<div><button data-p="HOME">HOME</button><button data-p="PLL">PLL</button><button data-p="PS">PS</button></div>
<button class="p" id="mov">Mover R3</button><button id="rviz">Abrir RViz</button><div id="msg"></div>
<p class="s">Solo con la linea detenida (STOP). Mueve el R3 de Gazebo y el real a la vez.</p></section>
<section><h2>Banderas PC &lt;-&gt; ESP32</h2><div id="ban" class="log"></div></section>
<section><h2>Coordinador</h2><div id="est" class="log"></div></section>
</main>
<script>
const $=id=>document.getElementById(id);let ini=false,linea='detenida';
const fmt=(v,u)=>v==null?'—':(u=='m'?(v*1000).toFixed(1)+' mm':(v*180/Math.PI).toFixed(1)+'°');
function sliders(f){$('sl').innerHTML=f.map(r=>`<label>${r.nombre}: <b id="v_${r.junta}"></b></label>
<input type=range id="s_${r.junta}" min=${r.min} max=${r.max} step=0.001 value=${r.gazebo??0}>`).join('');
f.forEach(r=>{const s=$('s_'+r.junta);const up=()=>$('v_'+r.junta).textContent=fmt(+s.value,r.unidad);s.oninput=up;up()});}
function poner(v,f){f.forEach((r,i)=>{$('s_'+r.junta).value=v[i];$('s_'+r.junta).oninput()})}
async function tick(){try{const e=await (await fetch('/api/estado')).json();
if(!ini){sliders(e.filas);document.querySelectorAll('[data-p]').forEach(b=>b.onclick=()=>poner(e.posturas[b.dataset.p],e.filas));ini=true}
$('ang').innerHTML=e.filas.map(r=>{const d=(r.gazebo!=null&&r.real!=null)?r.real-r.gazebo:null;
const mal=d!=null&&(r.unidad=='m'?Math.abs(d)>0.003:Math.abs(d)>0.0524);
return `<tr><td>${r.nombre}</td><td>${fmt(r.gazebo,r.unidad)}</td><td>${fmt(r.real,r.unidad)}</td><td class="${mal?'mal':''}">${fmt(d,r.unidad)}</td></tr>`}).join('');
$('ban').innerHTML=e.banderas.map(b=>`<div><span class=s>${b[0]}</span> ${b[1]}</div>`).join('')||'<span class=s>sin banderas todavia</span>';
$('est').innerHTML=e.estados.map(b=>`<div><span class=s>${b[0]}</span> ${b[1]}</div>`).join('');
$('esp').className='chip '+(e.esp?'ok':'no');$('esp').textContent=e.esp?'ESP32 conectado':'ESP32 sin conectar';
$('enc').className='chip '+(e.encoders?'ok':'no');$('enc').textContent=e.encoders?'encoders en vivo':'sin encoders';
linea=e.linea;$('lin').textContent='linea '+e.linea;$('lin').className='chip '+(e.linea=='detenida'?'':'ok');
$('tra').textContent=e.tramo;$('mov').disabled=e.linea!='detenida';}catch(x){}}
async function post(u,b){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});return (await r.json()).msg}
$('mov').onclick=async()=>{const v=[...document.querySelectorAll('#sl input')].map(s=>(+s.value).toFixed(4));
$('msg').textContent=await post('/api/comando',{texto:'MANUAL R3 '+v.join(' ')})};
$('rviz').onclick=async()=>$('msg').textContent=await post('/api/rviz');
setInterval(tick,300);tick();
</script></body></html>"""


def servidor(nodo, puerto):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass
        def responder(self, cuerpo, tipo='application/json'):
            datos = cuerpo.encode()
            self.send_response(200)
            self.send_header('Content-Type', tipo + '; charset=utf-8')
            self.send_header('Content-Length', str(len(datos)))
            self.end_headers()
            self.wfile.write(datos)
        def do_GET(self):
            if self.path.startswith('/api/estado'):
                self.responder(json.dumps(nodo.json_estado()))
            else:
                self.responder(PAGINA, 'text/html')
        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0))
            cuerpo = json.loads(self.rfile.read(n) or b'{}')
            if self.path == '/api/comando':
                texto = str(cuerpo.get('texto', ''))[:200]
                nodo.mandar(texto)
                self.responder(json.dumps({'msg': f'enviado: {texto}'}))
            elif self.path == '/api/rviz':
                self.responder(json.dumps({'msg': nodo.abrir_rviz()}))
            else:
                self.responder(json.dumps({'msg': 'ruta desconocida'}))
    ThreadingHTTPServer(('0.0.0.0', puerto), H).serve_forever()


def main():
    rclpy.init()
    nodo = InterfazR3()
    puerto = int(nodo.get_parameter('puerto_web').value)
    threading.Thread(target=servidor, args=(nodo, puerto), daemon=True).start()
    nodo.get_logger().info(f'Pagina del R3 en http://localhost:{puerto} (o http://<IP del PC>:{puerto} en la red)')
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
