import importlib.util, math, os, re, struct, json
import xml.etree.ElementTree as ET
from math import cos, sin, radians
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory as share
from scipy.spatial import ConvexHull
from scipy.optimize import linprog

SH = share('celda_aceite')
_spec = importlib.util.spec_from_file_location('celda_launch', os.path.join(SH, 'launch', 'celda.launch.py'))
CL = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(CL)
CFG = yaml.safe_load(open(os.path.join(SH, 'config', 'robots.yaml')))['robots']
SEC = json.load(open(os.path.join(SH, 'config', 'secuencia.json')))


def tf_xml(o):
    if o is None:
        return np.eye(4)
    xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
    r, p, y = [float(v) for v in o.get('rpy', '0 0 0').split()]
    cr, sr, cp, sp, cy, sy = cos(r), sin(r), cos(p), sin(p), cos(y), sin(y)
    T = np.eye(4)
    T[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    T[:3, 3] = xyz
    return T


def eje(ax, q, tipo):
    ax = np.array(ax, float); ax /= np.linalg.norm(ax)
    T = np.eye(4)
    if tipo == 'prismatic':
        T[:3, 3] = ax * q
        return T
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    T[:3, :3] = np.eye(3) + sin(q) * K + (1 - cos(q)) * K @ K
    return T


def leer_malla(fn):
    if fn.startswith('package://'):
        pkg, rest = fn[10:].split('/', 1)
        fn = os.path.join(share(pkg), rest)
    fn = fn.replace('file://', '')
    d = open(fn, 'rb').read()
    if fn.lower().endswith('.dae'):
        m = re.search(rb'<float_array id="[^"]*positions-array"[^>]*>([^<]*)<', d)
        return np.array(m.group(1).split(), float).reshape(-1, 3)
    if d[:5] == b'solid' and b'facet' in d[:400]:
        return np.array([list(map(float, l.split()[1:4])) for l in d.decode(errors='ignore').splitlines()
                         if l.strip().startswith('vertex')])
    n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3).astype(float)


class Robot:
    def __init__(self, ns):
        r = CFG[ns]
        xml = CL._leer_descripcion(os.path.join(share(r['pkg']), r['urdf']))
        if r.get('raiz'):
            xml = CL._rerootear(xml, r['raiz'])
        root = ET.fromstring(xml)
        yaw = radians(r.get('yaw_deg', 0.0)); bx, by = r.get('base_cm', [0.0, 0.0])
        x = (r['x_cm'] - (cos(yaw) * bx - sin(yaw) * by)) / 100.0
        y = (r['y_cm'] - (sin(yaw) * bx + cos(yaw) * by)) / 100.0
        self.T0 = eje([0, 0, 1], yaw, 'revolute'); self.T0[:3, 3] = [x, y, r.get('z_cm', 0.0) / 100.0]
        self.por_hijo = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.piezas = []
        for l in root.findall('link'):
            for col in (l.findall('collision') or l.findall('visual')):
                g = col.find('geometry/mesh')
                if g is None:
                    continue
                sc = np.array([float(s) for s in g.get('scale', '1 1 1').split()])
                V = leer_malla(g.get('filename')) * sc
                V = V[ConvexHull(V).vertices]
                self.piezas.append((l.get('name'), (tf_xml(col.find('origin')) @ np.c_[V, np.ones(len(V))].T).T))
    def T_link(self, link, q):
        def W(l):
            if l not in self.por_hijo:
                return self.T0
            j = self.por_hijo[l]
            T = W(j.find('parent').get('link')) @ tf_xml(j.find('origin'))
            if j.get('type') in ('revolute', 'continuous', 'prismatic'):
                ax = [float(v) for v in j.find('axis').get('xyz').split()] if j.find('axis') is not None else [1, 0, 0]
                T = T @ eje(ax, q.get(j.get('name'), 0.0), j.get('type'))
            return T
        return W(link)
    def links(self, q):
        out = []
        cache = {}
        for name, P in self.piezas:
            if name not in cache:
                cache[name] = self.T_link(name, q)
            Wp = (cache[name] @ P.T).T[:, :3]
            out.append((name, Wp, Wp.min(0), Wp.max(0)))
        return out


ROB = {ns: Robot(ns) for ns in CFG}


def separacion(A, B):
    E = np.vstack([ConvexHull(A).equations, ConvexHull(B).equations])
    M = np.c_[E[:, :3], np.linalg.norm(E[:, :3], axis=1)]
    r = linprog([0, 0, 0, -1], A_ub=M, b_ub=-E[:, 3], bounds=[(None, None)] * 3 + [(None, 0.1)])
    return -r.fun if r.status == 0 else -1.0


def choques(conf, umbral=0.002, solo=None, margen_bbox=0.03):
    solo = [solo] if isinstance(solo, str) else solo
    mundo = []
    for ns, rb in ROB.items():
        if ns in conf:
            for name, Wp, lo, hi in rb.links(conf[ns]):
                mundo.append((ns, name, Wp, lo - margen_bbox, hi + margen_bbox))
    res = []
    for i in range(len(mundo)):
        for j in range(i + 1, len(mundo)):
            a, b = mundo[i], mundo[j]
            if a[0] == b[0] or (solo and a[0] not in solo and b[0] not in solo):
                continue
            if np.any(a[3] > b[4]) or np.any(b[3] > a[4]):
                continue
            s = separacion(a[2], b[2])
            if s > umbral:
                res.append((s, a[0], a[1], b[0], b[1]))
    return sorted(res, reverse=True)


def home():
    return {ns: {j: float(v) for j, v in (CFG[ns].get('home') or {}).items()} for ns in CFG}
