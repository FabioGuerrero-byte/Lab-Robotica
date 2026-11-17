#!/usr/bin/env python3
import itertools, json, math, os, sys, time
import numpy as np
from geometria_celda import ROB, SEC, home, separacion

HOLGURA_MOVIL = 0.015
HOLGURA_QUIETO = 0.005
PASO = math.radians(2.0)
H = home()
SALIDA = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'conflictos.json')


def interpolar(q0, puntos):
    out = [dict(q0)]
    for p in puntos:
        a = out[-1]; b = dict(a); b.update(p)
        d = max(abs(b[j] - a.get(j, 0.0)) for j in b)
        n = max(1, int(d / PASO))
        for i in range(1, n + 1):
            out.append({j: a.get(j, 0.0) + (b[j] - a.get(j, 0.0)) * i / n for j in b})
    return out


def tramos():
    T, esperas = {}, {}
    r1 = SEC['robot1']; V = SEC.get('robot1_verter', 0.87)
    T[('robot1', 'salir')] = interpolar(H['robot1'], [r1['RI_arriba']])
    T[('robot1', 'cargar')] = interpolar(r1['RI_arriba'], [r1['RI_dentro'], r1['RI_arriba']])
    T[('robot1', 'verter')] = interpolar(r1['RI_arriba'], [r1['PLL_arriba'], r1['PLL_verter'], {'joint_4': V},
                                                           {'joint_4': 0.0}, r1['PLL_arriba'], r1['RI_arriba']])
    T[('robot1', 'volver')] = interpolar(r1['RI_arriba'], [H['robot1']])
    esperas[('robot1', 'RI_arriba')] = [dict(r1['RI_arriba'])]
    G3 = {0: 0.0, 1: 0.45, 2: 0.24, 3: 0.24, 4: 0.24, 5: 0.24, 6: 0.45, 7: 0.45, 8: 0.0}
    for paso in SEC['plan']:
        ns = paso['robot']
        kf = [list(x) for x in paso['keyframes']]
        if ns == 'robot3':
            for i, x in enumerate(kf):
                g = {'garra1_joint': G3[i], 'garra2_joint': G3[i]}
                x[1] = dict(x[1], **g)
                if len(x) > 3:
                    x[3] = [dict(v, **g) for v in x[3]]
        def pts(rango):
            out = []
            for nombre, q, ev, *via in [kf[i] for i in rango]:
                out += list(via[0] if via else []) + [q]
            return out
        ir = pts(range(0, 4)); llevar = pts(range(4, 8)); volver = pts([8])
        T[(ns, 'ir')] = interpolar(H[ns], ir)
        q_arriba = T[(ns, 'ir')][-1]
        T[(ns, 'llevar')] = interpolar(q_arriba, llevar)
        T[(ns, 'volver')] = interpolar(T[(ns, 'llevar')][-1], volver)
        esperas[(ns, 'arriba_recoger')] = [dict(q_arriba)]
    for ns in H:
        esperas[(ns, 'HOME')] = [dict(H[ns])]
    return T, esperas


def geometria(ns, qs):
    G = [ROB[ns].links(q) for q in qs]
    lo = np.min([l[2] for g in G for l in g], axis=0); hi = np.max([l[3] for g in G for l in g], axis=0)
    return G, lo, hi


def chocan(A, B, m):
    GA, loA, hiA = A; GB, loB, hiB = B
    if np.any(loA - m > hiB) or np.any(loB - m > hiA):
        return None
    for ga in GA:
        for gb in GB:
            for la, Wa, a0, a1 in ga:
                for lb, Wb, b0, b1 in gb:
                    if np.any(a0 - m > b1) or np.any(b0 - m > a1):
                        continue
                    if separacion(Wa, Wb) > -m / 2:
                        return (la, lb)
    return None


if __name__ == '__main__':
    t0 = time.time()
    T, E = tramos()
    todos = {**{('tramo',) + k: v for k, v in T.items()}, **{('espera',) + k: v for k, v in E.items()}}
    geo = {k: geometria(k[1], v) for k, v in todos.items()}
    print(f'{len(T)} tramos, {len(E)} esperas, {sum(len(v) for v in todos.values())} posturas', flush=True)
    conflictos = []
    claves = list(todos)
    for a, b in itertools.combinations(claves, 2):
        if a[1] == b[1] or (a[0] == 'espera' and b[0] == 'espera'):
            continue
        m = HOLGURA_QUIETO if 'espera' in (a[0], b[0]) else HOLGURA_MOVIL
        c = chocan(geo[a], geo[b], m)
        if c:
            conflictos.append([f'{a[1]}:{a[2]}', f'{b[1]}:{b[2]}', c[0], c[1]])
            print(f'  CONFLICTO {a[1]}:{a[2]} <-> {b[1]}:{b[2]}  ({c[0]} / {c[1]})', flush=True)
    trabas = [c for c in conflictos if c[0].endswith(':HOME') or c[1].endswith(':HOME')]
    if trabas:
        sys.exit('NO se guarda la tabla: estos tramos chocan con un robot quieto en HOME y trabarian la linea '
                 '(hay que re-planear esas rutas primero):\n  ' + '\n  '.join(f'{x[0]} <-> {x[1]}' for x in trabas))
    json.dump({'holgura_movil_m': HOLGURA_MOVIL, 'holgura_quieto_m': HOLGURA_QUIETO,
               'nota': 'generado por tools/tabla_conflictos.py: pares de tramos/esperas que no pueden coincidir',
               'conflictos': conflictos}, open(SALIDA, 'w'), indent=1)
    print(f'{len(conflictos)} conflictos -> {os.path.normpath(SALIDA)}  ({time.time() - t0:.0f} s)')
