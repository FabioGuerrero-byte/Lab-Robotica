#!/usr/bin/env python3
import heapq, json, math, os, sys
import numpy as np
from geometria_celda import choques, home, SEC

JUNTAS = {'robot2': ('Revoluci_n_1', 'Revoluci_n_2', (-2.5, 2.4), (-2.6, 2.6)),
          'robot3': ('eslabon1_joint', 'eslabon2_joint', (-2.6, 2.6), (-1.95, 1.95)),
          'robot4': ('Joint_1', 'Joint_2', (-math.pi, math.pi), (-2.7, 2.7)),
          'robot5': ('Join_1', 'Joint_2', (-1.536, 1.536), (-1.536, 1.536))}
NOMBRE = {'ir': 'ir a recoger', 'llevar': 'llevar', 'volver': 'volver a HOME'}
EXTRA = {('robot3', 'llevar'): {'garra1_joint': 0.24, 'garra2_joint': 0.24},
         ('robot3', 'volver'): {'garra1_joint': 0.0, 'garra2_joint': 0.0},
         ('robot3', 'ir'): {'garra1_joint': 0.0, 'garra2_joint': 0.0}}
RUTA = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'secuencia.json')

ns, tramo = sys.argv[1], sys.argv[2]
HOLGURA = float(sys.argv[3]) if len(sys.argv) > 3 else 0.008
j1, j2, r1, r2 = JUNTAS[ns]
H = home()
S = json.load(open(RUTA))
paso = [p for p in S['plan'] if p['robot'] == ns][0]
kfs = paso['keyframes']
k = [n for n, x in enumerate(kfs) if x[0] == NOMBRE[tramo]][0]
fijo = {j: v for j, v in kfs[k][1].items() if j not in (j1, j2)}
fijo.update(EXTRA.get((ns, tramo), {}))
A = kfs[k - 1][1] if k > 0 else H[ns]
B = kfs[k][1]
A = np.array([A[j1], A[j2]]); B = np.array([B[j1], B[j2]])


def libre(p):
    conf = {r: dict(q) for r, q in H.items()}
    conf[ns] = dict(fijo, **{j1: float(p[0]), j2: float(p[1])})
    return not choques(conf, umbral=-HOLGURA / 2, solo=ns)


def recto(P, Q):
    n = max(2, int(np.max(np.abs(Q - P)) / math.radians(1)) + 1)
    return all(libre(P + (Q - P) * s) for s in np.linspace(0, 1, n))


print(f'{ns} {tramo}: de {np.degrees(A).round(0)} a {np.degrees(B).round(0)} grados, holgura {HOLGURA * 1000:.0f} mm',
      flush=True)
if recto(A, B):
    pts = [A, B]
else:
    paso_g = math.radians(4)
    Q1 = np.arange(r1[0], r1[1] + 1e-9, paso_g); Q2 = np.arange(r2[0], r2[1] + 1e-9, paso_g)
    L = np.array([[libre((a, b)) for b in Q2] for a in Q1])
    print(f'  libre: {L.mean() * 100:.0f}% de las posturas', flush=True)
    ia = (int(np.argmin(abs(Q1 - A[0]))), int(np.argmin(abs(Q2 - A[1]))))
    ib = (int(np.argmin(abs(Q1 - B[0]))), int(np.argmin(abs(Q2 - B[1]))))
    L[ia] = L[ib] = True
    D = np.full(L.shape, np.inf); prev = {}; D[ia] = 0; h = [(0.0, ia)]
    while h:
        d, (i, kk) = heapq.heappop(h)
        if (i, kk) == ib:
            break
        if d > D[i, kk]:
            continue
        for di in (-1, 0, 1):
            for dk in (-1, 0, 1):
                ii, k2 = i + di, kk + dk
                if (di or dk) and 0 <= ii < len(Q1) and 0 <= k2 < len(Q2) and L[ii, k2]:
                    nd = d + math.hypot(di, dk)
                    if nd < D[ii, k2]:
                        D[ii, k2] = nd; prev[(ii, k2)] = (i, kk); heapq.heappush(h, (nd, (ii, k2)))
    if not np.isfinite(D[ib]):
        sys.exit('  SIN RUTA con esa holgura')
    c = [ib]
    while c[-1] != ia:
        c.append(prev[c[-1]])
    grid = [A] + [np.array([Q1[i], Q2[kk]]) for i, kk in c[::-1][1:-1]] + [B]
    pts = [grid[0]]; i = 0
    while i < len(grid) - 1:
        kk = len(grid) - 1
        while kk > i + 1 and not recto(grid[i], grid[kk]):
            kk -= 1
        pts.append(grid[kk]); i = kk
via = [dict({j: v for j, v in kfs[k][1].items() if j not in (j1, j2)}, **{j1: float(p[0]), j2: float(p[1])})
       for p in pts[1:-1]]
kfs[k] = [kfs[k][0], kfs[k][1], kfs[k][2], via]
json.dump(S, open(RUTA, 'w'), indent=1)
print('  ruta:', ' -> '.join('(%.0f, %.0f)' % tuple(np.degrees(p)) for p in pts), '-> secuencia.json')
