#!/usr/bin/env python3
import random, socket, sys, threading, time

PC = (sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1', 5005)
ROBOTS = [int(x) for x in (sys.argv[2] if len(sys.argv) > 2 else '1,2,3,4,5').split(',')]
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0


def robot(n):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(0.1)
    conectado, hechos = False, set()
    angulos = [1.6275, 1.8523, 0.0, 0.0] if n == 3 else None
    ultimo_hola = ultimo_pos = 0.0
    while True:
        if not conectado and time.time() - ultimo_hola > 1.0:
            s.sendto(f'R{n} HOLA'.encode(), PC); ultimo_hola = time.time()
        if conectado and angulos and time.time() - ultimo_pos > 0.1:
            s.sendto((f'R{n} POS ' + ' '.join(f'{a:.4f}' for a in angulos)).encode(), PC); ultimo_pos = time.time()
        try:
            msg = s.recv(256).decode().strip()
        except socket.timeout:
            continue
        p = msg.split()
        if len(p) >= 2 and p[1] == 'BIENVENIDO':
            if not conectado:
                print(f'R{n}: conectado al PC', flush=True)
            conectado = True
        elif len(p) >= 4 and p[1] == 'MUEVE':
            k = p[-1]
            s.sendto(f'R{n} OK MUEVE {k}'.encode(), PC)
            if ('MUEVE', k) in hechos:
                continue
            hechos.add(('MUEVE', k))
            def mover(objetivo=[float(x) for x in p[2:-1]], k=k):
                print(f'R{n}: moviendo a mano a {objetivo}', flush=True)
                inicio = list(angulos)
                for i in range(1, 21):
                    angulos[:] = [a + (b - a) * i / 20 for a, b in zip(inicio, objetivo)]
                    time.sleep(0.1)
                s.sendto(f'R{n} LISTO MUEVE {k}'.encode(), PC)
            threading.Thread(target=mover, daemon=True).start()
        elif len(p) == 4 and p[1] == 'HAZ':
            tramo, k = p[2], p[3]
            s.sendto(f'R{n} OK {tramo} {k}'.encode(), PC)
            if (tramo, k) in hechos:
                continue
            hechos.add((tramo, k))
            def hacer(tramo=tramo, k=k):
                print(f'R{n}: haciendo "{tramo}" ({k})', flush=True)
                time.sleep(DUR * random.uniform(0.8, 1.2))
                s.sendto(f'R{n} LISTO {tramo} {k}'.encode(), PC)
                print(f'R{n}: LISTO "{tramo}" ({k})', flush=True)
            threading.Thread(target=hacer, daemon=True).start()


for n in ROBOTS:
    threading.Thread(target=robot, args=(n,), daemon=True).start()
print(f'ESP32 simulados {ROBOTS} -> PC {PC[0]}:{PC[1]} (cada tramo ~{DUR} s). Ctrl+C para salir.', flush=True)
while True:
    time.sleep(1)
