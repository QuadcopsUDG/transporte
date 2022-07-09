import time
from typing import Any, Callable, Iterable, Mapping
import sim
import numpy as np
import cv2
import threading
import logging
import random
import math

logging.basicConfig(level=logging.DEBUG)

PUERTO_SIMULACION = 19999

def mover_dron(id_cliente: int, referencia_objetivo: int, pos: list):
    # Mover el objetivo a nueva, posicion
    sim.simxSetObjectPosition(
        id_cliente,
        referencia_objetivo,
        -1,
        pos,
        sim.simx_opmode_oneshot
    )

def mover_para_enfocar(id_cliente: int, referencia_objetivo: int):
    _, pos = sim.simxGetObjectPosition(id_cliente, referencia_objetivo, -1, sim.simx_opmode_blocking)
    pos[2] += 20
    mover_dron(id_cliente, referencia_objetivo, pos)
    _, pos = sim.simxGetObjectPosition(id_cliente, referencia_objetivo, -1, sim.simx_opmode_blocking)
    pos[2] -= 20
    mover_dron(id_cliente, referencia_objetivo, pos)

def esbos_tresesenta(id_cliente: int, referencia_objetivo: int):
    # Angulos de Euler
    # [
    #   roll
    #   pitch
    #   yaw (el unico que necesitamos mover)
    # ]

    # iniciar "streaming" de orientación del dron
    _, pos = sim.simxGetObjectOrientation(
        id_cliente,
        referencia_objetivo,
        -1,
        sim.simx_opmode_streaming
    )

    rotacion_inicial = pos[2]
    pos[2] += 0.1

    while math.ceil(abs(pos[2])) > rotacion_inicial and sim.simxGetConnectionId(id_cliente) != -1:
        _, pos = sim.simxGetObjectOrientation(
            id_cliente,
            referencia_objetivo,
            -1,
            sim.simx_opmode_buffer
        )

        pos[2] += 0.1
        print(pos[2])

        # aqui deberiamos hacer la rotacion con los angulos de Euler. Kemeyo
        sim.simxSetObjectOrientation(
            id_cliente,
            referencia_objetivo,
            -1,
            pos,
            sim.simx_opmode_oneshot
        )

        time.sleep(0.3)

def hilo_dron(id_cliente: int, referencia_objetivo: int, referencia_dron: int):
    """
    En este target se manejan todos los movimientos del dron y acá
    """
    # mover_para_enfocar(id_cliente, referencia_objetivo)
    # sim.simxGetObjectPosition(id_cliente, referencia_objetivo, -1, sim.simx_opmode_blocking)
    esbos_tresesenta(id_cliente, referencia_objetivo)

def correr_camara(id_cliente: int, referencia_camara: int) -> None:
    logging.debug("Corriendo el hilo de la camara")
    err, resolution, image = sim.simxGetVisionSensorImage(id_cliente, referencia_camara, 0, sim.simx_opmode_streaming)
    while (sim.simxGetConnectionId(id_cliente) != -1):
        err, resolution, image = sim.simxGetVisionSensorImage(id_cliente, referencia_camara, 0, sim.simx_opmode_buffer)
        if err == sim.simx_return_ok:
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[1], resolution[0], 3])
            img = cv2.flip(img, 1)
            cv2.imshow('image', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == sim.simx_return_novalue_flag:
            pass
        else:
            logging.error(err)

    cv2.destroyAllWindows()

def conectar_a_coppelia(puerto):
    logging.debug("Conectando a Coppelia")
    host = "127.0.0.1"
    sim.simxFinish(-1) # cerrar conexiones
    id_cliente = sim.simxStart('127.0.0.1', puerto, True, True, 2000, 5)

    if id_cliente == 0:
        logging.info("Conectado a {}:{}".format(host, puerto))
    else:
        raise Exception("No se pudo conectar a {}:{}".format(host, puerto))

    return id_cliente

def main():
    id_cliente = conectar_a_coppelia(19999)
    _, target_dron = sim.simxGetObjectHandle(id_cliente, 'Quadcopter_target', sim.simx_opmode_blocking)
    _, referencia_dron = sim.simxGetObjectHandle(id_cliente, 'Quadcopter', sim.simx_opmode_blocking)
    # _, posicion_dron = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_blocking)

    # Camara
    _, camara = sim.simxGetObjectHandle(id_cliente, 'camara', sim.simx_opmode_blocking)

    logging.debug("Inicializando hilo de la cámara")
    hilo_camara = threading.Thread(target = correr_camara, args = (id_cliente, camara))
    logging.debug("Corriendo hilo de la cámara")
    hilo_camara.start()

    logging.debug("Inicializando hilo para mover el bote")
    hilo_mover_el_bote = threading.Thread(target = hilo_dron, args = (id_cliente, target_dron, referencia_dron))
    logging.debug("Moviendo el bote...")
    hilo_mover_el_bote.start()

if __name__ == "__main__":
    main()