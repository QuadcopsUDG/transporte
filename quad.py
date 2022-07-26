import time
import sim
import numpy as np
import cv2
import threading
import logging
import math

logging.basicConfig(level=logging.DEBUG)

PUERTO_SIMULACION = 19999

def mover_dron(id_cliente, referencia_dron, referencia_objetivo, transformacion):
    """
    Mover el objetivo a una nueva posicion, transformación relativa
    [delta x, delta y, delta z] EN METROS
    """
    # posición DEL objetivo del dron. El dron lo seguirá
    _, posicion_inicial_absoluta = sim.simxGetObjectPosition(
        id_cliente,
        referencia_objetivo,
        -1,
        sim.simx_opmode_buffer
    )

    # posición a la que se busca que vaya el dron
    posicion_objetivo = [
        transformacion[i] + posicion_inicial_absoluta[i]
        for i in range(len(posicion_inicial_absoluta))
    ]

    logging.debug(f"Moviendo desde {str(posicion_inicial_absoluta)}")
    logging.debug(f"hacia -------> {str(posicion_objetivo)}")

    # Mover el objetivo a una nueva posicion
    sim.simxSetObjectPosition(
        id_cliente,
        referencia_objetivo,
        -1,
        posicion_objetivo,
        sim.simx_opmode_oneshot
    )

    # Esperar a que el dron alcance el objetivo
    while sim.simxGetConnectionId(id_cliente) != -1:
        _, posicion_dron = sim.simxGetObjectPosition(
            id_cliente,
            referencia_dron,
            -1,
            sim.simx_opmode_blocking
        )

        # comparar posiciones del objetivo y del dron
        diferencia_total = sum(
            abs(posicion_dron[i] - posicion_objetivo[i])
            for i in range(len(posicion_objetivo))
        )

        if diferencia_total <= 0.007: # 7 milímetros de tolerancia
            break

        # revisar cada medio segundo
        time.sleep(0.5)

def mover_para_enfocar(id_cliente: int, referencia_dron: int, referencia_objetivo: int):
    mover_dron(id_cliente, referencia_dron, referencia_objetivo, [0, 0, 0.5])
    mover_dron(id_cliente, referencia_dron, referencia_objetivo, [0, 0, -0.5])

def esbos_tresesenta(id_cliente: int, referencia_objetivo: int):
    # Angulos de Euler
    # [
    #   roll
    #   pitch
    #   yaw (el unico que necesitamos mover)
    # ]

    # iniciar "streaming" de orientación del dron
    sim.simxGetObjectOrientation(
        id_cliente,
        referencia_objetivo,
        -1,
        sim.simx_opmode_streaming
    )


    # las orientaciones llegan a ser PI como máximo, luego pasa a -2.999, -1 y de vuelta a 0
    # este acumulador nos permite _no_ rompernos la cabeza para saber si ya hizo una rotación completa
    rotacion_total = 0

    # dar una vuelta de 360°
    while sim.simxGetConnectionId(id_cliente) != -1:
        # orientacion = rotaciones en [X, Y, Z]
        _, orientacion = sim.simxGetObjectOrientation(
            id_cliente,
            referencia_objetivo,
            -1,
            sim.simx_opmode_buffer
        )

        rotacion_total += 0.1

        if rotacion_total >= math.tau:
            orientacion[2] = 0
            sim.simxSetObjectOrientation(
                id_cliente,
                referencia_objetivo,
                -1,
                orientacion,
                sim.simx_opmode_oneshot
            )
            break

        orientacion[2] += 0.1

        sim.simxSetObjectOrientation(
            id_cliente,
            referencia_objetivo,
            -1,
            orientacion,
            sim.simx_opmode_oneshot
        )

        time.sleep(0.4)

def hilo_dron(id_cliente: int, referencia_dron: int, referencia_objetivo: int):
    """
    En este target se manejan todos los movimientos del dron y acá
    """
    mover_para_enfocar(id_cliente, referencia_dron, referencia_objetivo)
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
                # sin cámara no tiene sentido vivir...
                sim.simxFinish(id_cliente)
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
    _, referencia_dron = sim.simxGetObjectHandle(id_cliente, 'Quadcopter', sim.simx_opmode_blocking)
    _, referencia_objetivo = sim.simxGetObjectHandle(id_cliente, 'Quadcopter_target', sim.simx_opmode_blocking)

    # posición DEL objetivo del dron. El dron lo seguirá
    _, posicion_inicial_absoluta = sim.simxGetObjectPosition(
        id_cliente,
        referencia_objetivo,
        -1,
        sim.simx_opmode_streaming
    )
    logging.debug(f"Posicion inicial {str(referencia_objetivo)} {str(posicion_inicial_absoluta)}")

    # Camara
    _, camara = sim.simxGetObjectHandle(id_cliente, 'camara', sim.simx_opmode_blocking)

    logging.debug("Inicializando hilo de la cámara")
    hilo_camara = threading.Thread(target = correr_camara, args = (id_cliente, camara))
    logging.debug("Corriendo hilo de la cámara")
    hilo_camara.start()

    logging.debug("Inicializando hilo para mover el bote")
    hilo_mover_el_bote = threading.Thread(
        target = hilo_dron,
        args = (id_cliente, referencia_dron, referencia_objetivo)
    )
    logging.debug("Moviendo el bote...")
    hilo_mover_el_bote.start()

if __name__ == "__main__":
    main()