# A trabajar con quad.ttt

import sim
import numpy as np
import cv2
import slam


def connect(port):
    sim.simxFinish(-1)  # cerrar conexiones
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)  # conectarse
    if clientID == 0:
        print("Conectado a: ", port)
    else:
        print("No se pudo conectar")
    return clientID


def main():
    key_W = ord('w')
    key_A = ord('a')
    key_S = ord('s')
    key_D = ord('d')
    key_K = ord('k')
    key_I = ord('i')
    key_J = ord('j')
    key_L = ord('l')

    positional_movement_keys = [key_W, key_A, key_S, key_D, key_I, key_K]
    rotation_movement_keys = [key_J, key_L]
    movement_keys = [*positional_movement_keys, *rotation_movement_keys]

    clientID = connect(19999)
    # QUAD
    returnCode, target = sim.simxGetObjectHandle(clientID, '/target', sim.simx_opmode_blocking)
    # returnCode, target_pos = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_blocking)
    # print(target_pos)
    # sim.simxSetObjectPosition(clientID, target, -1, [-.3, .5, .4], sim.simx_opmode_oneshot)
    # Camara
    ret, camara = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
    sim.simxGetVisionSensorImage(clientID, camara, 0, sim.simx_opmode_streaming)
    while sim.simxGetConnectionId(clientID) != -1:
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, camara, 0, sim.simx_opmode_buffer)
        if err == sim.simx_return_ok:
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[1], resolution[0], 3])
            img = cv2.flip(img, 1)
            cv2.imshow('image', img)
            slam.generate_SLAM(img)
            key_pressed = cv2.waitKey(1) & 0xFF
            if key_pressed == ord('q'):
                break
            elif key_pressed in movement_keys:
                returnCode, target_pos = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_blocking)
                returnCode, target_rotation = sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_blocking)
                target_pos = np.array(target_pos)
                target_rotation = np.array(target_rotation)
                if key_pressed == key_W:
                    movement_vector = np.array([0.1, 0, 0])
                elif key_pressed == key_A:
                    movement_vector = np.array([0.0, 0.1, 0])
                elif key_pressed == key_S:
                    movement_vector = np.array([-0.1, 0, 0])
                elif key_pressed == key_D:
                    movement_vector = np.array([0, -0.1, 0])
                elif key_pressed == key_I:
                    movement_vector = np.array([0, 0, 0.1])
                elif key_pressed == key_K:
                    movement_vector = np.array([0, 0, -0.1])
                elif key_pressed == key_J:
                    movement_vector = np.array([0, 0, 0.5])
                else:
                    movement_vector = np.array([0, 0, -0.5])
                if key_pressed in positional_movement_keys:
                    sim.simxSetObjectPosition(
                        clientID,
                        target,
                        sim.sim_handle_parent,
                        target_pos + movement_vector,
                        sim.simx_opmode_oneshot
                    )
                else:
                    sim.simxSetObjectOrientation(
                        clientID,
                        target,
                        -1,
                        target_rotation + movement_vector,
                        sim.simx_opmode_oneshot
                    )
        elif err == sim.simx_return_novalue_flag:
            pass
        else:
            print(err)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
