# C-digo-para-robot-humanoide-con-lenguaje-de-programaci-n-python
# Robot Humanoide Bípedo – Control y Coordinación con ROS

## Descripción general

Este repositorio contiene el código base para el control de un **robot humanoide bípedo** utilizando **MicroPython**, un **ESP32** y un **driver PCA9685** para el manejo de servomotores. El enfoque principal del proyecto es el desarrollo de un **ciclo de caminata** mediante **cinemática inversa**, así como la proyección futura hacia un sistema distribuido donde **múltiples robots humanoides puedan comunicarse y coordinarse entre sí** usando **ROS (Robot Operating System)**.

Actualmente, el sistema implementa trayectorias sincronizadas para ambas piernas, permitiendo movimientos estables y repetibles. A partir de esta base, se plantea la integración de ROS como capa de comunicación y coordinación entre robots.

## Objetivo del proyecto

El objetivo principal es desarrollar una arquitectura en la que varios robots humanoides:

* Puedan **detectarse entre sí** dentro de una misma red.
* **Compartan información** relevante (estado, posición, tarea actual).
* Ejecuten **acciones coordinadas**, como caminar al mismo tiempo o realizar una tarea asignada de forma colectiva.
* Escalen fácilmente desde un solo robot a un **sistema multi‑robot**.

## Rol de ROS en el proyecto
ROS se plantea como el middleware encargado de:

* Gestionar la **comunicación entre robots** mediante nodos y tópicos.
* Permitir la asignación de **tareas globales** (por ejemplo: caminar, detenerse, formarse, seguir a otro robot).
* Sincronizar comportamientos usando mensajes estándar.

### Concepto de funcionamiento

1. Cada robot ejecuta el código de control de bajo nivel (servos y cinemática) en el ESP32.
2. Un nodo ROS actúa como **interfaz de alto nivel**, enviando comandos como:

   * `caminar`
   * `detener`
   * `sincronizar movimiento`
3. Los robots publican su estado (ej. `robot_state`) y escuchan instrucciones comunes.
4. Al recibir una orden compartida, todos los robots ejecutan la misma acción de forma sincronizada.

## Alcance actual y futuro

### Estado actual

* Control de servomotores mediante PCA9685.
* Implementación de cinemática inversa para piernas.
* Ciclo de caminata continuo y estable.

### Trabajo futuro

* Integración de ROS/ROS2 mediante un nodo puente (PC o SBC como Raspberry Pi).
* Descubrimiento automático de robots en la red.
* Comunicación multi‑robot usando tópicos ROS.
* Ejecución de comportamientos cooperativos.

## Enfoque educativo

Este proyecto está orientado a:

* Aprendizaje de **robótica humanoide**.
* Introducción práctica a **cinemática inversa**.
* Comprensión de sistemas **multi‑robot**.
* Uso de **ROS como estándar industrial y académico**.

## Nota

Este repositorio representa una base experimental y educativa. El código y la arquitectura están en constante evolución conforme se integran nuevas tecnologías y conceptos de coordinación entre robots.

#A continuacion el codigo:
from machine import Pin, I2C
from pca9685 import PCA9685
import math, time


# CONFIGURACIÓN DEL PCA9685 Y SERVOS
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
pca = PCA9685(i2c)
pca.freq(50)

def set_servo_angle(channel, angle):
    angle = max(0, min(180, int(angle)))
    min_us, max_us = 500, 2500
    us = min_us + (max_us - min_us) * angle / 180
    ticks = int(us * pca.freq() * 4096 / 1_000_000)
    pca.duty(channel, ticks)


# PARÁMETROS DEL ROBOT
l1 = 70
l2 = 55

# TRAYECTORIAS (x, y, z = inclinación)
trayectoria1 = [

    (-40, 85, 7),
    (-40, 85, 15),
    (-40, 85, 7),
    
    (5, 118, 0),
    
    (5, 118, -5),
    (20, 118, -10),
    (20, 118, -5),
    
    (5, 118, 0),
  
  
]

trayectoria2 = [
    (-5, 118, -5),
    (20, 118, -10),
    (20, 118,-5),
    
    (-5, 118, 0),
    
    (-40, 85, 7),
     (-40, 85, 15),
    (-40, 85, 7),
    
    (-5, 118, 0),
    

]


# CINEMÁTICA INVERSA + MOVIMIENTO
def mover_a_punto_izquierda(px, py, inclinacion):
    num = (px**2 + py**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if num > 1 or num < -1:
        print("Punto fuera del alcance (izq):", px, py)
        return
    s2 = math.sqrt(1 - num**2)
    teta1 = math.atan2(px, py) - math.atan2(l2 * s2, l1 + l2 * num)
    teta2 = math.atan2(s2, num)
    t1_deg = math.degrees(teta1)
    t2_deg = math.degrees(teta2)

    servo1 = 82 + t1_deg
    servo2 = 90 + t2_deg
    servo3 = 80 - ((t2_deg + t1_deg) / 2)
    servo_cadera = 90 + inclinacion

    set_servo_angle(0, servo1)
    set_servo_angle(1, servo2)
    set_servo_angle(2, servo3)
    set_servo_angle(3, servo_cadera)

def mover_a_punto_derecha(px, py, inclinacion):
    num = (px**2 + py**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if num > 1 or num < -1:
        print("Punto fuera del alcance (der):", px, py)
        return
    s2 = math.sqrt(1 - num**2)
    teta1 = math.atan2(px, py) - math.atan2(l2 * s2, l1 + l2 * num)
    teta2 = math.atan2(s2, num)
    t1_deg = math.degrees(teta1)
    t2_deg = math.degrees(teta2)

    servo4 = 92 - t1_deg
    servo5 = 90 - t2_deg
    servo6 = 100 + ((t2_deg + t1_deg) / 2)
    servo_cadera = 90 - inclinacion

    set_servo_angle(4, servo4)
    set_servo_angle(5, servo5)
    set_servo_angle(6, servo6)
    set_servo_angle(7, servo_cadera)


# FUNCIÓN: MOVIMIENTO SIMULTÁNEO
def ejecutar_trayectorias_sync(tray1, tray2, delay=1):
    longitud_max = max(len(tray1), len(tray2))

    for i in range(longitud_max):
        p1 = tray1[i] if i < len(tray1) else tray1[-1]
        p2 = tray2[i] if i < len(tray2) else tray2[-1]

        print(f"\nPaso {i+1}/{longitud_max}")
        mover_a_punto_izquierda(*p1)
        mover_a_punto_derecha(*p2)

        time.sleep(delay)


# LOOP INFINITO (WALK CYCLE)
print("Iniciando ciclo infinito de caminata...")

while True:
    ejecutar_trayectorias_sync(trayectoria1, trayectoria2, delay=.19)
    time.sleep(0.01)   # pequeña pausa opcional para estabilidad
