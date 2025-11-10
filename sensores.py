#!/usr/bin/env python3
# Seguidor de línea EV3 (ev3dev2) con PID y calibración
# Requiere: 3 x ColorSensor (puertos 1, 2 y 3), 2 x LargeMotor (puertos B y C)

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, MoveTank, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.button import Button
from ev3dev2.sound import Sound
import time

# === Parámetros PID y velocidad ===
BASE_SPEED = 30       # % de potencia base por rueda (ajusta a tu pista/robot)
KP = 1.2              # Proporcional
KI = 0.0              # Integral (empieza en 0, sube si hay error sostenido)
KD = 0.8              # Derivativo (suaviza oscilaciones)

# === Inicialización de hardware ===
tank = MoveTank(OUTPUT_B, OUTPUT_C)
cs1 = ColorSensor(INPUT_1)  # Sensor principal para el control (PID)
cs2 = ColorSensor(INPUT_2)  # Sensor secundario (solo registro)
cs3 = ColorSensor(INPUT_3)  # Sensor secundario (solo registro)
btn = Button()
snd = Sound()

def clip(v, lo, hi):
    return max(lo, min(hi, v))

def calibrar():
    """
    Calibra usando el SENSOR 1 (principal):
    1) Pon el sensor 1 sobre NEGRO (línea) y pulsa el botón central.
    2) Pon el sensor 1 sobre BLANCO (pista) y pulsa el botón central.
    Devuelve (black, white, target).
    """
    snd.speak('Calibracion. Negro primero.')
    while not btn.enter:
        time.sleep(0.01)
    black = cs1.reflected_light_intensity
    snd.beep()
    while btn.enter:
        time.sleep(0.01)

    snd.speak('Ahora blanco.')
    while not btn.enter:
        time.sleep(0.01)
    white = cs1.reflected_light_intensity
    snd.beep()
    while btn.enter:
        time.sleep(0.01)

    target = (black + white) / 2.0
    snd.speak('Listo')
    print("Calibrado: black={}, white={}, target={}".format(black, white, target))
    return black, white, target

def seguir_linea(black, white, target):
    integral = 0.0
    last_error = 0.0
    last_t = time.time()

    snd.speak('Iniciando')

    # Abrimos archivo de log
    # Nota: formato CSV simple para facilitar análisis posterior.
    with open("sensor_log.txt", "w") as f:
        # Encabezado
        f.write("timestamp,light1,light2,light3,error1,error2,error3,left,right\n")

        try:
            while not btn.backspace:  # Salir con botón "back"
                # Medición y tiempo
                l1 = cs1.reflected_light_intensity
                l2 = cs2.reflected_light_intensity
                l3 = cs3.reflected_light_intensity

                now = time.time()
                dt = max(1e-3, now - last_t)

                # Errores respecto al objetivo (target de sensor 1)
                e1 = target - l1
                e2 = target - l2
                e3 = target - l3

                # PID (solo con sensor 1)
                integral += e1 * dt
                derivative = (e1 - last_error) / dt
                turn = KP * e1 + KI * integral + KD * derivative

                # Mezcla diferencial
                left = clip(BASE_SPEED + turn, -100, 100)
                right = clip(BASE_SPEED - turn, -100, 100)

                # Mover
                tank.on(SpeedPercent(left), SpeedPercent(right))

                # Guardar en archivo
                f.write("{:.3f},{},{},{},{:.3f},{:.3f},{:.3f},{:.2f},{:.2f}\n".format(
                    now, l1, l2, l3, e1, e2, e3, left, right
                ))

                # Actualiza historial
                last_error = e1
                last_t = now

                time.sleep(0.01)

        except KeyboardInterrupt:
            pass
        finally:
            tank.off()
            snd.speak('Detenido')

if __name__ == "__main__":
    # Modo reflectancia para los tres sensores
    cs1.mode = 'COL-REFLECT'
    cs2.mode = 'COL-REFLECT'
    cs3.mode = 'COL-REFLECT'

    black, white, target = calibrar()
    seguir_linea(black, white, target)
