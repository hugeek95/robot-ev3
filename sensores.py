#!/usr/bin/env python3
# Seguidor de línea EV3 (ev3dev2) con PID y calibración
# Requiere: 1 x ColorSensor (puerto 1), 2 x LargeMotor (puertos B y C)

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, MoveTank, SpeedPercent
from ev3dev2.sensor import INPUT_1
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
cs = ColorSensor(INPUT_1)
btn = Button()
snd = Sound()

def clip(v, lo, hi):
    return max(lo, min(hi, v))

def calibrar():
    """
    1) Pon el sensor sobre NEGRO (línea) y pulsa el botón central.
    2) Pon el sensor sobre BLANCO (pista) y pulsa el botón central.
    Devuelve (black, white, target).
    """
    snd.speak('Calibracion. Negro primero.')
    while not btn.enter:
        time.sleep(0.01)
    black = cs.reflected_light_intensity
    snd.beep()
    while btn.enter:
        time.sleep(0.01)

    snd.speak('Ahora blanco.')
    while not btn.enter:
        time.sleep(0.01)
    white = cs.reflected_light_intensity
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
    with open("sensor_log.txt", "w") as f:
        # Encabezado
        f.write("timestamp,light,error,left,right\n")

        try:
            while not btn.backspace:  # Salir con botón "back"
                # Medición y tiempo
                light = cs.reflected_light_intensity
                now = time.time()
                dt = max(1e-3, now - last_t)

                # Error respecto al objetivo
                error = target - light

                # PID
                integral += error * dt
                derivative = (error - last_error) / dt
                turn = KP * error + KI * integral + KD * derivative

                # Mezcla diferencial
                left = clip(BASE_SPEED + turn, -100, 100)
                right = clip(BASE_SPEED - turn, -100, 100)

                # Mover
                tank.on(SpeedPercent(left), SpeedPercent(right))

                # Guardar en archivo
                f.write("{:.3f},{},{:.3f},{:.2f},{:.2f}\n".format(now, light, error, left, right))

                # Actualiza historial
                last_error = error
                last_t = now

                time.sleep(0.01)

        except KeyboardInterrupt:
            pass
        finally:
            tank.off()
            snd.speak('Detenido')

if __name__ == "__main__":
    cs.mode = 'COL-REFLECT'
    black, white, target = calibrar()
    seguir_linea(black, white, target)
