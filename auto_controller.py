import os
import csv
from controller import Robot, Camera, Display, Keyboard
from vehicle import Car, Driver
import numpy as np
from PIL import Image
import cv2
import tensorflow as tf

# Función para obtener dispositivos
def get_device(robot, type_of_device):
    if type_of_device == "camera" or type_of_device == "display":
        device_front = robot.getDevice(f"front_{type_of_device}")
        if type_of_device == "display":
            return device_front
    timestep = int(robot.getBasicTimeStep())
    if type_of_device == "camera":
        device_front.enable(timestep)
        return device_front
    keyboard = Keyboard()
    keyboard.enable(timestep)
    return keyboard    

# Función para mostrar imagen de la cámara en el display
def display_camera_image(camera, display):
    raw_image = camera.getImage()
    height = camera.getHeight()
    width = camera.getWidth()
    image_array = np.frombuffer(raw_image, np.uint8).reshape((height, width, 4))
    rgb_image = image_array[:, :, [2, 1, 0]]  # Convertir BGRA a RGB
    displayed_image = display.imageNew(rgb_image.tobytes(), Display.RGB, width, height)
    display.imagePaste(displayed_image, 0, 0, False)
    display.imageDelete(displayed_image)
    return rgb_image  # Devolver la imagen RGB para guardarla

# Inicialización de ángulos y velocidad
manual_steering = 0
steering_angle = 0
angle = 0.0
speed = 20
# Ruta donde se encuentra el modelo guardado
modelo_ruta = 'nvidia_model_5.keras'
# Cargar el modelo
modelo = tf.keras.models.load_model('nvidia_model.keras')

# Establecer la velocidad objetivo
def set_speed(kmh):
    global speed
    speed = kmh

# Actualizar el ángulo de dirección
def set_steering_angle(wheel_angle):
    global angle, steering_angle
    if (wheel_angle - steering_angle) > 0.1:
        wheel_angle = steering_angle + 0.1
    if (wheel_angle - steering_angle) < -0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle
    if wheel_angle > 0.5:
        wheel_angle = 0.5
    elif wheel_angle < -0.5:
        wheel_angle = -0.5
    angle = wheel_angle

# Validar incremento del ángulo de dirección
def change_steer_angle(inc):
    global manual_steering
    new_manual_steering = manual_steering + inc
    if new_manual_steering <= 25.0 and new_manual_steering >= -25.0: 
        manual_steering = new_manual_steering
        set_steering_angle(manual_steering * 0.02)
    if manual_steering == 0:
        print("going straight")
    else:
        turn = "left" if steering_angle < 0 else "right"
        print("turning {} rad {}".format(str(steering_angle), turn))

def img_preprocess(img):
  # Recortar la parte superior y lateral de la imagen
  img_crop = img[35:60, 0:60]
  # Escalar la imagen a 200x66
  img_crop = cv2.resize(img_crop, (200, 66))
  # Transformar el color
  img_crop = cv2.cvtColor(img_crop, cv2.COLOR_RGB2YUV)
  # Aplicar desenfoque Gaussiano
  img_crop = cv2.GaussianBlur(img_crop, (3, 3), 0)
  return np.expand_dims(img_crop, axis=0) 


# Función principal
def main():
    global img_num, recording
    robot = Car()
    driver = Driver()
    
    camera_front = get_device(robot, "camera")
    display_front = get_device(robot, "display")
    keyboard = get_device(robot, "keyboard")
    
    while robot.step() != -1:
        rgb_front = display_camera_image(camera_front, display_front)
        rgb_front = img_preprocess(rgb_front)
        angle_front = modelo.predict(rgb_front)
        mean_angle = angle_front
        print(f"The mean angle is: {mean_angle}")

        key = keyboard.getKey()
        if key == Keyboard.UP:
            set_speed(speed + 5.0)
            print("up")
        elif key == Keyboard.DOWN:
            set_speed(speed - 5.0)
            print("down")
        driver.setSteeringAngle(mean_angle)
        driver.setCruisingSpeed(speed)

if __name__ == "__main__":
    main()
