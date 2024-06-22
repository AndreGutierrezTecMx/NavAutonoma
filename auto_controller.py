import os
import csv
from controller import Robot, Camera, Display, Keyboard
from vehicle import Car, Driver
import numpy as np
from PIL import Image

# Crear el directorio para almacenar las imágenes y el archivo CSV si no existen
output_dir = "Train_data_navegacion_autonoma"
images_dir = os.path.join(output_dir, "images")
os.makedirs(images_dir, exist_ok=True)
csv_file = os.path.join(output_dir, "data.csv")

# Inicializar el archivo CSV
if not os.path.exists(csv_file):
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["path_img_right", "path_img_left", "path_img_front", "angulo_de_giro"])

# Función para obtener dispositivos
def get_devices(robot, type_of_device):
    if type_of_device == "camera" or type_of_device == "display":
        device_right = robot.getDevice(f"right_{type_of_device}")
        device_left = robot.getDevice(f"left_{type_of_device}")
        device_front = robot.getDevice(f"front_{type_of_device}")
        if type_of_device == "display":
            return device_right, device_left, device_front
    timestep = int(robot.getBasicTimeStep())
    if type_of_device == "camera":
        device_right.enable(timestep)
        device_left.enable(timestep)
        device_front.enable(timestep)
        return device_right, device_left, device_front
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

# Función para guardar imágenes y registrar en el CSV
def save_images_and_log(rgb_images, angle, img_num):
    right_img_path = os.path.join(images_dir, f"right_image_{img_num}.png")
    left_img_path = os.path.join(images_dir, f"left_image_{img_num}.png")
    front_img_path = os.path.join(images_dir, f"front_image_{img_num}.png")
    
    Image.fromarray(rgb_images[0]).save(right_img_path)
    Image.fromarray(rgb_images[1]).save(left_img_path)
    Image.fromarray(rgb_images[2]).save(front_img_path)

    with open(csv_file, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([right_img_path, left_img_path, front_img_path, angle])

# Inicialización de ángulos y velocidad
manual_steering = 0
steering_angle = 0
angle = 0.0
speed = 15
img_num = 0  # Contador de imágenes
recording = False

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

# Función principal
def main():
    global img_num, recording
    robot = Car()
    driver = Driver()
    
    camera_right, camera_left, camera_front = get_devices(robot, "camera")
    display_right, display_left, display_front = get_devices(robot, "display")
    keyboard = get_devices(robot, "keyboard")
    
    while robot.step() != -1:
        rgb_right = display_camera_image(camera_right, display_right)
        rgb_left = display_camera_image(camera_left, display_left)
        rgb_front = display_camera_image(camera_front, display_front)

        key = keyboard.getKey()
        if key == Keyboard.UP:
            set_speed(speed + 5.0)
            print("up")
        elif key == Keyboard.DOWN:
            set_speed(speed - 5.0)
            print("down")
        elif key == Keyboard.RIGHT:
            change_steer_angle(+0.8)
            print("right")
        elif key == Keyboard.LEFT:
            change_steer_angle(-0.8)
            print("left")
        elif key == ord('R'):  # Empezar a capturar el video
            recording = True
            print("Start recording")
        elif key == ord('S'): # Terminar de capturar el video
            recording = False
            print("Stop recording")
        driver.setSteeringAngle(angle)
        driver.setCruisingSpeed(speed)
        if (recording): 
            save_images_and_log([rgb_right, rgb_left, rgb_front], angle, img_num)
            img_num += 1

if __name__ == "__main__":
    main()
