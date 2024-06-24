import cv2
import mediapipe as mp
import serial
import time
import numpy as np

# Configurar el puerto serie
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.1)

def send_command(command):
    arduino.write((command + '\n').encode())
    time.sleep(0.01)  # Reducir la latencia del envío de comandos

def calculate_angles(x, y, frame_width, frame_height, current_pan_angle, current_tilt_angle):
    # Definir el centro del cuadro
    center_x = frame_width // 2
    center_y = frame_height // 2

    # Definir márgenes para los bordes del cuadro
    margin = 20  # Pixels
    pan_change = 1  # Grados
    tilt_change = 1  # Grados

    # Ajustar los ángulos basados en la distancia del rostro al centro del cuadro
    if x < center_x - margin:
        current_pan_angle = min(180, current_pan_angle + pan_change)
    elif x > center_x + margin:
        current_pan_angle = max(0, current_pan_angle - pan_change)

    if y < center_y - margin:
        current_tilt_angle = max(0, current_tilt_angle - tilt_change)
    elif y > center_y + margin:
        current_tilt_angle = min(180, current_tilt_angle + tilt_change)

    return current_pan_angle, current_tilt_angle

def main():
    mp_face_detection = mp.solutions.face_detection
    face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.7)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Reducir la resolución para procesar más rápido
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    time.sleep(2)  # Esperar a que el Arduino se reinicie

    # Ángulos iniciales
    current_pan_angle = 90
    current_tilt_angle = 90

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = face_detection.process(frame_rgb)

        if result.detections:
            for detection in result.detections:
                # Obtener el cuadro delimitador del rostro
                bboxC = detection.location_data.relative_bounding_box
                x_center = int((bboxC.xmin + bboxC.width / 2) * frame.shape[1])
                y_center = int((bboxC.ymin + bboxC.height / 2) * frame.shape[0])

                # Calcular los ángulos para los servos
                current_pan_angle, current_tilt_angle = calculate_angles(x_center, y_center, frame.shape[1], frame.shape[0], current_pan_angle, current_tilt_angle)

                # Enviar los comandos al Arduino
                send_command(f"PAN{current_pan_angle}")
                send_command(f"TILT{current_tilt_angle}")

                # Dibujar el cuadro delimitador en el frame
                start_point = (int(bboxC.xmin * frame.shape[1]), int(bboxC.ymin * frame.shape[0]))
                end_point = (int((bboxC.xmin + bboxC.width) * frame.shape[1]), int((bboxC.ymin + bboxC.height) * frame.shape[0]))
                cv2.rectangle(frame, start_point, end_point, (255, 0, 0), 2)

                # Dibujar el punto central
                cv2.circle(frame, (x_center, y_center), 5, (0, 255, 0), -1)

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
