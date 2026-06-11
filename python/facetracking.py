import cv2
from cvzone.FaceDetectionModule import FaceDetector
# import pyfirmata  <-- COMENTADO
import numpy as np
import socket

# --- Configurações de Rede para Comunicação com o ESP32 de controle ---
ESP32_IP = "192.168.1.106"
ESP32_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

cap = cv2.VideoCapture("http://esp32cam.local:81/stream", cv2.CAP_FFMPEG)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

# --- Configuração do Arduino Comentada ---
# port = "COM7"
# board = pyfirmata.Arduino(port)
# servo_pinX = board.get_pin('d:9:s') #pin 9 Arduino
# servo_pinY = board.get_pin('d:10:s') #pin 10 Arduino

detector = FaceDetector()
servoPos = [90, 90] # posição inicial (pode manter para visualização)

while True:
    success, img = cap.read()
    if not success or img is None:
        continue

    # Pega a resolução real da imagem que veio da câmera
    hs, ws, _ = img.shape
    # Define o centro da tela baseado na imagem atual
    centerX, centerY = ws // 2, hs // 2
        
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        # Pega as coordenadas centrais do rosto detectado
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        
        # Converte coordenadas para graus (0 a 180) baseando-se no tamanho real da imagem
        servoX = np.interp(fx, [0, ws], [0, 180])
        servoY = np.interp(fy, [0, hs], [0, 180])

        servoPos[0] = int(np.clip(servoX, 0, 180))
        servoPos[1] = int(np.clip(servoY, 0, 180))

        # --- Envia as coordenadas do rosto para o ESP32 de controle via UDP ---
        erroX = fx - centerX
        erroY = fy - centerY
        message = f"{erroX},{erroY}"
        sock.sendto(message.encode(), (ESP32_IP, ESP32_PORT))

        # --- DESENHOS COM ALVO ---
        # Círculo externo e ponto central
        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        
        # Coordenadas numéricas próximas ao rosto
        cv2.putText(img, f"[{fx}, {fy}]", (fx + 15, fy - 15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        
        # Linhas de mira (cruz) seguindo o rosto
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # Linha Horizontal
        cv2.line(img, (fx, 0), (fx, hs), (0, 0, 0), 2)  # Linha Vertical
        
        # Status (posicionado proporcionalmente à direita)
        cv2.putText(img, "ALVO NA MIRA", (ws - 300, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 3)

    else:
        # --- DESENHOS SEM ALVO (MIRA NO CENTRO) ---
        cv2.putText(img, "SEM ALVO", (ws - 250, 40), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 3)
        cv2.circle(img, (centerX, centerY), 80, (0, 0, 255), 2)
        cv2.circle(img, (centerX, centerY), 15, (0, 0, 255), cv2.FILLED)
        
        # Linhas de mira fixas no centro
        cv2.line(img, (0, centerY), (ws, centerY), (0, 0, 0), 2)
        cv2.line(img, (centerX, 0), (centerX, hs), (0, 0, 0), 2)

    # Exibe os graus dos Servos no canto superior esquerdo
    cv2.putText(img, f'Servo X: {servoPos[0]} deg', (20, 40), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {servoPos[1]} deg', (20, 80), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
    
    # --- Escrita nos Servos Comentada ---
    # servo_pinX.write(servoPos[0])
    # servo_pinY.write(servoPos[1])

    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'): # Pressione 'q' para sair
        break

cap.release()
cv2.destroyAllWindows()