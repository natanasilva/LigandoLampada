import cv2
import numpy as np
import serial
import time

# Configuração da porta serial
arduino = serial.Serial('COM5', 9600)  # Substitua 'COM5' pela sua porta serial
time.sleep(2)  # Atraso para a conexão ser estabelecida

# Ponto fixo na tela (retângulo na parte superior esquerda)
fixed_rect_start = (50, 50)  # Canto superior esquerdo do retângulo
fixed_rect_end = (150, 150)  # Canto inferior direito do retângulo

# Função para detectar a mão e seus contornos
def detect_hand_movement(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definindo intervalo de cor para a pele
    lower_skin = np.array([0, 20, 70], dtype=np.uint8)
    upper_skin = np.array([20, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_skin, upper_skin)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500:
            # Retorna o contorno completo da mão para desenhar todos os pontos
            return largest_contour
    return None  # Retorna None se não houver contornos

# Captura de vídeo
cap = cv2.VideoCapture(0)
relay_active = False  # Estado do relé

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detectar a mão e seu contorno
        hand_contour = detect_hand_movement(frame)

        # Desenhar o ponto fixo (retângulo) na tela
        cv2.rectangle(frame, fixed_rect_start, fixed_rect_end, (0, 255, 0), 2)  # Retângulo verde

        if hand_contour is not None:
            # Desenhar o contorno completo da mão
            cv2.drawContours(frame, [hand_contour], -1, (255, 0, 0), 2)  # Contorno da mão em azul

            # Obter o ponto central do contorno para verificação de posição
            x, y, w, h = cv2.boundingRect(hand_contour)
            hand_center = (x + w // 2, y + h // 2)

            # Verifica se o centro da mão está dentro do retângulo
            if (fixed_rect_start[0] < hand_center[0] < fixed_rect_end[0] and
                fixed_rect_start[1] < hand_center[1] < fixed_rect_end[1]):
                if not relay_active:
                    print("Mão detectada no retângulo! Acionando o relé.")
                    arduino.write(b'1')  # Envia '1' para o Arduino para ligar o relé
                    relay_active = True  # Marca o relé como ativado
            else:
                if relay_active:
                    print("Mão fora do retângulo! Desligando o relé.")
                    arduino.write(b'0')  # Envia '0' para o Arduino para desligar o relé
                    relay_active = False  # Marca o relé como desativado

        cv2.imshow('Frame', frame)

        # Verifica se a tecla 'q' foi pressionada
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
