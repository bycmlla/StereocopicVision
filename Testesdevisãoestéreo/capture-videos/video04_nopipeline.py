import cv2

cap = cv2.VideoCapture(0) 

if not cap.isOpened():
    print("Erro: Não foi possível abrir a câmera.")
    exit()

ret, frame = cap.read()

if not ret:
    print("Erro: Não foi possível capturar a imagem.")
    exit()

cv2.imwrite("imagem_capturada.jpg", frame)

cap.release()

cv2.destroyAllWindows()
