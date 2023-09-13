import cv2

cap = cv2.VideoCapture("DISPLAY=:0.0 gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1' ! nvoverlaysink -e")

if not cap.isOpened():
    print("Erro: Não foi possível abrir a câmera.")
else:
    while True:
        ret, frame = cap.read()

        cv2.imshow('Camera', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
