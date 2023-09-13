import cv2

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()

    if ret:
        cv2.imshow('Camera', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()