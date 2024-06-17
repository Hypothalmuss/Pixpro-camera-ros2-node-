import cv2

def capture_calibration_images(width=640, height=480):
    cap = cv2.VideoCapture(0)  # Open the camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    counter = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow('Calibration Image Capture', frame)
        key = cv2.waitKey(1)
        if key == ord('c'):
            cv2.imwrite(f'calibration_image_{counter}.png', frame)
            counter += 1
            print(f'Captured calibration_image_{counter}.png')
        elif key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    capture_calibration_images()

