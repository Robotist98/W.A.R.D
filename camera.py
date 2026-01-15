import cv2

def main():
    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        print(f"Error: cannot open camera")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            cv2.imshow("test", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC to quit
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()