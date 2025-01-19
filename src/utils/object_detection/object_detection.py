from ultralytics import YOLO
import cv2

if __name__ == "__main__":
    model = YOLO("yolo11n.pt")

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Objects to look for: traffic light, stop sign
        results = model(frame, conf=0.5, classes=[9, 11])

        annotated_frame = results[0].plot()

        cv2.imshow("YOLOv11 Detection", annotated_frame)

        # 'q' to stop
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
