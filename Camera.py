
import cv2
from pyzbar.pyzbar import decode

# Open the USB camera (0 is usually the default camera, change if needed)
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Scanning QR codes. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image")
        break

    # Decode QR codes in the frame
    qr_codes = decode(frame)

    for qr_code in qr_codes:
        qr_data = qr_code.data.decode("utf-8")
        print("QR Code:", qr_data)

    # Display the frame (optional, useful for debugging)
    cv2.imshow("QR Scanner", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
