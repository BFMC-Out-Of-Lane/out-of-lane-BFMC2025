import cv2

# PARAMETERS
IMG_PATH = 'racetrack/test_track.png'  
PIXELS_PER_METER = 1600 / 6  # ~266.67
SCALE = 0.5  # Visual scaling, without affecting calculations
radius = 4

# Load original image
img = cv2.imread(IMG_PATH)

if img is None:
    print("The image could not be loaded. Please check the path.")
    exit()

# Create resized image for display only
width = int(img.shape[1] * SCALE)
height = int(img.shape[0] * SCALE)
img_view = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

# clic event
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Actual coordinates in the original image (pixels)
        x_real = int(x / SCALE)
        y_real = int(y / SCALE)

        # Conversion to meters
        x_m = x_real / PIXELS_PER_METER
        y_m = y_real / PIXELS_PER_METER

        # Show coordinates pixels and meters
        print(f"Pixel -> X: {x_real}, Y: {y_real} / En metros- X: {x_m:.2f} m, Y: {y_m:.2f} m ")
        print(f"Pixel -> X: {x_real}, Y: {y_real} / En metros- X: {x_m:.2f} m, Y: {6 - y_m:.2f} m ")

        # Draw point on copy of viewed image
        img_copy = img_view.copy()
        cv2.circle(img_copy, (x, y), radius, (0, 0, 255), -1)
        cv2.imshow("Pista", img_copy)

# Display scaled image and capture clicks
cv2.imshow("Pista", img_view)
cv2.setMouseCallback("Pista", click_event, img_view)

print("Hacé click sobre la imagen para obtener coordenadas. Presioná 'q' para salir.")

while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

