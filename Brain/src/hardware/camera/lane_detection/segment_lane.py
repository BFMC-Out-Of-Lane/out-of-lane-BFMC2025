import numpy as np
from ultralytics import YOLO
# from src.hardware.camera.lane_detection.MODs_seg.modelMOD import YOLO

import cv2
import os
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
# pip install scikit-learn

MAX_ALTURA_Y = 390
# Cargar el modelo YOLO
model = YOLO("src/hardware/camera/lane_detection/yolo11n-seg-lane.pt")  # Ruta al modelo personalizado

def process_frame_with_midpoint(frame):
    """
    Procesa un frame para detectar carriles y calcula los valores requeridos.
    Args:
        frame: Imagen de entrada.
        traffic_sign: Información de señales de tráfico (opcional).
    Returns:
        processed_image: Imagen procesada con carriles dibujados.
        cross_tracking_error: Error de seguimiento transversal.
        deviation: Desviación calculada.
    """
    # Convertir el frame a escala de grises
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame_bgr = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)

    # Predecir con el modelo YOLO
    results = model(gray_frame_bgr, show=False, show_boxes=False, conf=0.5)

    # Procesar los resultados del modelo
    if results and len(results) > 0:
        for result in results:
            gray_frame_bgr = process_results(result, gray_frame_bgr)

    # Calcular el error de seguimiento transversal y la desviación
    cross_tracking_error = 0  # Lógica para calcular el error
    deviation = 0  # Lógica para calcular la desviación

    return gray_frame_bgr, cross_tracking_error, deviation


def fit_polynomial_ransac(points, degree=2, residual_threshold=5):
    """
    Ajusta un polinomio de grado `degree` usando RANSAC para ignorar outliers.
    residual_threshold: distancia máxima punto-curva para considerarlo inlier.
    Devuelve máscara de inliers y modelo ajustado.
    """
    x = points[:, 0].reshape(-1, 1)
    y = points[:, 1]
    model = make_pipeline(PolynomialFeatures(degree), RANSACRegressor(residual_threshold=residual_threshold))
    model.fit(x, y)
    inlier_mask = model.named_steps['ransacregressor'].inlier_mask_
    return inlier_mask, model


def draw_curve(points, color, gray_frame_bgr):
    """Dibuja una curva polinómica ajustada a los puntos."""
    filtered_points = [point for point in points if point[1] >= MAX_ALTURA_Y]

    # Draw all filtered points in yellow
    for point in filtered_points:
        x, y = int(point[0]), int(point[1])
        cv2.circle(gray_frame_bgr, (x, y), radius=3, color=(0, 255, 255), thickness=-1)  # Yellow points

    print(f"Length of filtered points: {len(filtered_points)}")
    if len(filtered_points) > 5:
        pts = np.array(filtered_points, dtype=np.float32)
        mask, model = fit_polynomial_ransac(pts, degree=2, residual_threshold=4)
        inliers = pts[mask]
        if inliers.size > 0:
            x_min, x_max = inliers[:, 0].min(), inliers[:, 0].max()
            x_curve = np.linspace(x_min, x_max, 200)
            y_curve = model.predict(x_curve.reshape(-1, 1))
            curve = np.column_stack((x_curve, y_curve)).astype(np.int32)
            cv2.polylines(gray_frame_bgr, [curve], isClosed=False, color=color, thickness=5)
    return gray_frame_bgr

def calculate_max_min(points):
    x_values = [point[0] for point in points]
    y_values = [point[1] for point in points]

    # max_x = max(x_values)
    # min_x = min(x_values)
    max_y = max(y_values)
    # min_y = min(y_values)

    # Encontrar las posiciones de los valores máximos y mínimos
    # max_x_index = x_values.index(max_x)
    # min_x_index = x_values.index(min_x)
    max_y_index = y_values.index(max_y)
    # min_y_index = y_values.index(min_y)

    x_value_max_y = x_values[max_y_index]

    # return max_x, min_x, max_y, min_y
    return x_value_max_y


def process_results(results, gray_frame_bgr):
    """
    Procesa los resultados del modelo, selecciona las máscaras centrales y dibuja curvas.
    """
    x_value_max_y_0 = None
    x_value_max_y_1 = None

    if hasattr(results.masks, 'xy') and len(results.masks.xy) > 0:
        height, width, _ = gray_frame_bgr.shape
        center_x = width // 2
        center_y = (3 * height) // 4  # Set the center height to 3/4 of the frame's height

        distances = []
        valid_masks_with_points_indices = []
        for idx, mask_xy in enumerate(results.masks.xy):
            if len(mask_xy) > 0:
                avg_x = np.mean([point[0] for point in mask_xy])
                distance = abs(avg_x - center_x)
                distances.append(distance)
                valid_masks_with_points_indices.append(idx)
            else:
                distances.append(float('inf'))

        num_valid_masks = len(valid_masks_with_points_indices)
        num_to_keep = min(2, num_valid_masks)

        x_value_max_y_0 = None
        x_value_max_y_1 = None
        if num_valid_masks > 0:
            # Obtener los índices de las máscaras válidas más cercanas al centro
            closest_valid_indices_in_distances = np.argsort(distances)[:num_to_keep]
            closest_original_indices = [valid_masks_with_points_indices[i] for i in closest_valid_indices_in_distances]

            print("----------------------LANE-SEGMENT-------------------------")

            print(f"Number of original masks: {len(results.masks.xy)}")
            print(f"Distances to center for valid masks: {[distances[i] for i in closest_valid_indices_in_distances]}")
            print(f"Indices of closest original masks: {closest_original_indices}")

            for i_orig in closest_original_indices:
                mask_xy = results.masks.xy[i_orig]
                filtered_points = [pointxy for pointxy in mask_xy if pointxy[1] >= MAX_ALTURA_Y]
                # print(f"Filtered points for original index {i_orig}: {filtered_points}")
                if len(filtered_points) > 20:
                    # Calculate the x value of the maximum y point
                    if i_orig % 2 == 0:
                        x_value_max_y_0 = calculate_max_min(filtered_points)
                    else:
                        x_value_max_y_1 = calculate_max_min(filtered_points)

            # CASO AMBAS LINEAS
            # Ensure both x_value_max_y_0 and x_value_max_y_1 are not None before comparison
            if x_value_max_y_0 is not None and x_value_max_y_1 is not None:
                lane_position_0 = "Left" if x_value_max_y_0 < x_value_max_y_1 else "Right"
                print(f"Lane position 0: {lane_position_0}")
                lane_position_1 = "Right" if x_value_max_y_0 < x_value_max_y_1 else "Left"
                print(f"Lane position 1: {lane_position_1}")

                # Assign colors based on lane positions
                color_0 = (255, 0, 0) if lane_position_0 == "Left" else (0, 255, 0)  # Blue for Left, Green for Right
                color_1 = (255, 0, 0) if lane_position_1 == "Left" else (0, 255, 0)  # Blue for Left, Green for Right

                # Draw curves with the assigned colors
                for i_orig in closest_original_indices:
                    mask_xy = results.masks.xy[i_orig]
                    filtered_points = [pointxy for pointxy in mask_xy if pointxy[1] >= MAX_ALTURA_Y]
                    if len(filtered_points) > 100:
                        if i_orig % 2 == 0:
                            gray_frame_bgr = draw_curve(filtered_points, color=color_0, gray_frame_bgr=gray_frame_bgr)
                        else:
                            gray_frame_bgr = draw_curve(filtered_points, color=color_1, gray_frame_bgr=gray_frame_bgr)
            else:
                print("Insufficient data to determine lane positions.")

            if x_value_max_y_0 is not None and x_value_max_y_1 is not None:
                midpoint_x = int((x_value_max_y_0 + x_value_max_y_1) / 2)
                midpoint_y = int(center_y)
                cv2.circle(gray_frame_bgr, (midpoint_x, midpoint_y), radius=20, color=(0, 0, 255),
                           thickness=-1)
            elif x_value_max_y_0 is not None:
                distance_from_curve = 50  # Distancia fija desde la curva
                if x_value_max_y_0 < center_x:  # Curva izquierda
                    circle_x = int(x_value_max_y_0 + distance_from_curve)
                else:  # Curva derecha
                    circle_x = int(x_value_max_y_0 - distance_from_curve)
                cv2.circle(gray_frame_bgr, (circle_x, int(center_y)), radius=20, color=(0, 0, 255),
                           thickness=-1)
            elif x_value_max_y_1 is not None:
                distance_from_curve = 50  # Distancia fija desde la curva
                if x_value_max_y_1 < center_x:  # Curva izquierda
                    circle_x = int(x_value_max_y_1 + distance_from_curve)
                else:  # Curva derecha
                    circle_x = int(x_value_max_y_1 - distance_from_curve)
                cv2.circle(gray_frame_bgr, (circle_x, int(center_y)), radius=20, color=(0, 0, 255),
                           thickness=-1)
            else:
                print("Insufficient data to determine circle position.")

    # Si no hay puntos válidos, dibujar el círculo en el centro de la imagen
    if x_value_max_y_0 is None and x_value_max_y_1 is None:
        height, width, _ = gray_frame_bgr.shape
        center_x = width // 2
        center_y = height // 2  # Centro de la imagen
        cv2.circle(gray_frame_bgr, (center_x, center_y), radius=20, color=(0, 0, 255), thickness=-1)
        # print("Red circle drawn at the center of the image.")

    return gray_frame_bgr
