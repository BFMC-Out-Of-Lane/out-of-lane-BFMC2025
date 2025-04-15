import cv2
import networkx as nx

# PARAMETERS
GRAPH_PATH = 'racetrack/test_track.graphml'
IMG_PATH = 'racetrack/test_track.png'
PIXELS_PER_METER = 1600 / 6  # ~266.67
VISUAL_SCALE = 0.5  # Visual downscale factor (for display only)
RADIUS = 2  # Radius in pixels for node circles

if __name__ == '__main__':
    # Load the graph from the .graphml file
    G = nx.read_graphml(GRAPH_PATH)

    # Read the original image using OpenCV
    image = cv2.imread(IMG_PATH)

    if image is None:
        print("Error: Could not load image.")
        exit()

    # Draw red circles at each node position (in pixel coordinates)
    for node in G.nodes(data=True):
        x = int(float(node[1]['x']) * PIXELS_PER_METER)
        y = int(float(node[1]['y']) * PIXELS_PER_METER)
        cv2.circle(image, (x, y), RADIUS, (0, 0, 255), -1)  # BGR format: Red

    # Resize the image for display only (without affecting calculations)
    new_size = (
        int(image.shape[1] * VISUAL_SCALE),  # Width
        int(image.shape[0] * VISUAL_SCALE)   # Height
    )
    image_visual = cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)

    # Show the resized image with drawn nodes
    cv2.imshow('Graph Nodes', image_visual)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
