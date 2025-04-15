import cv2
import networkx as nx
import json
import argparse

# Argument parser
parser = argparse.ArgumentParser(description='Visualizar grafo sobre imagen.')
parser.add_argument('--test', type=str, required=True,
                    help='Nombre del test: "r", "h" o "bosch"')
args = parser.parse_args()
test = args.test
# --test h -> track with highway
# --test r -> track with roundabout
# --test bosch -> competition track

with open(r'.\configTest.json', 'r') as f:
    configs = json.load(f)

config = configs[str(test)] 

# dinamic parameters
GRAPH_PATH = config['GRAPH_PATH']
IMG_PATH = config['IMG_PATH']
IMAGE_WIDTH_METERS = config['IMAGE_WIDTH_METERS']
IMAGE_HEIGHT_METERS = config['IMAGE_HEIGHT_METERS'] 

# CONSTANT PARAMETERS
VISUAL_SCALE = 0.5  # Visual downscale factor (for display only)
RADIUS = 4  # Radius in pixels for node circles

if __name__ == '__main__':
    
    G = nx.read_graphml(GRAPH_PATH) # Load the graph 
    image = cv2.imread(IMG_PATH) # Read the original 

    if image is None:
        print("Error: Could not load image.")
        exit()
    
    image_width_pixels = image.shape[1]
    image_height_pixels = image.shape[0]

    pixels_per_meter_x = image_width_pixels / IMAGE_WIDTH_METERS
    pixels_per_meter_y = image_height_pixels / IMAGE_HEIGHT_METERS

    # Draw red circles at each node position (in pixel coordinates)
    for node in G.nodes(data=True):
        x = int(float(node[1]['x']) * pixels_per_meter_x)
        y = int((IMAGE_HEIGHT_METERS - float(node[1]['y'])) * pixels_per_meter_y)  # Flip Y axis
        cv2.circle(image, (x, y), RADIUS, (0, 0, 255), -1)  
        cv2.putText(image, str(node[0]), (x + RADIUS + 2, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (255, 255, 255), 1, cv2.LINE_AA)

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
