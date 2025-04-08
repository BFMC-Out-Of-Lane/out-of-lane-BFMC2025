from networkx import graphml
from PIL import Image, ImageDraw

if __name__ == '__main__':
    G = graphml.read_graphml('racetrack/test_track.graphml')
    image = Image.open('racetrack/test_track.png')
    factor = 1600 / 6
    draw = ImageDraw.Draw(image)
    radius = 2

    for node in G.nodes(data=True):
        x = node[1]['x'] * factor
        y = node[1]['y'] * factor
        left_up_point = (x - radius, y - radius)
        right_down_point = (x + radius, y + radius)
        draw.ellipse([left_up_point, right_down_point], fill='red', outline='red')

    image.show()
