#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import math

def create_pgm_map():
    svg_path = "/home/user/repos/robot/khr2026_team1_bt_controller/src/assets/khr2026_field.svg"
    pgm_path = "/home/user/repos/robot/khr2026_team1_rspi/src/robot_control/maps/field.pgm"
    yaml_path = "/home/user/repos/robot/khr2026_team1_rspi/src/robot_control/maps/field.yaml"

    tree = ET.parse(svg_path)
    root = tree.getroot()

    # Get viewBox dimensions to calculate scale
    # viewBox="0 0 92.604165 185.20833"
    viewBox = root.attrib['viewBox'].split()
    vb_w = float(viewBox[2])
    vb_h = float(viewBox[3])

    # Target Map Properties
    # Field Size: 3.5m x 7.0m
    FIELD_W_M = 3.5
    FIELD_H_M = 7.0
    RESOLUTION = 0.01 # 1cm/pixel => 350x700 pixels
    
    IMG_W = int(FIELD_W_M / RESOLUTION)
    IMG_H = int(FIELD_H_M / RESOLUTION)

    print(f"Generating Map: {IMG_W}x{IMG_H} pixels")

    # Scale factors: SVG units -> Map Pixels
    scale_x = IMG_W / vb_w
    scale_y = IMG_H / vb_h

    # 0 = Black (Occupied), 254 = White (Free), 205 = Grey (Unknown)
    # Initialize with White (Free Space)
    grid = [[254 for _ in range(IMG_W)] for _ in range(IMG_H)]

    # Namespaces usually present in Inkscape SVGs
    ns = {'svg': 'http://www.w3.org/2000/svg'}

    # Target Layers and IDs
    # layer1: Outer Frame (rect1-4)
    # layer2: Wood Walls (rect5-13)
    target_ids = [
        # Outer Frame
        'rect1', 'rect2', 'rect3', 'rect4',
        # Wood Walls
        'rect5', 'rect6', 'rect7', 'rect8', 'rect9', 'rect10', 'rect11', 'rect12', 'rect13'
    ]

    def draw_rect(x, y, w, h):
        # Convert SVG coords to Pixel coords
        px_x = int(x * scale_x)
        px_y = int(y * scale_y)
        px_w = int(math.ceil(w * scale_x))
        px_h = int(math.ceil(h * scale_y))

        # Fill rectangle with Black (0)
        for r in range(px_y, min(px_y + px_h, IMG_H)):
            for c in range(px_x, min(px_x + px_w, IMG_W)):
                if 0 <= r < IMG_H and 0 <= c < IMG_W:
                    grid[r][c] = 0

    # Find all rects recursively
    for rect in root.findall('.//{http://www.w3.org/2000/svg}rect'):
        rect_id = rect.attrib.get('id')
        if rect_id in target_ids:
            x = float(rect.attrib['x'])
            y = float(rect.attrib['y'])
            w = float(rect.attrib['width'])
            h = float(rect.attrib['height'])
            
            print(f"Processing {rect_id}: x={x}, y={y}, w={w}, h={h}")
            draw_rect(x, y, w, h)

    # Write PGM (P2 format - ASCII)
    with open(pgm_path, 'w') as f:
        f.write("P2\n")
        f.write(f"{IMG_W} {IMG_H}\n")
        f.write("255\n")
        for row in grid:
            f.write(" ".join(map(str, row)) + "\n")
    
    print(f"Written PGM to {pgm_path}")

    # Update YAML
    yaml_content = f"""image: field.pgm
resolution: {RESOLUTION}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"Updated YAML at {yaml_path}")

if __name__ == "__main__":
    create_pgm_map()
