#!/usr/bin/env python3
"""
Create ROS map from SVG field layout.
Extracts brown walls (#dfab4c) from the SVG and converts to PGM format.
"""

import os
import xml.etree.ElementTree as ET

import numpy as np


def create_map_from_svg(svg_path, output_base):
    # Parse SVG
    tree = ET.parse(svg_path)
    root = tree.getroot()

    # SVG namespace
    ns = {"svg": "http://www.w3.org/2000/svg"}

    # SVG dimensions (in pixels, 1px = 1cm in this case)
    svg_width_px = 350  # 3.5m
    svg_height_px = 700  # 7m

    # ROS map parameters
    resolution = 0.05  # 50mm per pixel
    resolution_mm = resolution * 1000

    field_width_mm = 3500
    field_height_mm = 7000
    margin_mm = 1000

    # Calculate map size
    width_px = int((field_width_mm + 2 * margin_mm) / resolution_mm)
    height_px = int((field_height_mm + 2 * margin_mm) / resolution_mm)

    print(f"Creating map from SVG: {width_px}x{height_px} px")

    # Create image (254 = free, 0 = wall)
    img = np.full((height_px, width_px), 254, dtype=np.uint8)

    def svg_to_map_coords(svg_x, svg_y):
        """Convert SVG coordinates to map pixel coordinates"""
        # SVG viewBox: "0 0 92.604165 185.20833" (in mm, actually represents cm)
        # Real field: 3500mm x 7000mm = 350cm x 700cm
        # SVG scale: 92.604165 SVG units = 350cm = 3500mm
        # So: 1 SVG unit = 3500mm / 92.604165 = 37.795 mm/unit

        scale_factor = field_width_mm / 92.604165  # mm per SVG unit

        mm_x = svg_x * scale_factor
        mm_y = svg_y * scale_factor

        # Convert to map pixel coordinates
        map_x = int((mm_x + margin_mm) / resolution_mm)
        # Flip Y axis: SVG top-left origin to map bottom-left origin
        map_y = int((field_height_mm + margin_mm - mm_y) / resolution_mm)

        return map_x, map_y

    def draw_wall(svg_x, svg_y, svg_w, svg_h):
        """Draw a wall rectangle on the map"""
        scale_factor = field_width_mm / 92.604165

        x_px, y_top_px = svg_to_map_coords(svg_x, svg_y)
        _, y_bottom_px = svg_to_map_coords(svg_x, svg_y + svg_h)

        w_px = int(svg_w * scale_factor / resolution_mm)
        h_px = abs(y_bottom_px - y_top_px)

        # Clip to image bounds
        x_end = min(x_px + w_px, width_px)
        y_end = min(y_top_px + h_px, height_px)
        x_px = max(0, x_px)
        y_top_px = max(0, y_top_px)

        img[y_top_px:y_end, x_px:x_end] = 0  # Black = wall

    # Find all rect elements with brown fill (#dfab4c)
    brown_color = "#dfab4c"
    wall_count = 0

    # Only include outer frame walls
    allowed_walls = {"外枠_左", "外枠_上", "外枠_右", "外枠_下"}

    for rect in root.iter("{http://www.w3.org/2000/svg}rect"):
        style = rect.get("style", "")
        fill = rect.get("fill", "")

        # Check if this is a brown wall
        if brown_color in style or brown_color in fill:
            label = rect.get(
                "{http://www.inkscape.org/namespaces/inkscape}label", "unknown"
            )

            # Skip walls not in the allowed list
            if label not in allowed_walls:
                print(f"  Skipping wall: {label}")
                continue

            try:
                x = float(rect.get("x", 0))
                y = float(rect.get("y", 0))
                w = float(rect.get("width", 0))
                h = float(rect.get("height", 0))

                print(f"  Drawing wall: {label} ({x:.2f}, {y:.2f}, {w:.2f}x{h:.2f})")

                draw_wall(x, y, w, h)
                wall_count += 1
            except ValueError as e:
                print(f"  Warning: Failed to parse rect: {e}")

    print(f"✓ Drew {wall_count} walls from SVG")

    # Write PGM file
    pgm_path = output_base + ".pgm"
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{width_px} {height_px}\n255\n".encode())
        f.write(img.tobytes())

    print(f"✓ Saved: {pgm_path}")

    # Write YAML file
    yaml_path = output_base + ".yaml"
    with open(yaml_path, "w") as f:
        f.write(f"image: {os.path.basename(pgm_path)}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{-margin_mm / 1000:.1f}, {-margin_mm / 1000:.1f}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")

    print(f"✓ Saved: {yaml_path}")
    print("\nMap generated from SVG walls successfully!")


if __name__ == "__main__":
    # Path to SVG file
    svg_file = (
        "/home/host1/repos/khr2026_team1_bt_controller/src/assets/khr2026_field.svg"
    )

    # Output directory
    output_dir = os.path.join(os.path.dirname(__file__), "../maps")
    os.makedirs(output_dir, exist_ok=True)

    # Generate map
    create_map_from_svg(svg_file, os.path.join(output_dir, "field"))
