#!/usr/bin/env python3
"""
Generate PDF cut list for LifeTrac v25 structural steel parts.
Creates one page per unique angle iron and square tubing part with:
- Engineering drawing showing dimensions and hole positions
- List of cutting and drilling operations with checkboxes
- Part code and quantity needed
- OpenSCAD rendered views (top, side, end, isometric)
"""

import sys
import os
import subprocess
import tempfile
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch, mm
from reportlab.pdfgen import canvas
from reportlab.lib import colors
from reportlab.platypus import Table, TableStyle
from reportlab.lib.utils import ImageReader
from PIL import Image


class StructuralPart:
    """Represents a structural steel part (angle iron or square tubing)"""
    
    def __init__(self, part_code, name, material, length_mm, quantity, holes=None, notes=""):
        self.part_code = part_code
        self.name = name
        self.material = material
        self.length_mm = length_mm
        self.length_inches = length_mm / 25.4
        self.quantity = quantity
        self.holes = holes or []
        self.notes = notes
        
        # Categorize holes by leg
        self.vertical_leg_holes = []
        self.horizontal_leg_holes = []
        for hole in self.holes:
            if 'vertical leg' in hole.get('description', '').lower():
                self.vertical_leg_holes.append(hole)
            elif 'horizontal leg' in hole.get('description', '').lower():
                self.horizontal_leg_holes.append(hole)
            elif 'other leg' in hole.get('description', '').lower():
                # For parts with holes in both legs but not specifically labeled
                self.horizontal_leg_holes.append(hole)
            else:
                # Default to vertical leg if not specified
                self.vertical_leg_holes.append(hole)
    
    def get_operations(self):
        """Generate list of cutting and drilling operations"""
        operations = []
        
        # Cutting operation
        operations.append({
            "description": f"Cut {self.material} to {self.length_inches:.2f}\" ({self.length_mm:.1f} mm)",
            "checked": False
        })
        
        # Drilling operations
        for i, hole in enumerate(self.holes, 1):
            position_inches = hole["position_mm"] / 25.4
            diameter_inches = hole["diameter_mm"] / 25.4
            # Convert diameter to fraction
            frac = self.decimal_to_fraction(diameter_inches)
            operations.append({
                "description": f"Drill {frac}\" hole at {position_inches:.2f}\" ({hole['position_mm']:.1f} mm) - {hole['description']}",
                "checked": False
            })
        
        return operations
    
    @staticmethod
    def decimal_to_fraction(decimal):
        """Convert decimal inches to common fraction (public utility method)"""
        # Common drill bit sizes
        fractions = {
            0.125: "1/8",
            0.1875: "3/16",
            0.25: "1/4",
            0.3125: "5/16",
            0.375: "3/8",
            0.4375: "7/16",
            0.5: "1/2",
            0.5625: "9/16",
            0.625: "5/8",
            0.75: "3/4",
            0.875: "7/8",
            1.0: "1"
        }
        
        # Find closest fraction
        closest = min(fractions.keys(), key=lambda x: abs(x - decimal))
        if abs(closest - decimal) < 0.01:  # Within 0.01"
            return fractions[closest]
        return f"{decimal:.3f}"


def draw_angle_iron_drawing(c, part, x_start, y_start, width, height):
    """Draw engineering drawing for angle iron part"""
    # Scale to fit drawing area
    scale = min(width / (part.length_mm + 100), height / 150)
    
    # Center the drawing
    drawing_width = part.length_mm * scale
    x_offset = x_start + (width - drawing_width) / 2
    y_center = y_start + height / 2
    
    # Draw angle iron profile (L-shape) - side view
    leg_size_mm = 50.8  # 2" angle iron
    thick_mm = 6.35     # 1/4" thickness
    
    # Draw main length line
    c.setStrokeColor(colors.black)
    c.setLineWidth(2)
    c.line(x_offset, y_center, x_offset + drawing_width, y_center)
    
    # Cross-section profile removed per user request
    
    # Draw dimension line for total length (blue)
    dim_y = y_center - 40
    c.setStrokeColor(colors.blue)
    c.setLineWidth(0.5)
    # Dimension line
    c.line(x_offset, dim_y, x_offset + drawing_width, dim_y)
    # End markers
    c.line(x_offset, dim_y - 5, x_offset, dim_y + 5)
    c.line(x_offset + drawing_width, dim_y - 5, x_offset + drawing_width, dim_y + 5)
    # Dimension text
    c.setFillColor(colors.blue)
    c.setFont("Helvetica", 8)
    dim_text = f"{part.length_inches:.2f}\" ({part.length_mm:.1f} mm)"
    text_width = c.stringWidth(dim_text, "Helvetica", 8)
    c.drawString(x_offset + (drawing_width - text_width) / 2, dim_y - 15, dim_text)
    
    # Draw holes and dimension lines for vertical leg (red)
    if part.vertical_leg_holes:
        dim_y_vertical = y_center + 30
        c.setStrokeColor(colors.red)
        c.setLineWidth(0.5)
        
        for hole in part.vertical_leg_holes:
            hole_x = x_offset + (hole["position_mm"] * scale)
            hole_radius = (hole["diameter_mm"] * scale) / 2
            
            # Draw hole
            c.setStrokeColor(colors.red)
            c.setFillColor(colors.white)
            c.setLineWidth(1.5)
            c.circle(hole_x, y_center, hole_radius, fill=1, stroke=1)
            
            # Draw dimension line from start to hole
            c.setStrokeColor(colors.red)
            c.setLineWidth(0.5)
            c.line(x_offset, dim_y_vertical, hole_x, dim_y_vertical)
            c.line(x_offset, dim_y_vertical - 3, x_offset, dim_y_vertical + 3)
            c.line(hole_x, dim_y_vertical - 3, hole_x, dim_y_vertical + 3)
            
            # Dimension text
            c.setFillColor(colors.red)
            c.setFont("Helvetica", 7)
            hole_pos_text = f"{hole['position_mm']:.1f}"
            c.drawString(hole_x - 10, dim_y_vertical + 5, hole_pos_text)
        
        # Add label for vertical leg
        c.setFont("Helvetica-Oblique", 7)
        c.drawString(x_offset, dim_y_vertical + 18, "Vertical Leg Holes")
    
    # Draw holes and dimension lines for horizontal leg (green)
    if part.horizontal_leg_holes:
        dim_y_horizontal = y_center + 55
        
        for hole in part.horizontal_leg_holes:
            hole_x = x_offset + (hole["position_mm"] * scale)
            hole_radius = (hole["diameter_mm"] * scale) / 2
            
            # Draw hole with green circle
            c.setStrokeColor(colors.green)
            c.setFillColor(colors.white)
            c.setLineWidth(1.5)
            c.circle(hole_x, y_center, hole_radius, fill=1, stroke=1)
            
            # Draw dimension line from start to hole
            c.setStrokeColor(colors.green)
            c.setLineWidth(0.5)
            c.line(x_offset, dim_y_horizontal, hole_x, dim_y_horizontal)
            c.line(x_offset, dim_y_horizontal - 3, x_offset, dim_y_horizontal + 3)
            c.line(hole_x, dim_y_horizontal - 3, hole_x, dim_y_horizontal + 3)
            
            # Dimension text
            c.setFillColor(colors.green)
            c.setFont("Helvetica", 7)
            hole_pos_text = f"{hole['position_mm']:.1f}"
            c.drawString(hole_x - 10, dim_y_horizontal + 5, hole_pos_text)
        
        # Add label for horizontal leg
        c.setFont("Helvetica-Oblique", 7)
        c.setFillColor(colors.green)
        c.drawString(x_offset, dim_y_horizontal + 18, "Horizontal Leg Holes")
    
    # Draw material callout
    c.setFillColor(colors.black)
    c.setFont("Helvetica-Bold", 10)
    c.drawString(x_start, y_start + height + 10, f"Material: {part.material}")


def draw_operations_checklist(c, part, x_start, y_start, width, height):
    """Draw operations checklist with checkboxes"""
    operations = part.get_operations()
    
    # Title
    c.setFont("Helvetica-Bold", 11)
    c.setFillColor(colors.black)
    c.drawString(x_start, y_start + height - 20, "Manufacturing Operations:")
    
    # Draw checkboxes and operations
    y = y_start + height - 45
    checkbox_size = 10
    
    c.setFont("Helvetica", 9)
    for i, op in enumerate(operations, 1):
        # Draw checkbox
        c.setStrokeColor(colors.black)
        c.setLineWidth(1)
        c.rect(x_start, y - checkbox_size, checkbox_size, checkbox_size, fill=0, stroke=1)
        
        # Parse operation description to separate main action from position notes
        description = op['description']
        # Check if there's a position note (contains " - " or "at ")
        if ' - ' in description:
            main_part, detail_part = description.split(' - ', 1)
            # Draw main operation
            c.setFillColor(colors.black)
            c.drawString(x_start + checkbox_size + 5, y - 8, f"{i}. {main_part}")
            # Draw detail as bullet point
            y -= 15
            c.setFont("Helvetica", 8)
            c.setFillColor(colors.darkgray)
            c.drawString(x_start + checkbox_size + 15, y - 5, f"• {detail_part}")
            c.setFont("Helvetica", 9)
            y -= 5
        elif 'at ' in description and 'hole' in description.lower():
            # Split at "at" for hole positions
            parts = description.split(' at ', 1)
            if len(parts) == 2:
                main_part = parts[0]
                detail_part = f"at {parts[1]}"
                # Draw main operation
                c.setFillColor(colors.black)
                c.drawString(x_start + checkbox_size + 5, y - 8, f"{i}. {main_part}")
                # Draw detail as bullet point
                y -= 15
                c.setFont("Helvetica", 8)
                c.setFillColor(colors.darkgray)
                c.drawString(x_start + checkbox_size + 15, y - 5, f"• {detail_part}")
                c.setFont("Helvetica", 9)
                y -= 5
            else:
                # Draw as single line
                c.setFillColor(colors.black)
                c.drawString(x_start + checkbox_size + 5, y - 8, f"{i}. {description}")
        else:
            # Draw as single line
            c.setFillColor(colors.black)
            c.drawString(x_start + checkbox_size + 5, y - 8, f"{i}. {description}")
        
        y -= 20
        
        if y < y_start + 50:
            break
    
    # Add notes if any
    if part.notes and y > y_start + 60:
        y -= 15
        c.setFont("Helvetica-Oblique", 8)
        c.setFillColor(colors.darkblue)
        c.drawString(x_start, y, f"Notes: {part.notes}")


def generate_openscad_render(part_code, scad_file, camera_config, output_path, autocenter=True):
    """
    Generate PNG render from OpenSCAD file
    
    Args:
        part_code: Part code (A1, A2, etc.)
        scad_file: Path to OpenSCAD file
        camera_config: Dict with camera, rotation, projection settings
        output_path: Where to save the PNG
        autocenter: If True, use --autocenter and --viewall for better framing
    
    Returns:
        True if successful, False otherwise
    """
    try:
        # Check if openscad is available
        if subprocess.run(['which', 'openscad'], capture_output=True).returncode != 0:
            print(f"  ⚠️  OpenSCAD not found, skipping render for {part_code}")
            return False
        
        # Build OpenSCAD command
        cmd = [
            'openscad',
            '--render',
            '--imgsize=800,600',
            '--colorscheme=Tomorrow',
            f'--projection={camera_config.get("projection", "ortho")}',
        ]
        
        # Add camera or viewall
        if autocenter:
            cmd.extend([
                '--autocenter',
                '--viewall',
            ])
        
        # Add camera rotation
        if 'camera' in camera_config:
            cmd.append(f'--camera={camera_config["camera"]}')
        
        cmd.extend(['-o', output_path, scad_file])
        
        # Run with timeout
        result = subprocess.run(cmd, capture_output=True, timeout=30, env={'DISPLAY': ':99'})
        
        if result.returncode == 0 and os.path.exists(output_path):
            return True
        else:
            print(f"  ⚠️  OpenSCAD render failed for {part_code}: {result.stderr.decode()[:100]}")
            return False
            
    except subprocess.TimeoutExpired:
        print(f"  ⚠️  OpenSCAD render timeout for {part_code}")
        return False
    except Exception as e:
        print(f"  ⚠️  Error rendering {part_code}: {str(e)}")
        return False


def convert_to_grayscale_with_white_bg(image_path):
    """Convert image to grayscale with white background for easier printing"""
    try:
        img = Image.open(image_path)
        
        # Convert to RGBA to handle transparency
        if img.mode != 'RGBA':
            img = img.convert('RGBA')
        
        # Create white background
        white_bg = Image.new('RGBA', img.size, (255, 255, 255, 255))
        
        # Composite image onto white background
        composite = Image.alpha_composite(white_bg, img)
        
        # Convert to grayscale
        grayscale = composite.convert('L')
        
        # Save as grayscale
        grayscale.save(image_path)
        return True
    except Exception as e:
        print(f"  ⚠️  Error converting to grayscale: {str(e)}")
        return False


def generate_part_renders(part_code, scad_module_name, temp_dir, part=None):
    """
    Generate orthogonal and diagonal views of a part
    
    Args:
        part_code: Part code (A1, A2, T1, etc.)
        scad_module_name: Name of OpenSCAD module to render
        temp_dir: Temporary directory for outputs
        part: StructuralPart object with dimensions and hole data
    
    Returns:
        Dict with paths to rendered images, or None if rendering failed
    """
    # Map part codes to their dedicated SCAD files
    scad_file_map = {
        'A1': '../openscad/parts/structural/angle_iron_a1_back_outer_vertical.scad',
        'A2': '../openscad/parts/structural/angle_iron_a2_back_inner_vertical.scad',
        'A3': '../openscad/parts/structural/angle_iron_a3_front_outer_vertical.scad',
        'A4': '../openscad/parts/structural/angle_iron_a4_frame_tube_mount.scad',
        'A5': '../openscad/parts/structural/angle_iron_a5_arm_crossbeam_mount.scad',
        'A6-1': '../openscad/parts/structural/angle_iron_a6_bottom_horizontal.scad',
        'A6-2': '../openscad/parts/structural/angle_iron_a6_bottom_horizontal.scad',
        'A6-3': '../openscad/parts/structural/angle_iron_a6_bottom_horizontal.scad',
        'A7': '../openscad/parts/structural/angle_iron_a7_platform_side.scad',
        'A8': '../openscad/parts/structural/angle_iron_a8_platform_transverse.scad',
        'A9': '../openscad/parts/structural/angle_iron_a9_front_center.scad',
        'A10': '../openscad/parts/structural/angle_iron_a10_front_outer.scad',
        'T1': '../openscad/parts/structural/tube_t1_front_frame.scad',
        'T2': '../openscad/parts/structural/tube_t2_rear_frame.scad',
        'T3': '../openscad/parts/structural/tube_t3_arm_crossbeam.scad',
        'T4': '../openscad/parts/structural/tube_t4_arm_main.scad',
        'T5': '../openscad/parts/structural/tube_t5_arm_leg_spacer.scad',
    }
    
    # Get the SCAD file path
    scad_file_rel = scad_file_map.get(part_code)
    if not scad_file_rel:
        print(f"  ⚠️  No SCAD file defined for part {part_code}")
        return None
    
    # Create absolute path from temp directory
    scad_path = os.path.join(temp_dir, f'{part_code}.scad')
    
    # Get absolute path to structural_parts.scad
    script_dir = os.path.dirname(os.path.abspath(__file__))
    structural_parts_path = os.path.join(script_dir, 'openscad/parts/structural/structural_parts.scad')
    
    # Create a wrapper SCAD file that includes the part file
    with open(scad_path, 'w') as f:
        # Use the structural_parts.scad master file for consistent module names
        if part_code.startswith('A') or part_code.startswith('T'):
            # Map part codes to their module names in structural_parts.scad
            module_map = {
                'A1': 'part_a1_back_outer_vertical',
                'A2': 'part_a2_back_inner_vertical',
                'A3': 'part_a3_front_outer_vertical',
                'A4': 'part_a4_frame_tube_mount',
                'A5': 'part_a5_arm_crossbeam_mount',
                'A6-1': 'part_a6_bottom_segment_1',
                'A6-2': 'part_a6_bottom_segment_2',
                'A6-3': 'part_a6_bottom_segment_3',
                'A7': 'part_a7_platform_side_angle',
                'A8': 'part_a8_platform_transverse_angle',
                'A9': 'part_a9_front_center_angle',
                'A10': 'part_a10_front_outer_angle',
                'T1': 'part_t1_front_frame_tube',
                'T2': 'part_t2_rear_frame_tube',
                'T3': 'part_t3_arm_crossbeam',
                'T4': 'part_t4_arm_main',
                'T5': 'part_t5_arm_leg_spacer_cut',  # Use cut version which accepts show_holes
            }
            
            module_name = module_map.get(part_code)
            if module_name:
                f.write(f'''use <{structural_parts_path}>

{module_name}(show_holes=true);
''')
    
    # Get part dimensions for camera positioning
    if part:
        if hasattr(part, 'length_mm'):
            length = part.length_mm
        elif hasattr(part, 'width_mm'):
            # For square/rectangular tubing
            length = max(part.width_mm, part.height_mm) if hasattr(part, 'height_mm') else part.width_mm
        else:
            length = 425.0
        
        # Determine typical dimensions
        if part_code.startswith('T'):
            # Tubular parts - use tube dimensions
            max_dim = 1200.0  # Approximate length of frame tubes
        else:
            # Angle iron parts
            leg = 50.8
            max_dim = max(length, leg * 2)
    else:
        length = 425.0
        max_dim = 425.0
    
    # Calculate camera distances based on part dimensions
    cam_dist = max_dim * 1.2
    
    # Camera configurations for different views with better positioning
    views = {
        'top': {
            'camera': f'0,0,0,0,0,0,{cam_dist}',
            'projection': 'ortho'
        },
        'side': {
            'camera': f'{cam_dist},0,0,90,0,0,{cam_dist}', 
            'projection': 'ortho'
        },
        'end': {
            'camera': f'0,{cam_dist},0,90,0,90,{cam_dist}',
            'projection': 'ortho'
        },
        'diagonal': {
            'camera': f'{cam_dist*0.7},{cam_dist*0.7},{cam_dist*0.5},55,0,45,{cam_dist}',
            'projection': 'ortho'
        },
    }
    
    renders = {}
    for view_name, camera_config in views.items():
        output_path = os.path.join(temp_dir, f'{part_code}_{view_name}.png')
        if generate_openscad_render(part_code, scad_path, camera_config, output_path, autocenter=True):
            convert_to_grayscale_with_white_bg(output_path)
            renders[view_name] = output_path
        else:
            # If OpenSCAD fails, we'll skip rendering for this part
            return None
    
    return renders if renders else None


def draw_part_renders(c, renders, x_start, y_start, width, height):
    """
    Draw the 4 rendered views in a single column (vertical stack)
    
    Args:
        c: Canvas object
        renders: Dict with paths to rendered images
        x_start, y_start: Position to start drawing
        width, height: Available space
    """
    if not renders:
        return
    
    # Layout: Single column, 4 images stacked vertically
    img_width = width
    img_height = height / 4 - 5  # Divide height by 4 images with small gaps
    
    # Title
    c.setFont("Helvetica-Bold", 10)
    c.setFillColor(colors.black)
    c.drawString(x_start, y_start + height + 5, "3D Views:")
    
    # Draw images in single column (vertical stack)
    view_order = ['top', 'side', 'end', 'diagonal']
    
    labels = {
        'top': 'Top View',
        'side': 'Side View',
        'end': 'End View',
        'diagonal': 'Isometric',
    }
    
    for i, view_name in enumerate(view_order):
        if view_name in renders:
            try:
                # Calculate Y position for this image (top to bottom)
                y = y_start + height - (i + 1) * (img_height + 5)
                
                # Draw image
                img = ImageReader(renders[view_name])
                c.drawImage(img, x_start, y, width=img_width, height=img_height, 
                           preserveAspectRatio=True, mask='auto')
                
                # Draw label
                c.setFont("Helvetica", 7)
                c.setFillColor(colors.black)
                c.drawString(x_start + 3, y + img_height - 10, labels[view_name])
                
            except Exception as e:
                print(f"  ⚠️  Error drawing {view_name} render: {str(e)}")


def generate_cut_list_page(c, part, temp_dir=None):
    """Generate one page for a structural part"""
    width, height = letter
    margin = 0.75 * inch
    
    # Header
    c.setFont("Helvetica-Bold", 24)
    c.setFillColor(colors.black)
    c.drawString(margin, height - margin, f"Part {part.part_code}: {part.name}")
    
    # Part info box
    c.setFont("Helvetica", 12)
    info_y = height - margin - 40
    c.drawString(margin, info_y, f"Quantity Needed: {part.quantity}")
    c.drawString(margin + 200, info_y, f"Material: {part.material}")
    
    # Draw horizontal line
    c.setStrokeColor(colors.black)
    c.setLineWidth(1)
    c.line(margin, info_y - 10, width - margin, info_y - 10)
    
    # Engineering drawing section (full width)
    drawing_y_start = info_y - 30
    drawing_height = 180
    draw_angle_iron_drawing(c, part, margin, drawing_y_start - drawing_height, 
                           width - 2*margin, drawing_height)
    
    # Calculate layout for operations and 3D views side by side
    content_y_start = drawing_y_start - drawing_height - 30
    content_height = 400  # Height for both operations and 3D views
    
    # 2/3 width for operations, 1/3 for 3D views
    page_width = width - 2*margin
    operations_width = page_width * 0.65  # 65% for operations (a bit more than 2/3)
    renders_width = page_width * 0.30     # 30% for renders
    gap = page_width * 0.05               # 5% gap between sections
    
    # Operations checklist section (left side, 2/3)
    operations_x = margin
    draw_operations_checklist(c, part, operations_x, content_y_start - content_height,
                              operations_width, content_height)
    
    # 3D Renders section (right side, 1/3)
    if temp_dir:
        renders_x = margin + operations_width + gap
        
        print(f"  Generating 3D renders for {part.part_code}...")
        renders = generate_part_renders(part.part_code, part.name.lower().replace(' ', '_'), temp_dir, part)
        
        if renders:
            draw_part_renders(c, renders, renders_x, content_y_start - content_height,
                            renders_width, content_height)
        else:
            # If rendering failed, add a note in the 3D views area
            c.setFont("Helvetica-Oblique", 8)
            c.setFillColor(colors.grey)
            # Wrap the text for narrow column
            note = "(3D renders require OpenSCAD)"
            c.drawString(renders_x, content_y_start - 10, note)
    
    # Footer
    c.setFont("Helvetica-Oblique", 8)
    c.setFillColor(colors.grey)
    footer_text = "LifeTrac v25 - Open Source Ecology - GPL v3"
    c.drawString(margin, margin - 20, footer_text)
    c.drawRightString(width - margin, margin - 20, f"Part Code: {part.part_code}")
    
    c.showPage()


def get_structural_parts():
    """Define all structural steel parts for LifeTrac v25"""
    parts = []
    
    # A1: Back Stiffener - Outer Wall Vertical
    # Height calculated from FRAME_Z_OFFSET + 350 to FRAME_Z_OFFSET + MACHINE_HEIGHT (~600mm)
    parts.append(StructuralPart(
        part_code="A1",
        name="Back Stiffener Outer Vertical",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=600.0,
        quantity=2,
        holes=[
            {"position_mm": 50.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 200.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 350.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 500.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 100.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 250.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 400.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 550.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
        ],
        notes="Back stiffener plate outer walls. 3/8\" holes at ~150mm spacing on both legs."
    ))
    
    # A2: Back Stiffener - Inner Wall Vertical
    parts.append(StructuralPart(
        part_code="A2",
        name="Back Stiffener Inner Vertical",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=600.0,
        quantity=4,
        holes=[
            {"position_mm": 50.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 200.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 350.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 500.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 100.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 250.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 400.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 550.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
        ],
        notes="Back stiffener inner walls (2 per panel). Same dimensions as A1."
    ))
    
    # A3: Front Stiffener - Outer Section Vertical (4.75")
    parts.append(StructuralPart(
        part_code="A3",
        name="Front Stiffener Outer Vertical",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=120.65,
        quantity=6,
        holes=[
            {"position_mm": 30.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 90.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 45.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 75.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
        ],
        notes="Front stiffener 5\" sections. 3/8\" holes, 3\" spacing."
    ))
    
    # A4: Frame Tube Mount Angles
    parts.append(StructuralPart(
        part_code="A4",
        name="Frame Tube Mount Angle",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=146.05,
        quantity=16,
        holes=[
            {"position_mm": 47.625, "diameter_mm": 12.7, "description": "Wall mounting bolt 1 (vertical leg)"},
            {"position_mm": 98.425, "diameter_mm": 12.7, "description": "Wall mounting bolt 2 (vertical leg)"},
            {"position_mm": 47.625, "diameter_mm": 12.7, "description": "Tube mounting bolt 1 (horizontal leg)"},
            {"position_mm": 98.425, "diameter_mm": 12.7, "description": "Tube mounting bolt 2 (horizontal leg)"},
        ],
        notes="Mounts 2×6 frame tubes to panels. 1/2\" holes: 4\" spacing on wall leg, 2\" spacing on tube leg."
    ))
    
    # A5: Arm Crossbeam Mount Angles
    parts.append(StructuralPart(
        part_code="A5",
        name="Arm Crossbeam Mount Angle",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=146.05,
        quantity=4,
        holes=[
            {"position_mm": 47.625, "diameter_mm": 12.7, "description": "Plate mounting bolt 1 (vertical leg)"},
            {"position_mm": 98.425, "diameter_mm": 12.7, "description": "Plate mounting bolt 2 (vertical leg)"},
            {"position_mm": 47.625, "diameter_mm": 12.7, "description": "Beam mounting bolt 1 (horizontal leg)"},
            {"position_mm": 98.425, "diameter_mm": 12.7, "description": "Beam mounting bolt 2 (horizontal leg)"},
        ],
        notes="Connects loader arm crossbeam to arm plates. 2 per arm × 2 arms = 4 total."
    ))
    
    # A6: Bottom Stiffener - Horizontal (3 segments)
    # Segment 1 (rear)
    parts.append(StructuralPart(
        part_code="A6-1",
        name="Bottom Horizontal Segment 1 (Rear)",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=200.0,
        quantity=8,
        holes=[
            {"position_mm": 50.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 150.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 75.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
        ],
        notes="Bottom plate rear segment. Split pattern with 8\" gaps at wheels."
    ))
    
    # Segment 2 (middle)
    parts.append(StructuralPart(
        part_code="A6-2",
        name="Bottom Horizontal Segment 2 (Middle)",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=300.0,
        quantity=8,
        holes=[
            {"position_mm": 50.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 200.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 75.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
            {"position_mm": 225.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
        ],
        notes="Bottom plate middle segment (between wheels)."
    ))
    
    # Segment 3 (front)
    parts.append(StructuralPart(
        part_code="A6-3",
        name="Bottom Horizontal Segment 3 (Front)",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=250.0,
        quantity=8,
        holes=[
            {"position_mm": 50.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 200.0, "diameter_mm": 9.525, "description": "Plate mounting (horizontal leg)"},
            {"position_mm": 75.0, "diameter_mm": 9.525, "description": "Wall mounting (vertical leg)"},
        ],
        notes="Bottom plate front segment."
    ))
    
    # A7: Platform Side Angle Iron
    parts.append(StructuralPart(
        part_code="A7",
        name="Platform Side Angle",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=400.0,
        quantity=2,
        holes=[
            {"position_mm": 60.0, "diameter_mm": 9.525, "description": "Pivot bracket bolt 1 (vertical leg)"},
            {"position_mm": 140.0, "diameter_mm": 9.525, "description": "Pivot bracket bolt 2 (vertical leg)"},
            {"position_mm": 260.0, "diameter_mm": 9.525, "description": "Deck bolt 1 (horizontal leg)"},
            {"position_mm": 340.0, "diameter_mm": 9.525, "description": "Deck bolt 2 (horizontal leg)"},
        ],
        notes="Standing platform side arms. Make one left and one right (mirror image)."
    ))
    
    # A8: Platform Transverse Angle Iron
    parts.append(StructuralPart(
        part_code="A8",
        name="Platform Transverse Angle",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=300.0,
        quantity=2,
        holes=[
            {"position_mm": 75.0, "diameter_mm": 9.525, "description": "Deck mounting hole"},
            {"position_mm": 150.0, "diameter_mm": 9.525, "description": "Deck mounting hole"},
            {"position_mm": 225.0, "diameter_mm": 9.525, "description": "Deck mounting hole"},
        ],
        notes="Platform transverse bracing (left-right across deck)."
    ))
    
    # A9: Front Stiffener - Center Section (5.75")
    parts.append(StructuralPart(
        part_code="A9",
        name="Front Center Angle",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=146.05,
        quantity=2,
        holes=[
            {"position_mm": 47.625, "diameter_mm": 12.7, "description": "Mounting bolt 1 (vertical leg)"},
            {"position_mm": 98.425, "diameter_mm": 12.7, "description": "Mounting bolt 2 (vertical leg)"},
            {"position_mm": 47.625, "diameter_mm": 12.7, "description": "Mounting bolt 1 (horizontal leg)"},
            {"position_mm": 98.425, "diameter_mm": 12.7, "description": "Mounting bolt 2 (horizontal leg)"},
        ],
        notes="Front stiffener motor plate inner faces. Same as A4."
    ))
    
    # A10: Front Stiffener - Outer Section (Motor Plate Sides)
    parts.append(StructuralPart(
        part_code="A10",
        name="Front Outer Angle (Motor Plate)",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=120.65,
        quantity=8,
        holes=[
            {"position_mm": 30.0, "diameter_mm": 9.525, "description": "Mounting hole 1 (vertical leg)"},
            {"position_mm": 90.0, "diameter_mm": 9.525, "description": "Mounting hole 2 (vertical leg)"},
            {"position_mm": 45.0, "diameter_mm": 9.525, "description": "Mounting hole 1 (horizontal leg)"},
            {"position_mm": 75.0, "diameter_mm": 9.525, "description": "Mounting hole 2 (horizontal leg)"},
        ],
        notes="Front stiffener outer sections and motor plate sides."
    ))
    
    # T1: Front Cross Frame Tube
    parts.append(StructuralPart(
        part_code="T1",
        name="Front Frame Tube",
        material='2" × 6" × 1/4" Rectangular Tubing',
        length_mm=1133.0,
        quantity=1,
        holes=[],
        notes="Front cross frame tube. Length = TRACK_WIDTH + SANDWICH_SPACING + 2×PANEL_THICKNESS + extensions."
    ))
    
    # T2: Rear Cross Frame Tube
    parts.append(StructuralPart(
        part_code="T2",
        name="Rear Frame Tube",
        material='2" × 6" × 1/4" Rectangular Tubing',
        length_mm=1133.0,
        quantity=1,
        holes=[],
        notes="Rear cross frame tube. Same length as T1."
    ))
    
    # T3: Arm Crossbeam Tube
    parts.append(StructuralPart(
        part_code="T3",
        name="Arm Crossbeam",
        material='2" × 6" × 1/4" Rectangular Tubing',
        length_mm=900.0,
        quantity=1,
        holes=[],
        notes="Loader arm crossbeam connecting left and right arms. Length = ARM_SPACING."
    ))
    
    # T4: Main Arm Tubes
    parts.append(StructuralPart(
        part_code="T4",
        name="Main Arm Tube",
        material='2" × 6" × 1/4" Rectangular Tubing',
        length_mm=800.0,
        quantity=2,
        holes=[],
        notes="Loader arm main sections (pivot to elbow). One per arm."
    ))
    
    # T5: Arm Leg Spacer Tubes
    parts.append(StructuralPart(
        part_code="T5",
        name="Arm Leg Spacer Tube",
        material='2" × 6" × 1/4" Rectangular Tubing',
        length_mm=150.0,
        quantity=2,
        holes=[],
        notes="Spacer tubes at loader arm elbows. Plasma cut to tapered profile."
    ))
    
    return parts


def main():
    """Generate the cut list PDF"""
    output_file = "structural_steel_cut_list.pdf"
    
    print("Generating LifeTrac v25 Structural Steel Cut List...")
    
    # Create temporary directory for OpenSCAD renders
    with tempfile.TemporaryDirectory() as temp_dir:
        # Create PDF
        c = canvas.Canvas(output_file, pagesize=letter)
        
        # Get all parts
        parts = get_structural_parts()
        
        # Generate one page per part
        for part in parts:
            print(f"  Adding page for {part.part_code}: {part.name} (Qty: {part.quantity})")
            generate_cut_list_page(c, part, temp_dir)
        
        # Save PDF
        c.save()
    
    print(f"\n✓ Generated {output_file}")
    print(f"  Total parts: {len(parts)}")
    print(f"  Total pages: {len(parts)}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
