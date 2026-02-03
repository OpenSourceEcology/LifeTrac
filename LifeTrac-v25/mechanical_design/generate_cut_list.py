#!/usr/bin/env python3
"""
Generate PDF cut list for LifeTrac v25 structural steel parts.
Creates one page per unique angle iron and square tubing part with:
- Engineering drawing showing dimensions and hole positions
- List of cutting and drilling operations with checkboxes
- Part code and quantity needed
- OpenSCAD rendered views (top, side, end, 45° diagonal)
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
    
    # Draw cross-section at left end
    profile_scale = 3
    leg_size = leg_size_mm * scale * profile_scale
    thick = thick_mm * scale * profile_scale
    
    # Draw L-shape profile
    c.setFillColor(colors.lightgrey)
    c.rect(x_offset - 20, y_center - leg_size/2, thick, leg_size, fill=1, stroke=1)
    c.rect(x_offset - 20, y_center - leg_size/2, leg_size, thick, fill=1, stroke=1)
    
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
        part_code: Part code (A1, A2, etc.)
        scad_module_name: Name of OpenSCAD module to render
        temp_dir: Temporary directory for outputs
        part: StructuralPart object with dimensions and hole data
    
    Returns:
        Dict with paths to rendered images, or None if rendering failed
    """
    # Create temporary SCAD file that calls the module
    scad_path = os.path.join(temp_dir, f'{part_code}.scad')
    
    # Get part dimensions for camera positioning
    if part:
        length = part.length_mm
        leg = 50.8  # 2" angle iron leg size
    else:
        length = 425.0  # default
        leg = 50.8
    
    # For now, create angle iron visualization with holes
    with open(scad_path, 'w') as f:
        if part_code == 'A1':
            # Use the actual platform_angle_arm module which has holes
            f.write('''
include <../openscad/lifetrac_v25_params.scad>
use <../openscad/parts/platform_angle_arm.scad>

// Render with holes visible
platform_angle_arm(show_holes=true);
''')
        else:
            # Generic angle iron with holes for other parts
            if not part:
                return None
            
            # Generate angle iron with holes
            f.write(f'''
$fn = 32;

// Angle iron visualization with holes
module angle_iron_with_holes() {{
    leg = 50.8;  // 2" angle iron
    thick = 6.35;  // 1/4" thickness
    length = {length};
    
    difference() {{
        // Main angle iron shape
        color("Gray")
        rotate([90, 0, 0])
        linear_extrude(height=length)
        polygon([
            [0, 0],
            [leg, 0],
            [leg, thick],
            [thick, thick],
            [thick, leg],
            [0, leg]
        ]);
        
        // Drill holes
''')
            
            # Add holes based on part data
            if part and part.holes:
                for hole in part.holes:
                    hole_pos = hole['position_mm']
                    hole_dia = hole['diameter_mm']
                    # Determine which leg the hole goes through
                    if 'vertical leg' in hole.get('description', '').lower():
                        # Hole through vertical leg (X direction)
                        f.write(f'''        // {hole['description']}
        translate([leg/2, -{hole_pos}, leg/2])
        rotate([0, 90, 0])
        cylinder(d={hole_dia}, h=leg+2, center=true, $fn=24);
''')
                    else:
                        # Hole through horizontal leg (Z direction) or default
                        f.write(f'''        // {hole['description']}
        translate([leg/2, -{hole_pos}, thick/2])
        cylinder(d={hole_dia}, h=thick+2, center=true, $fn=24);
''')
            
            f.write('''    }
}

angle_iron_with_holes();
''')
    
    # Calculate camera distances based on part dimensions
    # Use bounding box to determine good camera distance
    max_dim = max(length, leg * 2)  # diagonal of L-shape
    # Camera distance to fit object at ~90% of view
    # For ortho projection, distance affects the viewing volume
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
        'diagonal': '45° Diagonal',
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
    
    # Platform Angle Arms - based on platform_angle_arm.scad
    # Length = PLATFORM_ARM_LENGTH - PLATFORM_BRACKET_WIDTH/2 = 425 - 50 = 375mm
    # Actual arm length in the code uses full PLATFORM_ARM_LENGTH = 425mm
    # Based on platform_angle_arm.scad: length = PLATFORM_ARM_LENGTH - PLATFORM_BRACKET_WIDTH/2
    # But looking at the actual module, it uses the full length parameter
    platform_arm_length = 425.0  # PLATFORM_ARM_LENGTH from params
    
    # Hole positions from platform_angle_arm.scad:
    # Pivot end: PLATFORM_SIDE_BOLT_START and PLATFORM_SIDE_BOLT_START + PLATFORM_SIDE_BOLT_SPACING
    # Deck end: centered on length with PLATFORM_SIDE_DECK_BOLT_SPACING
    pivot_hole_1 = 20.0  # PLATFORM_SIDE_BOLT_START
    pivot_hole_2 = 90.0  # PLATFORM_SIDE_BOLT_START + PLATFORM_SIDE_BOLT_SPACING (20 + 70)
    
    center_y = platform_arm_length / 2  # 212.5mm
    deck_spacing = 150.0  # PLATFORM_SIDE_DECK_BOLT_SPACING
    deck_hole_1 = center_y - deck_spacing/2  # 137.5mm
    deck_hole_2 = center_y + deck_spacing/2  # 287.5mm
    
    parts.append(StructuralPart(
        part_code="A1",
        name="Platform Angle Arm",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=platform_arm_length,
        quantity=2,
        holes=[
            {"position_mm": pivot_hole_1, "diameter_mm": 12.7, "description": "Pivot bracket bolt 1 (vertical leg)"},
            {"position_mm": pivot_hole_2, "diameter_mm": 12.7, "description": "Pivot bracket bolt 2 (vertical leg)"},
            {"position_mm": deck_hole_1, "diameter_mm": 12.7, "description": "Deck bolt 1 (horizontal leg)"},
            {"position_mm": deck_hole_2, "diameter_mm": 12.7, "description": "Deck bolt 2 (horizontal leg)"},
        ],
        notes="Make one left and one right (mirror image). Two pivot holes through vertical leg, two deck holes through horizontal leg."
    ))
    
    # Frame Tube Angle Irons - based on frame_tube_angle_iron module in lifetrac_v25.scad
    # Length = 152.4mm (6") - 2×3.175mm (trim) = 146.05mm (5.75")
    parts.append(StructuralPart(
        part_code="A2",
        name="Frame Tube Mounting Angle",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=146.05,
        quantity=16,
        holes=[
            {"position_mm": 22.86, "diameter_mm": 12.7, "description": "Plate mounting bolt 1"},
            {"position_mm": 123.19, "diameter_mm": 12.7, "description": "Plate mounting bolt 2"},
            {"position_mm": 22.86, "diameter_mm": 12.7, "description": "Tube mounting bolt 1 (other leg)"},
            {"position_mm": 123.19, "diameter_mm": 12.7, "description": "Tube mounting bolt 2 (other leg)"},
        ],
        notes="Used to mount frame tubes to side panels. 2 per tube × 2 tubes × 4 panels = 16 total."
    ))
    
    # Loader Arm Angle Irons - based on angle_iron_mount in loader_arm_v2.scad  
    # Same length as frame tube angles
    parts.append(StructuralPart(
        part_code="A3",
        name="Loader Arm Mounting Angle",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=146.05,
        quantity=4,
        holes=[
            {"position_mm": 22.86, "diameter_mm": 12.7, "description": "Plate mounting bolt 1"},
            {"position_mm": 123.19, "diameter_mm": 12.7, "description": "Plate mounting bolt 2"},
            {"position_mm": 48.26, "diameter_mm": 12.7, "description": "Beam mounting bolt 1 (other leg)"},
            {"position_mm": 97.79, "diameter_mm": 12.7, "description": "Beam mounting bolt 2 (other leg)"},
        ],
        notes="Connects loader arm to side panels. 2 per arm × 2 arms = 4 total."
    ))
    
    # Vertical Angle Irons for side panels - multiple sizes
    parts.append(StructuralPart(
        part_code="A4",
        name="Side Panel Vertical Angle (Tall)",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=550.0,
        quantity=8,
        holes=[
            {"position_mm": 60.0, "diameter_mm": 9.525, "description": "Panel mounting hole"},
            {"position_mm": 490.0, "diameter_mm": 9.525, "description": "Panel mounting hole"},
        ],
        notes="Vertical stiffeners for side panels. Varies in height per panel section."
    ))
    
    # Horizontal Split Angle Irons for bottom plates
    parts.append(StructuralPart(
        part_code="A5",
        name="Bottom Plate Horizontal Angle (Front)",
        material='2" × 2" × 1/4" Angle Iron',
        length_mm=400.0,
        quantity=8,
        holes=[
            {"position_mm": 60.0, "diameter_mm": 9.525, "description": "Plate mounting hole"},
            {"position_mm": 340.0, "diameter_mm": 9.525, "description": "Plate mounting hole"},
        ],
        notes="Bottom plate stiffeners. Split pattern to clear wheel axles."
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
