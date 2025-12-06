#!/bin/bash
# export_all_cnc_parts.sh
# Batch export all CNC-ready DXF files from OpenSCAD
#
# Usage: ./export_all_cnc_parts.sh
#
# Requirements:
#   - OpenSCAD installed and in PATH
#   - Run from mechanical_design directory

set -e  # Exit on error

# Create output directory
mkdir -p output/dxf/quarter_inch
mkdir -p output/dxf/half_inch

echo "========================================="
echo "LifeTrac v25 CNC Parts Export"
echo "========================================="
echo ""

# Check if OpenSCAD is available
if ! command -v openscad &> /dev/null; then
    echo "ERROR: OpenSCAD is not installed or not in PATH"
    echo "Please install OpenSCAD from https://openscad.org/"
    exit 1
fi

echo "OpenSCAD found: $(openscad --version 2>&1 | head -1)"
echo ""

# Array of parts to export
# Format: "part_name|output_filename|thickness"
parts=(
    "wheel_mount|wheel_mounting_plate|half_inch"
    "deck|standing_deck|quarter_inch"
    "bucket_bottom|bucket_bottom_plate|quarter_inch"
    "bucket_back|bucket_back_plate|quarter_inch"
    "bucket_side|bucket_side_plate|quarter_inch"
    "arm_reinforcement|loader_arm_reinforcement|quarter_inch"
    "cylinder_lug|cylinder_mounting_lug|half_inch"
    "bucket_attach|bucket_attachment_plate|half_inch"
    "housing_base|control_housing_base|quarter_inch"
)

total_parts=${#parts[@]}
current=0

echo "Exporting $total_parts parts..."
echo ""

for part_info in "${parts[@]}"; do
    IFS='|' read -r part_name output_name thickness <<< "$part_info"
    current=$((current + 1))
    
    output_file="output/dxf/${thickness}/${output_name}.dxf"
    
    echo "[$current/$total_parts] Exporting $part_name to $output_file"
    
    # Export DXF
    openscad -o "$output_file" \
             -D "part=\"$part_name\"" \
             export_for_cnc.scad 2>&1 | grep -v "WARNING: Ignoring unknown" || true
    
    if [ -f "$output_file" ]; then
        size=$(du -h "$output_file" | cut -f1)
        echo "           ✓ Created ($size)"
    else
        echo "           ✗ Failed to create file"
    fi
    echo ""
done

echo "========================================="
echo "Export Complete!"
echo "========================================="
echo ""
echo "Quarter-inch (6.35mm) plate parts:"
ls -lh output/dxf/quarter_inch/*.dxf 2>/dev/null || echo "  (none)"
echo ""
echo "Half-inch (12.7mm) plate parts:"
ls -lh output/dxf/half_inch/*.dxf 2>/dev/null || echo "  (none)"
echo ""
echo "Next steps:"
echo "  1. Open DXF files in CAM software (SheetCAM, etc.)"
echo "  2. Arrange parts for efficient nesting"
echo "  3. Generate G-code for CNC plasma cutter"
echo "  4. Cut parts from steel plate"
echo ""
echo "Note: Parts are grouped by material thickness for efficient cutting"
