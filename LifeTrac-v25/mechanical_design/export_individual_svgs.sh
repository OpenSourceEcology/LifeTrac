#!/bin/bash
# export_individual_svgs.sh
# Export individual 2D SVG cutouts for each plate part
#
# Usage: ./export_individual_svgs.sh
#
# Requirements:
#   - OpenSCAD installed and in PATH
#   - Run from mechanical_design directory

set -e  # Exit on error

# Create output directory
mkdir -p output/svg/parts

echo "========================================="
echo "LifeTrac v25 Individual Part SVG Export"
echo "========================================="
echo ""

# Function to export a part as SVG
export_part() {
    local part_name=$1
    local scad_file=$2
    local output_file=$3
    
    echo "Exporting ${part_name}..."
    openscad -o "${output_file}" \
        --export-format=svg \
        "${scad_file}" 2>&1 | grep -v "WARNING" || true
    
    if [ -f "${output_file}" ]; then
        echo "  ✓ ${part_name} exported successfully"
        ls -lh "${output_file}" | awk '{print "    Size: " $5}'
    else
        echo "  ✗ Failed to export ${part_name}"
        return 1
    fi
    echo ""
}

# Export all parts
echo "Exporting Half-Inch (1/2\") Plate Parts:"
echo "-----------------------------------------"

export_part "Side Panel Outer" \
    "parts/export_side_panel_outer.scad" \
    "output/svg/parts/side_panel_outer.svg"

export_part "Side Panel Inner" \
    "parts/export_side_panel_inner.scad" \
    "output/svg/parts/side_panel_inner.svg"

export_part "Wheel Mount" \
    "parts/export_wheel_mount.scad" \
    "output/svg/parts/wheel_mount.svg"

export_part "Cylinder Lug" \
    "parts/export_cylinder_lug.scad" \
    "output/svg/parts/cylinder_lug.svg"

export_part "Rear Crossmember" \
    "parts/export_rear_crossmember.scad" \
    "output/svg/parts/rear_crossmember.svg"

echo ""
echo "Exporting Quarter-Inch (1/4\") Plate Parts:"
echo "-------------------------------------------"

export_part "Standing Deck" \
    "parts/export_standing_deck.scad" \
    "output/svg/parts/standing_deck.svg"

export_part "Bucket Bottom" \
    "parts/export_bucket_bottom.scad" \
    "output/svg/parts/bucket_bottom.svg"

export_part "Bucket Side" \
    "parts/export_bucket_side.scad" \
    "output/svg/parts/bucket_side.svg"

echo "========================================="
echo "Export Complete!"
echo "========================================="
echo ""
echo "SVG files saved to: output/svg/parts/"
echo ""
echo "Part Count Summary:"
echo "  - 4x Side Panel Outer (use side_panel_outer.svg)"
echo "  - 4x Side Panel Inner (use side_panel_inner.svg)"
echo "  - 4x Wheel Mount (use wheel_mount.svg)"
echo "  - 6x Cylinder Lug (use cylinder_lug.svg)"
echo "  - 1x Rear Crossmember"
echo "  - 1x Standing Deck"
echo "  - 1x Bucket Bottom"
echo "  - 2x Bucket Side (use bucket_side.svg, mirror for other side)"
echo ""
echo "Total: 23 parts from 8 unique designs"
echo ""
