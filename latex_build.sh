#!/bin/bash
# filepath: /home/aarmstrong/ROS2_MRM/build_latex.sh

# Clean previous files
rm -f ROS2_Investigation.aux ROS2_Investigation.toc ROS2_Investigation.log ROS2_Investigation.out ROS2_Investigation.pdf

# Compile LaTeX (run twice for table of contents)
pdflatex ROS2_Investigation.tex
pdflatex ROS2_Investigation.tex

# Open PDF if compilation successful
if [ -f "ROS2_Investigation.pdf" ]; then
    echo "PDF generated successfully!"
    evince ROS2_Investigation.pdf &
else
    echo "PDF generation failed!"
fi