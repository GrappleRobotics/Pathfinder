set term png size 1800,1200 enhanced
set output "cdt.png"
set multiplot layout 2, 3 title "CDT Model (Centre)"

set title "Position"
plot "cdt.csv" using 2:3 w l lw 2

set title "Heading"
plot "cdt.csv" using 1:4 w l lw 2

set title "Distance"
plot "cdt.csv" using 1:5 w l lw 2

set title "Velocity"
plot "cdt.csv" using 1:6 w l lw 2

set title "Acceleration"
plot "cdt.csv" using 1:7 w l lw 2

set title "Curvature"
plot "cdt.csv" using 1:8 w l lw 2