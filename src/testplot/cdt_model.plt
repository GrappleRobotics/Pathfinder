set term png size 1800,1200 enhanced
set output "cdt.png"
set multiplot layout 2, 3 title "CDT Model (Centre)"

set title "Position"
plot "cdt.csv" using 2:3 w p

set title "Heading"
plot "cdt.csv" using 1:4 w p

set title "Distance"
plot "cdt.csv" using 1:5 w p

set title "Velocity"
plot "cdt.csv" using 1:6 w p

set title "Acceleration"
plot "cdt.csv" using 1:7 w p

set title "Curvature"
plot "cdt.csv" using 1:8 w p