set output "hermite_".hermitetype.".png"
set multiplot layout 2, 1 title "Hermite ".hermitetype
set title "Path"
plot "hermite_".hermitetype.".csv" u 2:3 w l lw 3
set title "Curvature"
plot "" u 1:4 pt 7 ps 0.75 lc "green"
unset multiplot