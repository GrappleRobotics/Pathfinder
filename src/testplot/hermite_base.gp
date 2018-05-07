set output "hermite_".hermitefile.".png"
set multiplot layout 1, 2 title "Hermite ".hermitetype1." ".hermitetype2
set title "Path"
plot "hermite_".hermitetype1.".csv" u 2:3 w l lw 3 lc "red" title hermitetype1, \
     "hermite_".hermitetype2.".csv" u 2:3 w l lw 3 lc "green" title hermitetype2

set title "Curvature"
plot "hermite_".hermitetype1.".csv" u 1:4 pt 7 ps 0.75 lc "red" title hermitetype1,\
     "hermite_".hermitetype2.".csv" u 1:4 pt 7 ps 0.75 lc "green" title hermitetype2
unset multiplot