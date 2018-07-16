set term png size 1800,600 enhanced
set output "arc_param.png"
set multiplot layout 1, 3 title "Arc Parameterization"

set autoscale

set title "Path"
plot "arcparam.csv" using 3:4:1 w p ps 0.2 pt 7 lc palette lw 2

set autoscale x
set autoscale y

set title "Curvature"
plot "arcparam.csv" using 2:5:1 w l lc palette lw 2

set title "Angle"
plot "arcparam.csv" using 2:6:1 w l lc palette lw 2

unset multiplot