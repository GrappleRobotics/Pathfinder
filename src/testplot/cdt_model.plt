set term png size 600,600 enhanced
set output "cdt.png"

set title "CDT Model"
plot "cdt_model.csv" using 1:2 w l lw 2, "" using 1:3 w l lw 2, "" using 1:4 w l lw 2