set size noratio

set output "profile_trap.png"
set multiplot layout 2, 1 title "Trapezoidal Profile"
set title "Distance"
plot "profile_trap.csv" u 1:2 w l lw 2 lc "red", "profile_trap_simulated.csv" u 1:2 w l lw 2 lc "green" title "dist (sim)"

set title "Velocity / Accel"
plot "profile_trap.csv" u 1:3 w l lw 2 lc "red", "profile_trap_simulated.csv" u 1:3 w l lw 2 lc "green" title "vel (sim)", "profile_trap.csv" u 1:4 w l lw 2 lc "blue"
unset multiplot