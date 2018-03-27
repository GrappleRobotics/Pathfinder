set datafile separator ","
set key autotitle columnheader

set term x11 persist

plot '../build/test-results/kinematicsTest/coupled.csv' u 2:3 w l lw 2
