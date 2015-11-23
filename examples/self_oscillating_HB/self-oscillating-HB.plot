set terminal post eps 16
set output 'self-oscillating-HB.eps'
#set yrange [-200:500]
set xrange [0:400e-06]
set xlabel 'time [us]'
set ylabel 'i(load) [A]'
set xtics ("0" 0, "50" 50e-6, "150" 150e-6, "200" 200e-6, "250" 250e-6, "300" 300e-6, "350" 350e-6, "400" 400e-6)
plot \
'Load_traces.txt' using 1:3 notitle with lines
