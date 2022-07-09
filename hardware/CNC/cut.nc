%
/ use D=1mm tool

G90
G21
G17

F1000
G00 Z10
G00 X0 Y0

/cut edge
/left additional 0.1mm for each edge

M0

F1000 G00 Z5
F1000 G00 X-0.6 Y-0.6

F200 G01 Z-0.7
F200 G01 Y11.6
F200 G01 X36.6
F200 G01 Y-0.6
F200 G01 X-0.6

F200 G01 Z-1.4
F200 G01 Y11.6
F200 G01 X36.6
F200 G01 Y-0.6
F200 G01 X-0.6

F200 G01 Z-2.1
F200 G01 Y11.6
F200 G01 X36.6
F200 G01 Y-0.6
F200 G01 X-0.6

F200 G01 Z0
F1000 G00 Z10
F1000 G00 X0 Y0

M5
%

