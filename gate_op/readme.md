# Controlling the operation of Nice Robus 400 sliding gate gearmotor

gateop.c implement functions to control the operation of RB400 sliding gate

## Theory of operation
Gate gearmotor is controlled by 2 signals <b>Open</b> and <b>Close</b><br>
Gate state is sensed via <b>OGI</b>(open gate indicator) signal<br>
Open and Close are NO contacts. A short on these inputs will trigger gate sliding.<br>
Sliding continues until an other short on the input, or the limit switch is hit.<br>
A short duration of 300 msec looks ok. Duration is controlled by CMD_PW in gateop.h.<br>
Care must be taken because the signals are operating at ~35V DC.
