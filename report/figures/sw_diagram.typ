// Software system diagram. Included from report.typ.
// Image paths are relative to this file, so the PNG sits alongside it.
#figure(
  image("sw_diagram.png", width: 100%),
  caption: [Software organization. The 1 kHz PWM wrap ISR and the serial, button, and IMU retry protothreads run on core 0, the VGA draw thread runs on core 1. All of them exchange data through global variables.],
) <fig:flow>
