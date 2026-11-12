#set document(
  title: "ECE 4760 / 5730 Lab 3: PID Control of a 1D Helicopter",
  author: ("TODO: Name 1", "TODO: Name 2", "TODO: Name 3"),
)
#set page(paper: "us-letter", margin: 1in, numbering: "1")
#set text(font: "New Computer Modern", size: 11pt)
// Number only the main sections ("1.", "2.", ...); subheadings stay unnumbered
#set heading(numbering: (..n) => if n.pos().len() == 1 { numbering("1.", ..n) })
#set par(justify: true, leading: 0.65em)
#show link: set text(fill: blue)

#let placeholder(name) = block(
  width: 100%, height: 2in, fill: luma(240), stroke: (dash: "dashed", paint: luma(150)),
  align(center + horizon, text(fill: luma(100))[Figure placeholder: #raw(name)]),
)

#let todo(body) = text(fill: rgb("#b45309"), style: "italic")[TODO: #body]

// Appendix B prompt log helpers
#let prompt(n, body) = block(
  width: 100%, inset: (x: 10pt, y: 8pt), radius: 3pt, fill: rgb("#eef4fb"),
  stroke: (left: 2.5pt + rgb("#2563eb")), breakable: true,
)[
  #text(size: 9pt, weight: "bold", fill: rgb("#2563eb"))[Prompt #n]
  #v(0.2em)
  #body
]
#let reply(body) = block(
  width: 100%, inset: (left: 10pt, y: 4pt),
  stroke: (left: 1pt + luma(190)), breakable: true,
)[#body]
#let ide-note(body) = text(size: 9pt, style: "italic", fill: luma(90))[#body]

#align(center)[
  #text(size: 16pt, weight: "bold")[ECE 4760 / 5730 --- Lab 3]
  #v(0.4em)
  #text(size: 20pt, weight: "bold")[PID Control of a 1D Helicopter]
  #v(0.6em)
  #text(size: 12pt)[
    Demetrios Gavalas, Akmal Rupasingha #todo[other names]\
    Cornell University \
    ECE 5730 #todo[section/time] \u{00B7} #datetime.today().display("[month repr:long] [day], [year]")
  ]
]

#v(1em)
#outline(indent: auto, depth: 2)
#pagebreak()

#todo([Parishee])
// ============================================================
= Introduction
// ============================================================


// ============================================================
= Design and Testing Methods
// ============================================================

== System overview
Our 1D helicopter is a lever arm on a hinge with a small fan motor at the free end. An MPU6050 IMU on the arm measures acceleration and rotation rate, and the Pico uses these readings to estimate the arm angle and run a PID controller that sets the motor’s PWM duty cycle. The PWM signal drives the motor through an optoisolator, so the motor circuit stays electrically separate from the Pico. The whole control loop runs inside one interrupt. That interrupt comes from the same PWM slice that drives the motor, and it fires every time the PWM counter wraps. With a 150 MHz clock, a clock divider of 25, and a wrap value of 6000, the PWM and the control loop both run at about 1 kHz. Each time it runs, the interrupt reads the IMU over I2C, combines the accelerometer and gyro readings with a complementary filter to get the beam angle, runs the PID controller, and writes the new duty cycle to the PWM hardware. On core 0, one thread takes serial commands for the target angle and the three gains, one watches the button and runs the demo sequence, and one tries to reconnect the IMU once a second if it stops responding. Core 1 only draws the VGA display.

We measure the beam angle in degrees, with 0° when the arm hangs straight down, 90° when it is horizontal, and 180° when it points straight up. Using our convention, the handout’s button sequence of horizontal, 30° above, 30° below, and back to horizontal becomes targets of 90°, 120°, 60°, and 90°. The program zeroes the accelerometer angle when it starts, so the arm has to hang straight down whenever the Pico is powered on or reset.


#todo([Parishee])
== Hardware

=== IMU (MPU6050)

=== Motor drive and electrical isolation

#figure(
  table(
    columns: (1.2fr, 2fr),
    inset: 6pt, align: left, stroke: 0.5pt,
    table.header[*Pico pin*][*Connection*],
    [GPIO 4], [PWM output #sym.arrow optoisolator (motor drive)],
    [GPIO 8 / 9], [MPU6050 SDA / SCL],
    [GPIO 14], [ISR timing debug pin],
    [GPIO 15], [Button (active low, internal pull-up)],
    [GPIO 16, 17], [VGA HSYNC, VSYNC],
    [GPIO 18--21], [VGA green (470 $Omega$, 330 $Omega$), blue, red (330 $Omega$)],
    [3.3 V / GND], [MPU6050 power],
  ),
  caption: [Pin assignments.],
) <tab:pins>

#todo([Demetrios])
== Software

=== Angle estimation

$ theta[k] = alpha (theta[k-1] + omega[k] d t) + (1 - alpha) theta_a[k], quad alpha = 0.98 $


=== PID controller

$ u = K_p e - K_d dot(theta) + K_i integral e thin d t $


=== Serial interface

=== VGA display

=== Demo sequence

=== Noise and filtering

== Selecting the PID gains

#figure(
  table(
    columns: (0.6fr, 0.6fr, 0.6fr, 3fr),
    inset: 6pt, align: (right, right, right, left), stroke: 0.5pt,
    table.header[*$K_p$*][*$K_i$*][*$K_d$*][*Observation*],
    [#todo[]], [0], [0], [P only: ...],
    [], [], [], [],
    [20], [90], [7], [Final],
  ),
  caption: [Tuning log.],
) <tab:tune>

#todo([Demetrios])
== Testing

#figure(
  table(
    columns: (1.4fr, 1.6fr, 1.4fr),
    inset: 6pt, align: left, stroke: 0.5pt,
    table.header[*Requirement*][*Test*][*Pass criterion*],
    [IMU angle], [Move arm by hand, watch VGA], [Angle tracks arm; ~0 down, ~90 horizontal],
    [Motor open loop], [Fixed PWM duty], [Fan speed changes with duty],
    [Serial commands], [Enter target and gains at runtime], [Takes effect immediately, no reset],
    [Closed loop], [Set target to 90$degree$], [Arm rises and holds],
    [Control trace], [View VGA PWM trace], [No visible noise],
    [Angle range], [Step to many targets], [Stable at each (see Results)],
    [Four-angle sequence], [Hold, then release button], [Follows 90/120/60/90 at 5 s steps],
    [IMU fault], [Unplug IMU], [Motor off, warning shown, recovers on replug],
  ),
  caption: [Verification tests.],
)

=== Debugging 

// ============================================================
= Documentation
// ============================================================

#todo([Parishee])
== Schematic
#figure(
  // image("figures/schematic.png", width: 100%),  // restore once the file exists
  placeholder("schematic.png"),
  caption: [Schematic of the motor drive, IMU, and user-interface circuits.],
) <fig:schematic>

#todo([Demetrios])
== Software System Diagram
#figure(
  // image("figures/software-diagram.png", width: 90%),  // restore once the file exists
  placeholder("software-diagram.png"),
  caption: [Software organization: 1 kHz PID ISR, serial/button/retry threads on core 0, VGA draw thread on core 1.],
) <fig:flow>

== Mechanical assembly
The helicopter is a lever arm that hangs from a hinge, with the fan motor mounted at the free end. The hinge is screwed onto a wooden block, which holds the arm up so it can hang straight down and swing freely. The arm itself is a printed circuit board. The motor points so its thrust lifts the arm. The MPU6050 plugs into header pins on the PCB near the hinge. Its Y axis lines up with the hinge, which the angle code depends on.
#figure(
  // image("figures/assembly.jpeg", width: 80%),  // restore once the file exists
  placeholder("assembly.jpeg"),
  caption: [Lever arm, motor, hinge, and IMU.],
) <fig:assembly>

// ============================================================
= Results
// ============================================================
#todo([Akmal])
== Final Product
#figure(
  // image("figures/final-product.jpeg", width: 100%),  // restore once the file exists
  placeholder("final-product.jpeg"),
  caption: [Final 1D helicopter with the VGA display and controller board.],
) <fig:final-product>

#todo([Akmal])
== Four-angle demo sequence
#figure(
  // image("figures/vga-sequence.jpeg", width: 100%),  // restore once the file exists
  placeholder("vga-sequence.jpeg"),
  caption: [VGA capture of the button-triggered sequence: motor control and actual angle as the target goes 90#sym.degree #sym.arrow 120#sym.degree #sym.arrow 60#sym.degree #sym.arrow 90#sym.degree.],
) <fig:sequence>

#todo([Akmal])
== Stable angle range

== Settling times
#figure(
  table(
    columns: (1fr, 1fr, 1fr, 1fr),
    inset: 6pt, align: center, stroke: 0.5pt,
    table.header[*Start*][*Target*][*Overshoot*][*Settling time (s)*],
    [#todo[]], [], [], [],
    [], [], [], [],
  ),
  caption: [Step response at different angles.],
) <tab:settle>

// ============================================================
= Lab Questions
// ============================================================

#todo([Akmal])
== Question 1
_How did you select the three PID gains?_

#todo([Parishee])
== Question 2
_What are the maximum and minimum angles that give stable behavior with your gains?_

#todo([Parishee])
== Question 3
_What are the settling times at different angles?_


// ============================================================
= Conclusions
// ============================================================
#todo([Akmal])


// ============================================================
#heading(numbering: none)[Appendix A --- Commented Program Listing]
// ============================================================

#let listing(path) = block(
  width: 100%, breakable: true, fill: rgb("#f5f5f5"),
  stroke: 0.5pt + rgb("#dddddd"), radius: 3pt, inset: 10pt,
)[
  #set text(size: 7pt)
  #set par(justify: false, leading: 0.4em)
  #raw(read(path), lang: "c", block: true)
]

*imu_demo.c*
#listing("code/imu_demo.c")

*mpu6050.c*
#listing("code/mpu6050.c")

*mpu6050.h*
#listing("code/mpu6050.h")


#pagebreak()
// ============================================================
#heading(numbering: none)[Appendix B --- AI Prompt Log]
// ============================================================


#table(
  columns: (1fr, 1fr, 1fr, 1fr),
  inset: 6pt, align: center, stroke: 0.5pt,
  table.header[][*Additions*][*Deletions*][*Modifications*],
  [*Suggested*], [#todo[]], [#todo[]], [#todo[]],
  [*Accepted*], [#todo[]], [#todo[]], [#todo[]],
)

#prompt(1)[#todo[Paste prompt.]]
#reply[#todo[Paste or summarize reply.]]
#ide-note[#todo[Which suggestions were accepted or edited.]]
