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
    Demetrios Gavalas, Parishee Bajaj, Akmal Rupasingha\
    Cornell University \
    ECE 5730 \u{00B7} #datetime.today().display("[month repr:long] [day], [year]")
  ]
]

#v(1em)
#outline(indent: auto, depth: 2)
#pagebreak()

#todo([Parishee])
// ============================================================
= Introduction
// ============================================================
Motors are used in many applications from aerospace to consumer electronics, and due to the mechanical nature of motors, they are subject to wear and tear which can cause discrepancies in how they behave in different conditions and over time. One way to address these nonlinearities is to use a feedback control system, adjusting the input based on the output to achieve a desired behavior. In this lab, we implemented a PID controller to control the angle of a physical one-degree of freedom helicopter. The setup consists of a fan on a lever arm mounted onto a hinge, with an IMU mounted on the lever arm to pass relevant gyroscope & acceleration information, a motor controller to drive the fan, and a VGA display to showcase relevant information, all of which were interfaced with the RP2350. After implementation, we spent time in testing & tuning the PID parameters to reduce settling time & accuracy.  

// ============================================================
= Design and Testing Methods
// ============================================================

#todo([Akmal])
== System overview
#todo([Parishee])
== Hardware
The motor that we were provided was mounted onto the PCB lever arm that we were provided in class. The lever arm was a PCB that had a fan mounted on one end, and the IMU & hinge mounted on the other end. The electrical connections, motor +/-, 3.3V, GND, SDA, & SCL were all made using screw terminals. 

The motor itself was not directly driven by the RP2350 because of the power requirements, and instead it was driven by an electrical isolated motor drive circuit. This circuit was on a seperate PCB that we also soldered in lab, and the RP2350 communicated with it using a PWM signal. The motor itself was powered through the motor drive circuit PCB using a 5V barrel-jack power supply. The IMU was powered by the RP2350 using 3.3V and GND via screw terminals, and communicated with the RP2350 using I2C. The motor itself was driven by the high-power MOSFET rapidly switching on and off to pass the PWM signal to the motor. This MOSFET acted as a kill-switch and was powered by a seperate 12V power supply. The kill-switch allowed us to instantly break the connection between the motor and the MOSFET, so the motor would stop moving even though it was still technically powered by the 5V supply and the RP2350 was still sending PWM signals.

#figure(
  image("figures/motor-board.png", width: 75%),
  caption: [Populated motor control PCB with power connections, PWM input, optoisolator, MOSFET, and kill switch. Source: #link("https://vanhunteradams.com/Pico/Helicopter/MotorCircuit.html")[V. Hunter Adams].],
) <fig:motor-board>

=== IMU (MPU6050)
The IMU that we used was the MPU6050, integrated on an Adafruit breakout board. The IMU is 6-DOF meaning that we get information on 3 axes of acceleration and 3 axes of angular velocity. The IMU was powered by the RP2350 and communicated with it using I2C. It's important to have the IMU securely mounted & aligned with the fans motion ensure that the data we recieve is accurate to apply to the PID controller. 

#figure(
  image("figures/mpu6050.jpg", width: 75%),
  caption: [Adafruit MPU6050 breakout board showing the sensor axes and pin labels. Source: #link("https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/pinouts")[Adafruit].],
) <fig:imu>

=== Motor drive and electrical isolation

Motors are high power and can introduce a lot of electrical noise to whatever system they are connected to. This is especially apparent at startup and large changes in speed. Noise can interfere with sensitive data lines, especially with sensors, and also cause damage to the RP2350's power line which can fry the RP2350. The best way to preven this is to electrically isolate the motor from the RP2350 and relevant peripherals. We did this by using two different power supplies, the RP2350 being powered off of our laptop and the motor being powered off of a 5V barrel jack power supply. 

Despite the power isolation, the PWM signal from the RP2350 still had to reach the motor driver circuit which one may think would need to be an electrical connection. However, we used an optoisolator, a component which is able to convert electrical signal into light and transfer it to another electrical signal to the other side of the optoisolator. This communicated the PWm signal without unintentionally interfering with either side of the circuit.


#figure(
  image("figures/motor-drive.png", width: 100%),
  caption: [Motor drive circuit with a 4N35 optoisolator, IRLB8721 MOSFET, and mechanical kill switch. Source: #link("https://vanhunteradams.com/Pico/Helicopter/MotorCircuit.html")[V. Hunter Adams].],
) <fig:motor-drive>


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
After implementing all three terms of the PID controller, we noticed that the motor would behave strangely if we tried to reach large changes in angle very quickly. It would start oscillating, with the motor turning on and off uncontrollably, and we would also lose I2C connection with the IMU. We realized that this was due to the fact that to reach this large change in angle, the PID controller would output a very large PWM duty cycle. This would typically be fine, but if the controller was not moving at all, i.e. PWM duty cycle of 0, the large change in duty cycle would cause the motor to draw a lot of current and fail. To address this, we artificially limited the maximum PWM duty cycle to 70% of what the maximum, to reduce the largest change in duty cycle that the controller could output.


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

#todo([Akmal])
== Mechanical assembly
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

The minimum angle that we could stably hold was 15 degrees. This was due to the fact that our electrical connections were a bunch of solid core wires connected to screw terminals. These wires introduced some unwanted tension & pressure on the lever arm, preventing a full range of motion. The maximum angle that we could stably hold was 175 degrees. This was also limited by mechanical constraints since the screw attaching the lever arm to the hing would hit the wood plank at this angle preventing further motion.

#todo([Parishee])
== Question 3
_What are the settling times at different angles?_
The settling time from 0 to 120 degrees is 1.77 seconds. The settling time from 120 to 60 degrees is 3.87 seconds. The settling time from 60 to 90 degrees is 1.37 seconds. 



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
