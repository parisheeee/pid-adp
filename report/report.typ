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

The program logic is split into a few separate sections. In this lab, CPU time is generally not important, as the time to read from the sensor and calculate the PWM motor command signal is relatively short. The VGA driver is handled on core 1, so it doesn't eat any cycles on core 0 which handles the interrupts. This handles plotting the gyro rates and motor command signals from a history array. The following sections detail the design of each of these components.


=== Angle estimation
- Accel gives an absolute but noisy angle, theta_a = atan2(a_x, a_z) in the code; gyro (Y axis) gives a smooth rate that drifts when integrated. Say why neither alone is enough.
- Introduce the equation below, define each symbol, and say it runs once per 1 ms ISR tick, so dt is about 1 ms.

The angle estimation logic runs in the main PWM interrupt service routine, as it needs to be updated before the motor command signal is updated. Acceleration gives an angle which is absolute, but noisy. The gyroscope gives a smoother signal, but it drifts when integrated because the error component grows with time. The accelerometer alone would be too noisy, while the gyroscope alone would accumulate error from the absolute angle over time. To capture both signals without suffering drift or excessive noise, we use the following complementary filter:

$ theta[k] = alpha (theta[k-1] + omega[k] d t) + (1 - alpha) theta_a[k], quad alpha = 0.98 $

where:
- $omega[k]$ is the gyro angular rate about the IMU Y axis.
- $d t$ is the time between ISR ticks.
- $theta_a[k]$ is the angle from the accelerometer alone, $"atan2"(a_x, a_z)$.
- $alpha$ is the complementary filter weight on the gyro path (0.98).

This calculation runs once per 1 ms ISR tick, so $d t$ is 1 ms. The time constant is roughly $ (alpha)/(1 - alpha)d t$ = 49 ms. The gyro term accurately tracks short term changes, while the accel term dominates at longer timescales. Higher values of alpha give a noisier angle measurement, and lower values result in drift accumulating longer. 

We adopt the convention that the arm is at 0 degrees when hanging down, and 180 degrees when hanging up. To avoid edge cases in PWM calculations, we clamp the angle values to the non-negative bounds [0, 180]. The angle estimate is initialized at zero. Ideally, the system should be reset with the arm hanging straight down.   

Raw values are stored as `fix15`, but the filter calculation is done by casting the values to a float and generating a floating point output. 

=== PID controller
- Where it runs: in the 1 kHz PWM-wrap ISR. Walk through one tick in order: clear IRQ, read IMU, update filter, compute PID, write pwm_set_chan_level.
- Introduce the equation below, with e = target minus measured angle. Explain the derivative is taken on the measurement (gyro rate), not the error, so a target step does not make a spike in the motor command.

The PID controller also runs in the 1 kHz PWM-wrap ISR. The ISR clears the IRQ, reads from the IMU, and after calculating the complementary angle filter computes PID and `writes pwm_set_chan_level`. The PWM output is calculated by the following equation:

$ u = K_p e - K_d dot(theta) + K_i integral e thin d t $
- $K_{p, d, i}$ are user input parameters which correspond with the relative strength of the proportional, derivative, and integral terms respectively.
- $e$ is the error term, which corresponds with the difference between the target angle and the current filtered angle. 
- $dot(theta)$ is the angular velocity of the arm. This is a simple difference between the current and previous angles divided by the time constant $d t$.

The output is clamped to 70% of the max duty cycle, because we found that in practice driving the motor any more causes it to sputter and die as it draws too much current. The output is also clamped to 0 -- the fan can only blow in one direction, and gravity always points downwards ensuring the entire space is reachable while only driving in one direction. 

The integration term is implemented by accumulating the error term in a static variable. The integral accumulation also needs to be clamped: because it is time dependent, the integral term can grow without bound if the arm is stuck at a position below the target angle, or takes to long to reach the target angle. We clamp the integral term to `MAX_CONTROL` / $K_i$, so that it does not saturate the maximum PID control output.

=== Serial interface
The user interface is handled with a protothread on core 0 so that its execution can be preempted by the ISR. The thread prompts the user for the target angle and constants, and the value is updated as soon as the value is received. If an empty message is received, the old value is kept. If an illegal value (not a number, outside range, etc.) the prompt is repeated.

=== VGA display
- ISR keeps decimated circular buffers (one sample per 100 ticks, so 10 samples/s; the 530-pixel plot width then shows about 53 s of history) for gyro rate, accel angle, filtered angle, and motor PWM.
- Draw thread runs on core 1 so it never delays the control loop. Describe the four plots, axis ranges, and the text line (angle, target, Kp, Ki, Kd).
- The motor trace is the value actually written to the PWM hardware. The lab says it must look clean, so say what you did (e.g. the DLPF, decimation) and whether it needed more.
- IMU-missing warning in red, and the retry thread that re-probes once per second and frees a stuck I2C bus.

The ISR writes filtered beam angle and the PID control signal to a decimated / downsampled global buffer. The draw thread runs on core 1, so it never blocks the control or user interface loops. The x axis for both plots is the same ~50s time scale, and the plotted line scrolls across the screen with $t = 0$ on the right. The motor control plot is drawn from 0 to the max 4700 RPMs. The angle is plotted from 0 to 180 degrees. We also write the $K_p, K_d, K_i$ and target angle to the screen so it is easy to adjust parameters and see when the arm has stabilized. 

=== Demo sequence
The demo sequence is implemented with a button between GPIO 15 and ground. A separate core 0 button thread polls the GPIO every 10 ms. While the button is held down, the control signal is 0 and the target angle is 0, allowing the arm to hang down at rest. 

When the button transitions from pressed to released, the `time_us_32` is measured. Every time the protothread is entered subsequently, the time from release is used to schedule the target angle according to the schedule in the lab handout (t = 0 $arrow$ 90 degrees, t = 5 $arrow$ 120 degrees, t = 10 $arrow$ 60 degrees, t = 15 $arrow$ 90 degrees). 


== Selecting the PID gains

#figure(
  table(
    columns: (0.6fr, 0.6fr, 0.6fr, 3fr),
    inset: 6pt, align: (right, right, right, left), stroke: 0.5pt,
    table.header[*$K_p$*][*$K_i$*][*$K_d$*][*Observation*],
    [10], [0], [0], [P only: As expected, the motor speed is 0 at an above the target. When moving the arm down from the target, the speed increases linearly],
    [0], [10], [0], [D only: as expected, the motor tries to resist movement. If the arm is dropped, the fan spins to correct the motion],
    [0], [0], [10], [I only: the motor slowly spins up when below the target, overshoots and then slowly spins down.],
    [20], [9], [90], [This was our ideal parameter set. The fan quickly spins up to reach the target, slightly overshoots, and lowers to the target.],
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
    [IMU angle], [Move arm by hand, watch VGA], [Angle tracks arm: ~0 down, ~90 horizontal],
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

We tested the IMU measurements by plotting all the values to the screen and rotating the unmounted arm in all degrees of freedom, isolating different directions. 

We tested the PID control and motor by configuring a PID value via the serial monitor, and testing different values. This is also how we discovered that the motor could draw too much current and die if the PID command signal was too high.

Then, we tested closed loop control after adding each term. First we added proportional control, and found that the arm would oscillate below the target point, which was expected. Then, we added derivative control, which damped and then stopped the oscillation for higher values of D, although for higher values of D it would again oscillate. Still, the beam could not reach the target angle until we added integral control. 

We did have intermittent issues with losing connection to the IMU, especially when drawing too much current from the motor. We mitigated this by capping the maximum motor command signal and also by printing the IMU connection state to the screen so it was obvious when we lost connection.



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
== Final Product
Our finished 1D helicopter is a PCB lever arm hanging from a hinge on a wooden block, with a fan motor at the free end and an MPU6050 IMU near the hinge. The RP2350 reads the IMU, estimates the arm angle, and uses a PID controller to drive the motor so the arm holds whatever angle we choose. We set the target angle and the three gains from a serial terminal while the program runs, and the new values take effect right away. The final gains were $K_p = 20$, $K_i = 90$, and $K_d = 7$, and the program also starts with these values. A button on GPIO 15 runs the preset sequence of target angles asked for in the final/week 3 demo. The VGA display plots four live traces: gyro rate, accelerometer angle, filtered beam angle, and motor command. The top line of the screen also shows the current beam angle, target, and gains.
#figure(
  // image("figures/final-product.jpeg", width: 100%),  // restore once the file exists
  placeholder("final-product.jpeg"),
  caption: [Final 1D helicopter with the VGA display and controller board.],
) <fig:final-product>

== Four-angle demo sequence
The button runs a fixed test so we can observe how the controller handles steps both up and down. The program checks the button every 10 ms. While the button is held, the motor is forced off, the integral term is reset to zero, and the target is set to 0°, so the arm hangs straight down. When the button is released, a timer starts and the target changes on its own. It is 90° (horizontal) for the first 5 seconds, 120° (30° above horizontal) from 5 to 10 seconds, 60° (30° below horizontal) from 10 to 15 seconds, and then back to 90°, where it stays. The sequence uses whatever gains are set at the time. @fig:sequence is a VGA capture of one run. The yellow trace is the beam angle and the cyan trace is the motor command. The plots save one sample every 100 ms and the whole sequence fits on one screen. Because the motor command is capped at 70% duty, the cyan trace never goes above 70% of its plot height.
#figure(
  // image("figures/vga-sequence.jpeg", width: 100%),  // restore once the file exists
  placeholder("vga-sequence.jpeg"),
  caption: [VGA capture of the button-triggered sequence: motor control and actual angle as the target goes 90#sym.degree #sym.arrow 120#sym.degree #sym.arrow 60#sym.degree #sym.arrow 90#sym.degree.],
) <fig:sequence>

#todo([can whoever has the video for this take this section from me plss (＾▽＾) -ak])
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

== Question 1
_How did you select the three PID gains?_
We tuned the gains by hand, one term at a time, typing new values into the serial terminal while the arm was running so we could see the effect right away. We started with only proportional control, with $K_i$ and $K_d$ set to 0. We raised $K_p$ until the arm could lift off and get close to the target. With a low $K_p$ the arm sagged well below the target, and raising $K_p$ brought it closer but made it overshoot and swing back and forth. Next we added the derivative term to damp the motion. Our first version did not work: it used the raw gyro reading with the wrong sign, so instead of slowing the arm down it pushed it the same way it was already moving. The raw reading was also so large that even a small $K_d$ slammed the motor between off and full power whenever the arm moved. We fixed the sign and scaled the gyro reading down by $10^{-5}$ so that $K_d$ could be a normal-sized number. Last, we added the integral term to remove the steady-state error, since with P and D alone the arm settled a little below the target. The motor needs steady thrust to hold the arm up, and a P controller only gives thrust when there is some error. We stop the integral from growing while the motor is at its limit, so it cannot build up and cause a large overshoot later. We ended with $K_p = 20$, $K_i = 90$, and $K_d = 7$, because this set moved the arm to each new target quickly with little overshoot and stayed stable at every angle in the demo sequence.

#todo([Parishee])
== Question 2
_What are the maximum and minimum angles that give stable behavior with your gains?_

#todo([Parishee])
== Question 3
_What are the settling times at different angles?_


// ============================================================
= Conclusions
// ============================================================
#todo([feel free to add any concluding thoughts for what u found interesting i j added mine here hehe (＾▽＾) -ak])

One of the most interesting parts of this lab was seeing what each PID term actually did to the arm. With only proportional control, the arm oscillated. The P term works like a spring: the farther the arm is from the target, the harder the motor pushes it back. But the P term doesn’t care how fast the arm is already moving, so the arm overshoots the target and swings back and forth. When we added the derivative term, the oscillation was greatly reduced. Our D term uses the gyro rate, so it pushes against the arm’s motion, a lot like friction. The faster the arm moves toward the target, the more the D term slows it down, so it settles instead of overshooting. Before this, we knew that D adds damping. Watching the arm go from swinging back and forth to settling in a smoother fashion made that idea much easier to understand.

Building and tuning the 1D helicopter ourselves gave us a much better understanding for how a PID controller works. 


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
