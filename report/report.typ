#set document(
  title: "ECE 4760 / 5730 Lab 3: PID Control of a 1D Helicopter",
  author: ("Demetrios Gavalas", "Parishee Bajaj", "Akmal Rupasingha"),
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

// ============================================================
= Introduction
// ============================================================
Motors are used in many applications from aerospace to consumer electronics, and due to the mechanical nature of motors, they are subject to wear and tear which can cause discrepancies in how they behave in different conditions and over time. One way to address these nonlinearities is to use a feedback control system, adjusting the input based on the output to achieve a desired behavior. In this lab, we implemented a PID controller to control the angle of a physical one-degree of freedom helicopter. The setup consists of a fan on a lever arm mounted onto a hinge, with an IMU mounted on the lever arm to pass relevant gyroscope & acceleration information, a motor controller to drive the fan, and a VGA display to showcase relevant information, all of which were interfaced with the RP2350. After implementation, we spent time in testing & tuning the PID parameters to reduce settling time & accuracy.  

// ============================================================
= Design and Testing Methods
// ============================================================

== System overview
Our 1D helicopter is a lever arm on a hinge with a small fan motor at the free end. An MPU6050 IMU on the arm measures acceleration and rotation rate, and the Pico uses these readings to estimate the arm angle and run a PID controller that sets the motor’s PWM duty cycle. The PWM signal drives the motor through an optoisolator, so the motor circuit stays electrically separate from the Pico. The whole control loop runs inside one interrupt. That interrupt comes from the same PWM slice that drives the motor, and it fires every time the PWM counter wraps. With a 150 MHz clock, a clock divider of 25, and a wrap value of 6000, the PWM and the control loop both run at about 1 kHz. Each time it runs, the interrupt reads the IMU over I2C, combines the accelerometer and gyro readings with a complementary filter to get the beam angle, runs the PID controller, and writes the new duty cycle to the PWM hardware. On core 0, one thread takes serial commands for the target angle and the three gains, one watches the button and runs the demo sequence, and one tries to reconnect the IMU once a second if it stops responding. Core 1 only draws the VGA display.

We measure the beam angle in degrees, with 0° when the arm hangs straight down, 90° when it is horizontal, and 180° when it points straight up. Using our convention, the handout’s button sequence of horizontal, 30° above, 30° below, and back to horizontal becomes targets of 90°, 120°, 60°, and 90°. The program zeroes the accelerometer angle when it starts, so the arm has to hang straight down whenever the Pico is powered on or reset.


== Hardware
The motor was mounted onto the PCB lever arm that we were provided in class. The lever arm was a PCB that had a fan mounted on one end, and the IMU & hinge mounted on the other end. The electrical connections, motor +/-, 3.3V, GND, SDA, & SCL were all made using screw terminals. 

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
After implementing all three terms of the PID controller, we noticed that the motor would behave strangely if we tried to reach large changes in angle very quickly. It would start oscillating, with the motor turning on and off uncontrollably, and we would also lose I2C connection with the IMU. We realized that this was due to the fact that to reach this large change in angle, the PID controller would output a very large PWM duty cycle. This would typically be fine, but if the controller was not moving at all, i.e. PWM duty cycle of 0, the large change in duty cycle would cause the motor to draw a lot of current and fail. To address this, we artificially limited the maximum PWM duty cycle to 70% of what the maximum, to reduce the largest change in duty cycle that the controller could output.


// ============================================================
= Documentation
// ============================================================

== Schematic
#figure(
  image("figures/schematic.png", width: 100%),
  caption: [Schematic of the motor drive, IMU, and user-interface circuits.],
) <fig:schematic>

== Software System Diagram
#include "figures/sw_diagram.typ"

== Mechanical assembly
The helicopter is a lever arm that hangs from a hinge, with the fan motor mounted at the free end. The hinge is screwed onto a wooden block, which holds the arm up so it can hang straight down and swing freely. The arm itself is a printed circuit board. The motor points so its thrust lifts the arm. The MPU6050 plugs into header pins on the PCB near the hinge. Its Y axis lines up with the hinge, which the angle code depends on.
#figure(
  image("figures/assembly.png", width: 65%),
  caption: [Lever arm, motor, hinge, and IMU.],
) <fig:assembly>

// ============================================================
= Results
// ============================================================
== Final Product
Our finished 1D helicopter is a PCB lever arm hanging from a hinge on a wooden block, with a fan motor at the free end and an MPU6050 IMU near the hinge. The RP2350 reads the IMU, estimates the arm angle, and uses a PID controller to drive the motor so the arm holds whatever angle we choose. We set the target angle and the three gains from a serial terminal while the program runs, and the new values take effect right away. The final gains were $K_p = 20$, $K_i = 90$, and $K_d = 7$, and the program also starts with these values. A button on GPIO 15 runs the preset sequence of target angles asked for in the final/week 3 demo. The VGA display plots four live traces: gyro rate, accelerometer angle, filtered beam angle, and motor command. The top line of the screen also shows the current beam angle, target, and gains.

== Four-angle demo sequence
The button runs a fixed test so we can observe how the controller handles steps both up and down. The program checks the button every 10 ms. While the button is held, the motor is forced off, the integral term is reset to zero, and the target is set to 0°, so the arm hangs straight down. When the button is released, a timer starts and the target changes on its own. It is 90° (horizontal) for the first 5 seconds, 120° (30° above horizontal) from 5 to 10 seconds, 60° (30° below horizontal) from 10 to 15 seconds, and then back to 90°, where it stays. The sequence uses whatever gains are set at the time. @fig:sequence is a VGA capture of one run. The yellow trace is the beam angle and the cyan trace is the motor command. The plots save one sample every 100 ms and the whole sequence fits on one screen. Because the motor command is capped at 70% duty, the cyan trace never goes above 70% of its plot height.
#figure(
  image("figures/vga-sequence.png", width: 100%),
  caption: [VGA capture of the 90#sym.degree to 120#sym.degree target-angle transition, showing gyro rate, accelerometer angle, filtered beam angle, and motor PWM command.],
) <fig:sequence>

#figure(
  image("figures/vga-full-sequence.png", width: 75%),
  caption: [VGA traces of the complete four-step angle sequence (0#sym.degree #sym.arrow 90#sym.degree #sym.arrow 120#sym.degree #sym.arrow 60#sym.degree #sym.arrow 90#sym.degree), showing gyro rate, accelerometer angle, filtered beam angle, and motor PWM command.],
) <fig:full-sequence>
// ============================================================
= Lab Questions
// ============================================================

== Question 1
_How did you select the three PID gains?_
We tuned the gains by hand, one term at a time, typing new values into the serial terminal while the arm was running so we could see the effect right away. We started with only proportional control, with $K_i$ and $K_d$ set to 0. We raised $K_p$ until the arm could lift off and get close to the target. With a low $K_p$ the arm sagged well below the target, and raising $K_p$ brought it closer but made it overshoot and swing back and forth. Next we added the derivative term to damp the motion. Our first version did not work: it used the raw gyro reading with the wrong sign, so instead of slowing the arm down it pushed it the same way it was already moving. The raw reading was also so large that even a small $K_d$ slammed the motor between off and full power whenever the arm moved. We fixed the sign and scaled the gyro reading down by $10^{-5}$ so that $K_d$ could be a normal-sized number. Last, we added the integral term to remove the steady-state error, since with P and D alone the arm settled a little below the target. The motor needs steady thrust to hold the arm up, and a P controller only gives thrust when there is some error. We stop the integral from growing while the motor is at its limit, so it cannot build up and cause a large overshoot later. We ended with $K_p = 20$, $K_i = 90$, and $K_d = 7$, because this set moved the arm to each new target quickly with little overshoot and stayed stable at every angle in the demo sequence.

== Question 2
_What are the maximum and minimum angles that give stable behavior with your gains?_

The minimum angle that we could stably hold was 15 degrees. This was due to the fact that our electrical connections were a bunch of solid core wires connected to screw terminals. These wires introduced some unwanted tension & pressure on the lever arm, preventing a full range of motion. The maximum angle that we could stably hold was 175 degrees. This was also limited by mechanical constraints since the screw attaching the lever arm to the hing would hit the wood plank at this angle preventing further motion.

== Question 3
_What are the settling times at different angles?_
The settling time from 0 to 90 degrees is 1.77 seconds. The settling time from 90 to 120 degrees is 3.87 seconds. The settling time from 120 to 60 degrees is 1.37 seconds. 



// ============================================================
= Conclusions
// ============================================================

One of the most interesting parts of this lab was seeing what each PID term actually did to the arm. With only proportional control, the arm oscillated. The P term works like a spring: the farther the arm is from the target, the harder the motor pushes it back. But the P term doesn’t care how fast the arm is already moving, so the arm overshoots the target and swings back and forth. When we added the derivative term, the oscillation was greatly reduced. Our D term uses the gyro rate, so it pushes against the arm’s motion, a lot like friction. The faster the arm moves toward the target, the more the D term slows it down, so it settles instead of overshooting. Before this, we knew that D adds damping. Watching the arm go from swinging back and forth to settling in a smoother fashion made that idea much easier to understand.

Building and tuning the 1D helicopter ourselves gave us a much better understanding for how a PID controller works. 

If we were to implement this system again, we'd try to create better connections in between our several PCBs and the RP2350. These connnections often came lose during testing and would cause the system to fail. Additionally, with the way we had the rig, the wiring prevented our lever arm from naturally hanging at 0 degrees, and instead would be at 15 degrees if not held down. We also noticed, that there was some degree of freedom left and right with the hinges that we were working with which added a degree of freedom to our system that we did not account for. In the future, we'd try to make both our electrical & mechanical connections more robust. 

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

#prompt(2)[
  can you make this a complementary accel / gyro filter? simple change

  #ide-note[Selected in the IDE: the three `filtered_accel_*` low-pass lines in `imu_demo.c`.]
]
#reply[
  I swapped the accel low-pass for a complementary filter in the ISR in `imu_demo.c`. I haven't built it or run it on the hardware.

  - *Filter:* `beam_angle_deg = COMP_ALPHA * (beam_angle_deg + gyro_rate * dt) + (1 - COMP_ALPHA) * accel_angle`. I set `COMP_ALPHA` to 0.98.
  - *Gyro axis:* the rate comes from `gyro[1]`, the same axis your Kd term already uses.
  - *`dt`:* it is computed from `CLKDIV`, `WRAPVAL` and the system clock, so it follows the ISR period. That is about 1 ms if the Pico 2 runs at 150 MHz.
  - *Raw accel in:* the filter uses the unfiltered accel for the angle, because the old 1/16 low-pass would add lag. `filtered_accel_*` is still computed, but only for the plots.
  - *Start-up:* the angle starts at 0, which is the zeroed accel angle.
  - *`GYRO_SIGN`:* I added this define, set to 1.0, because I could not check the gyro direction against `ANGLE_SIGN`. If the angle jumps the wrong way when you rotate the beam quickly, change it to -1.0f.

  The IDE shows an "Unknown word" notice on `CLKDIV`, but it is defined a few lines below the new defines and is only used inside the ISR, so I do not expect a compile error.
]
#ide-note[Accepted as written. The change was a small edit to the angle estimation in the ISR.]
