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
    Demetrios Gavalas, Akmal Rupasingha, Parishee Bajaj]\
    Cornell University \
    ECE 5730 #todo[section/time] \u{00B7} #datetime.today().display("[month repr:long] [day], [year]")
  ]
\]

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

== Software System Diagram
#include "figures/sw_diagram.typ"

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

#prompt(3)[
  can you plot to the screen just accel angle, gyro rate, beam angle, and the motor pwm command that is already there. minimal changes no comments
]
#reply[
  I reduced the display to four plots in `imu_demo.c`. I did not build or run it.

  - *Gyro rate:* the top plot shows only the Y-axis gyro, in red, at plus or minus 250 deg/s.
  - *Accel angle:* a new plot, clamped to 0 to 180 degrees. I added a global `accel_angle_deg` and a history buffer, because the ISR only kept the accel angle in a local variable.
  - *Beam angle and motor PWM:* the existing plots, with the beam angle title renamed.
  - *Removed:* the gyro X and Z traces, the three accel (g) traces and the accel low-pass filter that fed them. Axis titles and labels were updated to match.

  I noted that the gyro plot used the raw `gyro[1]`, so its sign might not match the filter's convention.
]
#ide-note[Edits applied directly to `imu_demo.c`.]

#prompt(4)[
  the beam angle sign is flipped and offset -30 deg from 0.
]
#reply[
  I flipped the sign by changing `ANGLE_SIGN` to -1 and `GYRO_SIGN` to +1 in `imu_demo.c`, together, because the gyro and accel must agree for the complementary filter to work. I did not change the -30 degree offset and did not build or run it.

  I explained that the zero is captured from the first accel sample at start-up, so an offset like this usually means the arm was not fully down at power-up. I also noted that the plots clamp to 0 to 180, so a negative reading sits on the bottom line.
]
#ide-note[Sign flip applied. The offset was left for the user to retest.]

#prompt(5)[
  HOW DO I ADD AN I TERM? JUST EXPLAIN PLS
]
#reply[
  Explanation only, no code changes. I described adding a static `integral` state, accumulating `error * dt` each ISR tick, and adding `Ki * integral` to the output. I recommended anti-windup (clamp the integral or stop accumulating while saturated), resetting the integral when the IMU is not ready or the target changes, and starting with a small `Ki`, about 0.1 times `Kp`. I offered to make the edits.
]
#ide-note[No code was changed for this prompt. The user added the I term to the output equation themselves afterwards.]

#prompt(6)[
  FIX DT MACRO EXPLANSION ERROR. also the KD signs are correct you can remove the defines for those
]
#reply[
  I fixed the macro and removed the two sign defines in `imu_demo.c`. I did not build it.

  - *`DT`:* it was written as `#define DT = CLKDIV * ...`. The `=` made every use expand into a syntax error. It is now fully parenthesized: `(CLKDIV * (WRAPVAL + 1) / (float)clock_get_hz(clk_sys))`.
  - *Sign defines:* `ANGLE_SIGN` and `GYRO_SIGN` are removed. The accel angle is now `-relative_angle` and the gyro rate is plain `fix2float15(gyro[1])`.

  A grep showed no remaining references to either define.
]
#ide-note[Applied. The `DT` define and the `D_MULT` define were already in the file from the user's own edits.]

#prompt(7)[
  is the PWM command signal overflowing or wrapping? seeing weird pulsing behavior when the plotted motor PWM command is around 6000
]
#reply[
  Analysis only, no code changes. I found no software overflow: `output` is a float clamped to 0 to `WRAPVAL` before the `(int)` cast. I gave three likely causes.

  - *Integral windup:* `integral` accumulated without limit while the output was pinned, so the output stayed saturated after the error flipped.
  - *One-count glitch:* a level equal to the wrap value is 6000 of 6001 counts high, which leaves a short low pulse each period.
  - *Noise on the Kd term:* the raw gyro was not filtered.

  I suggested clamping the integral first and then the `WRAPVAL + 1` clamp.
]
#ide-note[No code was changed for this prompt.]

#prompt(8)[
  clamp the integral term assignment

  #ide-note[Selected in the IDE: `if (output > WRAPVAL) output = WRAPVAL;` in `imu_demo.c`.]
]
#reply[
  I clamped the integral right after the accumulate line in `imu_demo.c`. It was limited to plus or minus `WRAPVAL / Ki`, so `Ki * integral` could not exceed full scale, and the clamp was skipped when `Ki` was 0 to avoid dividing by zero. I did not build or run it.

  I did not change the output clamp and did not reset `integral` when the IMU dropped out.
]
#ide-note[Applied. This symmetric clamp was revised later in prompts 12 and 13.]

#prompt(9)[
  print the target angle on the screen next to accel angle. accel angle should be beam angle
]
#reply[
  I changed the on-screen readout in `imu_demo.c` to `Beam angle: X.X  Target: N`. The old label said "Accel angle" but was already printing `beam_angle_deg`, so only the label was wrong. The readout fits in the 40 character text buffer, and `snprintf` would truncate rather than overflow. I did not build or run it.
]
#ide-note[Applied.]

#prompt(10)[
  can you print kp ki kd on the screen next to target angle. also make the user interface such that if i hit enter on the prompt it preserves the old value
]
#reply[
  I made two changes in `imu_demo.c`, not built or run.

  - *Display:* the readout line now shows the beam angle, target, `Kp`, `Ki` and `Kd`. I enlarged `screentext` to 80 characters so it fits.
  - *Serial prompts:* an empty line makes `sscanf` return `EOF`, which I treat as "keep the current value". The target angle, `Kp`, `Ki` and `Kd` prompts each print "kept at ..." in that case. Bad input still gets the old error message and leaves the value unchanged.

  This assumes the serial read returns an empty string for a blank enter.
]
#ide-note[Applied.]

#prompt(11)[
  what is being plotted on the screen for motor pwm command? i am seeing that sometimes it seems to bug out and wrapval becomes the floor rather than the ceiling. and the motor is pulsing like 5000...0...0...5000
]
#reply[
  Explanation only, no code changes. The cyan trace is `pwm_history[]`, which stores `control` (the clamped value written to the PWM hardware) every tenth tick, scaled so 0 is the bottom line and 6000 is the top. It cannot show a value outside that range.

  I thought the controller was getting stuck at 0 for stretches because the output cannot go negative and my symmetric integral clamp allowed negative windup. That would explain a pattern like 5000, 0, 0, 5000. I suggested a one-sided integral clamp and checking whether `Kp` was high enough to saturate on its own.
]
#ide-note[No code was changed for this prompt. The diagnosis was corrected in the next two prompts.]

#prompt(12)[
  what im saying is that at some points the motor command signal will not go below 5000. but looking at the code the ceiling should be WRAPVAL / 5000.
]
#reply[
  I clarified that the ceiling is `WRAPVAL`; the `/ Ki` is only in the integral clamp. The floor near 5000 was the integral term: `Ki * integral` could reach plus 6000, so when it was wound up the output stayed near 5000 even with negative error. I also found that the integral accumulated while the output was forced to 0 (IMU not ready), and that changing `Ki` left the old `integral` in place.

  I changed the integral handling in `imu_demo.c`: it integrates only when the IMU is ready and the zero is set, resets otherwise, and was clamped to 0 up to `WRAPVAL / Ki`. I did not build or run it.
]
#ide-note[Applied, then corrected in the next prompt because the one-sided clamp was wrong.]

#prompt(13)[
  wait the integral term can be negative. the output / motor control signal can't be.
]
#reply[
  The user was right: the integral can legitimately be negative and only the final output must be clamped at 0. I restored the symmetric clamp of plus or minus `WRAPVAL / Ki` and added conditional integration (anti-windup).

  - If the previous tick's `control` was pinned at the maximum with a positive error, or pinned at 0 with a negative error, the integral stops accumulating.
  - The integral still resets to 0 when the IMU is not ready, the zero is not set, or `Ki` is 0.

  I did not build or run it.
]
#ide-note[Applied. This replaced the one-sided clamp from the previous prompt.]

#prompt(14)[
  sometimes we randomly lose beam angle data like it doesn't change is it possible that we are getting stuck somewhere?
]
#reply[
  Explanation only, no code changes. I traced it to `mpu6050_read_raw` in `mpu6050.c`.

  - It ignores the return values of its four I2C calls, so a failed read leaves the uninitialized `buffer` holding the previous call's bytes and the stale data is used as a fresh sample. That would freeze the angle.
  - `imu_ok` is never cleared after the first probe, so the retry thread never recovers a hung bus.
  - Four 250 microsecond timeouts can use up the whole ISR period and starve core 0.

  I proposed making the read return success or failure, skipping the angle update on failure, and clearing `imu_ok` after repeated failures.
]
#ide-note[No code was changed for this prompt. A partial fix was made in prompt 23.]

#prompt(15)[
  do you think its possible we are overflowing anywhere becasue we see really weird behavior when we max out hte pwm, the motor keeps turning on and off
]
#reply[
  Explanation only, no code changes. I checked the gyro and accel scaling, `Kd * D_MULT * gyro[1]`, the output cast, the plot arithmetic and the integral, and found no real overflow. I listed three likelier causes.

  - Supply sag or electrical noise at full current, which can brown out the Pico or corrupt I2C.
  - The one-count glitch at exactly 6000 counts.
  - A genuine limit cycle if `Kp` or `Kd` is large.

  I suggested capping the maximum command to test whether the problem is electrical, applying the `WRAPVAL + 1` clamp, and adding the I2C failure checks.
]
#ide-note[No code was changed for this prompt.]

#prompt(16)[
  can we clamp to wrap val + 1?
]
#reply[
  I changed the output clamp in `imu_demo.c` to `WRAPVAL + 1`, so at full output the level is 6001 against a wrap of 6000. The hardware then holds the pin solidly high instead of producing a one-count low pulse each period. The plot and the integral clamp still use `WRAPVAL`. I did not build it or try it on the motor.
]
#ide-note[Applied, then superseded by the duty ceiling in prompt 27.]

#prompt(17)[
  can we clamp the beam angle to never go below 0 or above 180
]
#reply[
  I clamped `beam_angle_deg` to 0 to 180 degrees right after the filter update in `imu_demo.c`. Because it clamps the value itself, the controller, the readout and the plot all see the limited angle. I noted that a start below the zero reference would pin at 0. I did not build or run it. The file had also changed on disk since I last read it; the edit applied cleanly.
]
#ide-note[Applied, then revised in prompts 18, 20 and 25.]

#prompt(18)[
  can you just set it to the last beam_angle to what it was before rather than just setting it to 0 or 180, because right now if it goes past 180 it goes to negative -180
]
#reply[
  I put the filter result into a local `new_angle` and only stored it into `beam_angle_deg` when it was between 0 and 180, so out-of-range results kept the previous value. I also removed a half-finished line (`if (beam_angle_deg < 0.0f)` with no body) left after the clamp, which would not have compiled. I noted that if the accel angle stays out of range the angle stays frozen, and the gyro integration stops with it. I did not build or run it.
]
#ide-note[Applied. The user asked for a clamp again in prompt 20.]

#prompt(19)[
  print imu_ok to the vga screen
]
#reply[
  I added a status line `imu_ok: N` at y = 52 on the VGA display in `imu_demo.c`; the red "IMU NOT DETECTED" message stayed. The IDE notes about `screentext` were only the spell checker. I pointed out that `imu_ok` is only cleared at start-up, so it would read 1 even during a mid-run dropout, and offered the I2C failure handling again. I did not build or run it.
]
#ide-note[Applied. The line was removed in prompt 33.]

#prompt(20)[
  right now beam angle wraps around from 180 to 0, but if it goes past 180 we just want to it to be 180 and if it goes past 0 to just be 0 can you make that change

  #ide-note[Selected in the IDE: `beam_angle` on line 228 of `imu_demo.c`.]
]
#reply[
  I changed the hold-last-value logic back to a clamp: `new_angle` is limited to 0 and 180 and then stored in `beam_angle_deg`, in `imu_demo.c`. I noted that the 180 to 0 wrap probably comes from the accel angle, which wraps across the atan2 boundary, and said that clamping `accel_angle` could be the next step. I did not build or run it.
]
#ide-note[Applied, then replaced by the fix in prompt 25.]

#prompt(21)[
  can you tell me what the deal is with imu_ok again like we only check it onces?
]
#reply[
  Explanation only, no code changes. `imu_ok` is set once by `imu_probe()` in `main`. The retry thread only acts when it is false, and the ISR only reads the sensor when it is true. Nothing sets it back to false, and `mpu6050_read_raw` ignores failed reads. A mid-run dropout therefore freezes the data while the flag still reads 1, and the bus recovery never runs. I proposed the same three-part fix: return success or failure from the read, count consecutive failures, then clear `imu_ok`.
]
#ide-note[No code was changed for this prompt.]

#prompt(22)[
  so when would imu_okay be low?
]
#reply[
  Explanation only, no code changes. It is only low before the first successful probe, meaning the WHO_AM_I read times out at boot, and until the retry thread succeeds. Once high, nothing sets it low again as the code stood, so a reset such as a brownout is the only other way to see it low.
]
#ide-note[No code was changed for this prompt.]

#prompt(23)[
  okay can you make it so we constantly probe imu_okay and then print it?
]
#reply[
  I added a continuous check in `imu_demo.c`. I did not build or run it.

  - *`imu_present(timeout_us)`:* a new function that does only the WHO_AM_I read with no reset. `imu_probe()` now calls it and then resets and configures the sensor.
  - *In the ISR:* every 10th tick, while `imu_ok` is high, it sets `imu_ok = imu_present(250)`. A failure drops `imu_ok` to 0, the ISR skips I2C and the output goes to 0. The retry thread then runs the bus recovery and re-probes once a second.
  - *Where it runs:* I put the check in the ISR, not the retry thread, so the ISR and a thread never use the I2C bus at the same time on core 0.

  The on-screen `imu_ok` line already printed the flag. I noted that a single missed check now cuts the motor for about a second, and that `mpu6050_read_raw` still ignores failed reads between probes.
]
#ide-note[Applied.]

#prompt(24)[
  what lines do you check it in the isr
]
#reply[
  Explanation only. The check is at lines 209 to 213 of `imu_demo.c`: the static tick counter, the `if (imu_ok && ++probe_tick >= 10)` test, and `imu_ok = imu_present(250)`. It sits just before the `if (imu_ok)` block that reads the sensor, so a failed check stops the read on the same tick.
]
#ide-note[No code was changed for this prompt.]

#prompt(25)[
  can you fix the beam angle clamp to go from 0 ... 180. importantly when it goes over 180 it goes to like -180
]
#reply[
  I fixed the wrap at its source in `imu_demo.c`. The accel angle wrapped to about -180 once the arm passed 180, and the filter blended toward that.

  - *Accel angle:* values below -90 degrees are treated as past 180 (add 360), then the angle is clamped to 0 to 180. Values from -180 to -90 become 180 and values from -90 to 0 become 0.
  - *Beam angle:* the filter result is clamped to 0 to 180 and stored.

  This replaced a snap-to-180 block the user had added, which handled going below 0 but not above 180. It assumes the arm never goes more than 90 degrees below the starting zero. I did not build or run it.
]
#ide-note[Applied. This replaced the user's own snap-to-180 edit.]

#prompt(26)[
  when i set a wrap value < 4000 i am no longer able to send serial messages to the device. i do get the initial set target value
]
#reply[
  Explanation only, no code changes. The control ISR fires once per PWM period, `CLKDIV * (WRAPVAL + 1) / 150 MHz`: 1.0 ms at 6000, 0.67 ms at 4000 and 0.33 ms at 2000. The ISR does two I2C transfers at 400 kHz plus software float math, which I estimated at 0.5 to 0.7 ms, so near a 4000 wrap it runs almost back to back and starves the serial thread on core 0. I noted these were estimates and that GPIO 14, which toggles each ISR call, can be scoped to measure it.

  Options I gave: raise `CLKDIV` as `WRAPVAL` falls to keep the same period, make the ISR cheaper (one 14-byte burst read, a faster I2C baud rate), or decouple the control loop from the PWM period with a 1 kHz timer interrupt.
]
#ide-note[No code was changed for this prompt. The user restored `WRAPVAL` to 6000.]

#prompt(27)[
  i want to put a ceiling on the motor rate i.e. the pwm duty cycle how could i do that? i put wrapval back to 6000.

  the symptom i am observing is that when the motor command signal is close to the max, it stops giving a constant fan output and becomes a periodic 1 Hz pulse of max and zero. the motor command plot stays high. think there is an issue with how we are driving the PWM signal.
]
#reply[
  I added a duty ceiling to `imu_demo.c`: `MAX_DUTY` (0.9, so 90 percent) and `MAX_CONTROL`, which is `WRAPVAL * MAX_DUTY`. The output clamp, the saturation check and the integral clamp all use `MAX_CONTROL`, replacing the `WRAPVAL + 1` clamp. The plot is still scaled to the full `WRAPVAL`. I did not build or run it.

  For the 1 Hz pulsing I suspected the constant 100 percent duty from my `WRAPVAL + 1` change: a driver with a bootstrap capacitor needs off pulses, and some drivers or ESCs treat a stuck-high input as a fault, or have overcurrent protection that cycles about once a second. I suggested lowering the cap further to find where it stops, watching for repeated boot messages on serial, and checking whether `imu_ok` flips at 1 Hz.
]
#ide-note[Applied.]

#prompt(28)[
  I added a button from GPIO 15 to ground. it needs to have the following behavior by setting the target value. :

  - Before time=0, while holding the button, the beam should be hanging vertically down (motor off)
  - When the button is released at time=0, target angle should be set to horizontal
  - At time=5 seconds, target angle should be set to approximately 30 degrees above horizontal
  - At time=10, target angle should be set to approximately 30 degrees below horizontal
  - At time=15, target angle should be set to horizontal
]
#reply[
  I implemented the button sequence in `imu_demo.c`. I did not build or run it.

  - *Pin conflict:* GPIO 15 was already a debug toggle in the VGA draw thread, so I removed that toggle. The pin is now an input with the internal pull-up, and a pressed button reads low.
  - *Button thread:* a new `protothread_button` polls every 10 ms. While held, `motor_hold` forces the output to 0 and resets the integral, and the target is 0. On release it starts a timer.
  - *Targets:* the angle scale has 0 as hanging down and 90 as horizontal, so the schedule was 90 at release, 120 at 5 s, 60 at 10 s and 90 at 15 s, after which the sequence stops so serial input works again.
]
#ide-note[Applied. The user later asked for the sequence to be driven by the beam angle and then back to timed (prompts 29 and 33).]

#prompt(29)[
  can we hard code the constant prarams as kp = 20 , ki = 90 and kd = 7 also the requirements should be met un one sweep for example once we hit 120 we should switch the target anghle to 150 and so on so we need logic that checks the beam angle with the target verfieis then switches to the next target
]
#reply[
  I made two changes in `imu_demo.c`, not built or run.

  - *Gains:* `Kp` = 20, `Ki` = 90 and `Kd` = 7 became the initial values, and I removed the three serial gain prompts.
  - *Waypoints:* the button thread stepped through `90, 120, 60, 90`, advancing when the beam was within 3 degrees of the current waypoint, with no timer.

  I flagged that 150 does not match the spec (30 degrees above horizontal is 120 here), that the 5, 10 and 15 second marks were gone, and that `Ki` = 90 may overshoot.
]
#ide-note[Applied. The gain prompts were restored in the next prompt, and the timed sequence in prompt 33.]

#prompt(30)[
  actually insteaad of hardcoding the param values those calues should jut be our initials we should still have the ability to change the params with the serial monitor like before
]
#reply[
  I restored the `Kp`, `Ki` and `Kd` serial prompts in `imu_demo.c`, including the keep-on-enter behavior. The values 20, 90 and 7 remain the starting values. I did not build or run it.
]
#ide-note[Applied.]

#prompt(31)[
  the initial target though should be 0
]
#reply[
  The target already started at 0 in its declaration and while the button was held. The remaining problem I could see was that with the button unpressed at power-up, the thread saw a release and started the sequence, so the target jumped to 90 at boot. I added an `armed` flag in the button thread so the sequence only starts after the button has been held and then released. I noted this means one press and release is needed after power-up, and asked the user to clarify if they meant something else. I did not build or run it.
]
#ide-note[Applied. This start-up gating was relaxed in prompt 34.]

#prompt(32)[
  can you draw the on the vga to display whether the gpio 15 button is pressed or not pressed. also please tell me the expected behavior in order to execute this sweep because right now after flashing the code the target angle is 90 not zero?
]
#reply[
  I added `Button: PRESSED` or `Button: NOT PRESSED` to the status line under the beam angle readout, reading GPIO 15 live. I did not build or run it.

  I described the expected sequence: power up with target 0 and the motor off, hold the button for the motor off and target 0, release for 90 and the motor on, then the targets advance, and pressing again resets to the hold state. For the target reading 90 after flashing I listed three possibilities: the sequence had already finished at 90, the old build was still flashed, or the button reading flickers.
]
#ide-note[Applied. The user confirmed the button works.]

#prompt(33)[
  button works remove the print and the imu check print on the screen.

  the sequence should be timed and just change the target angle at the times i gave you.
]
#reply[
  I removed the `imu_ok` and Button status line from the VGA display in `imu_demo.c`, leaving the beam angle, target and gain readout and the red "IMU NOT DETECTED" warning. I replaced the waypoint logic with the timed schedule: 90 degrees at release, 120 at 5 s, 60 at 10 s and 90 at 15 s, after which the sequence stops. I did not build or run it.
]
#ide-note[Applied.]

#prompt(34)[
  i want the ability to get to a target angle without going through the sequence. we only want to go thbrough the sequence when we hit the button otherwise we should be able to enter a target value and see the motor shift to that value
]
#reply[
  I changed `motor_hold` to start `false` in `imu_demo.c`. Before, it started `true` and the motor stayed off until the button had been pressed and released once. Now the motor is enabled at boot with the sequence idle, a target entered over serial moves the motor, and the button thread only touches the target during a sequence. Pressing the button still holds the motor off with target 0, and releasing it runs the timed sequence. I noted that a button press overwrites any serial target, and that a target typed during the 15 second sequence is overwritten. I did not build or run it.
]
#ide-note[Applied.]

#prompt(35)[
  can you make the timescale for the graphs like 10x longer so we can see 10x more duration in the window

  #ide-note[Selected in the IDE: `draw` on line 190 of `imu_demo.c`.]
]
#reply[
  I changed `threshold` from 10 to 100 in `imu_demo.c`. The graphs store one sample every `threshold` ISR ticks of about 1 ms, so the 530 pixel window now covers about 53 seconds instead of 5.3. The traces update and scroll ten times slower, and quick transients can be missed between samples. The control loop is unaffected. I did not build or run it.
]
#ide-note[Applied.]
