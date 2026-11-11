/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 *  - GPIO 4 ---> PWM output
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
// Include custom libraries
#include "VGA/vga16_graphics_v3.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// We will start drawing at column 81
#define LEFT_EDGE 81 

// Rescale the measurements for display
#define OLD_RANGE 500.  // (+/- 250)
#define NEW_RANGE 75.  // Height of each raw-sensor plot
#define OLD_MIN -250. 
#define OLD_MAX 250. 
#define ACCEL_SCALE 120.0

// Place the plots
#define TOP_PLOT_BOTTOM 145
#define BOTTOM_PLOT_BOTTOM 250
#define ANGLE_PLOT_TOP 280
#define ANGLE_PLOT_BOTTOM 355
#define PWM_PLOT_TOP 385
#define PWM_PLOT_BOTTOM 460
#define PLOT_WIDTH 530

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// Accelerometer tilt for rotation about the IMU Y axis.
// Keep the arm fully down at startup: the first nonzero sample sets zero.
static float angle_zero_deg = 0.0f;
static bool angle_zero_set = false;
volatile float beam_angle_deg = 0.0f;

// Arrays for circular display buffer
fix15 gy[PLOT_WIDTH] ;
float accel_angle_history[PLOT_WIDTH] ;
volatile float accel_angle_deg = 0.0f;
float angle_history[PLOT_WIDTH] ;
int pwm_history[PLOT_WIDTH] ;

// Pointer to head of buffer
volatile int head = 0 ;

// character array
char screentext[80];

// draw speed
volatile int threshold = 100 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// Some paramters for PWM
#define COMP_ALPHA 0.98f   /// gyro weight; time constant ~ alpha*dt/(1-alpha)
#define WRAPVAL 6000
#define MAX_DUTY 0.7f
#define MAX_CONTROL ((int)(WRAPVAL * MAX_DUTY))
#define CLKDIV  25.0
#define DT (CLKDIV * (WRAPVAL + 1) / (float)clock_get_hz(clk_sys))
uint slice_num ;

// GPIO we're using for PWM
#define PWM_OUT 4

#define BUTTON_PIN 15
volatile bool motor_hold = false ;

// PWM duty cycle
volatile int control ;

// Set once the MPU6050 answers on the I2C bus. If it doesn't, the ISR
// skips the I2C reads so core 0 isn't starved by timeouts.
volatile bool imu_ok = false ;

// PID Constants

volatile float Kp = 20.0f;

// Set to the measured PWM needed to hold the arm horizontal (90 degrees).
// Zero leaves feedforward disabled until calibrated.
#define HOVER_PWM 0.0f
volatile float target_angle_deg = 0.0f;
volatile float Ki = 90.0f;
volatile float Kd = 7.0f;


// Free the I2C bus and (re)initialize the I2C peripheral. If the Pico is
// reset mid-transfer while the MPU6050 stays powered, the IMU can hold SDA
// low forever. Clocking SCL until it lets go, then sending a STOP, frees it.
void i2c_bus_recover() {
    gpio_init(SDA_PIN) ;
    gpio_init(SCL_PIN) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;
    gpio_put(SDA_PIN, 0) ;
    gpio_put(SCL_PIN, 0) ;
    // Open-drain emulation: output = drive low, input = released (pulled high)
    for (int i = 0; i < 9 && !gpio_get(SDA_PIN); i++) {
        gpio_set_dir(SCL_PIN, GPIO_OUT) ; sleep_us(5) ;
        gpio_set_dir(SCL_PIN, GPIO_IN) ;  sleep_us(5) ;
    }
    // STOP condition: SDA low->high while SCL is high
    gpio_set_dir(SCL_PIN, GPIO_OUT) ; sleep_us(5) ;
    gpio_set_dir(SDA_PIN, GPIO_OUT) ; sleep_us(5) ;
    gpio_set_dir(SCL_PIN, GPIO_IN) ;  sleep_us(5) ;
    gpio_set_dir(SDA_PIN, GPIO_IN) ;  sleep_us(5) ;

    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
}

// Check that the MPU6050 answers (WHO_AM_I register 0x75), and configure it if so
bool imu_present(uint32_t timeout_us) {
    uint8_t reg = 0x75 ;
    uint8_t who = 0 ;
    return i2c_write_timeout_us(I2C_CHAN, ADDRESS, &reg, 1, true, timeout_us) == 1 &&
           i2c_read_timeout_us(I2C_CHAN, ADDRESS, &who, 1, false, timeout_us) == 1 ;
}

bool imu_probe() {
    if (imu_present(1000)) {
        mpu6050_reset() ;
        return true ;
    }
    return false ;
}


void drawAxes() {
    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    const int tops[4] = {70, 175, ANGLE_PLOT_TOP, PWM_PLOT_TOP};
    const char *titles[4] = {"Gyro rate (deg/s)", "Accel angle (deg)",
                             "Beam angle (deg), down = 0", "Motor PWM command"};
    const char *upper[4] = {"+250", "+180", "+180", "6000"};
    const char *middle[4] = {"0", "90", "90", "3000"};
    const char *lower[4] = {"-250", "0", "0", "0"};
    for (int p = 0; p < 4; p++) {
        int top = tops[p];
        drawVLine(80, top, 76, CYAN);
        drawHLine(75, top, 5, CYAN);
        drawHLine(75, top + 38, 5, CYAN);
        drawHLine(75, top + 75, 5, CYAN);
        setCursor(LEFT_EDGE, top - 15); writeString((char *)titles[p]);
        setCursor(40, top); writeString((char *)upper[p]);
        setCursor(40, top + 34); writeString((char *)middle[p]);
        setCursor(40, top + 67); writeString((char *)lower[p]);
    }
}

volatile int throttle = 10 ;
#define D_MULT 0.00001f

// Interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));
    gpio_put(14, !gpio_get(14)) ;

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    static int probe_tick = 0;
    if (imu_ok && ++probe_tick >= 10) {
        probe_tick = 0;
        imu_ok = imu_present(250);
    }
    if (imu_ok) {
        mpu6050_read_raw(acceleration, gyro);
        float accel_x = fix2float15(acceleration[0]);
        float accel_z = fix2float15(acceleration[2]);
        if (accel_x != 0.0f || accel_z != 0.0f) {

            float raw_angle = atan2f(accel_x, accel_z)
                            * (180.0f / 3.14159265f);

            if (!angle_zero_set) {
                angle_zero_deg = raw_angle;
                angle_zero_set = true;
                beam_angle_deg = 0.0f;   // start filter at accel angle
            }

            float relative_angle = raw_angle - angle_zero_deg;
            // Wrap across atan2's -180/+180 boundary.
            if (relative_angle > 180.0f) relative_angle -= 360.0f;
            if (relative_angle < -180.0f) relative_angle += 360.0f;

            float accel_angle = -relative_angle;
            if (accel_angle < -90.0f) accel_angle += 360.0f;
            if (accel_angle < 0.0f) accel_angle = 0.0f;
            if (accel_angle > 180.0f) accel_angle = 180.0f;
            accel_angle_deg = accel_angle;
            float gyro_rate = fix2float15(gyro[1]);
            float new_angle = COMP_ALPHA * (beam_angle_deg + gyro_rate * DT)
                            + (1.0f - COMP_ALPHA) * accel_angle;
            if (new_angle < 0.0f) new_angle = 0.0f;
            if (new_angle > 180.0f) new_angle = 180.0f;
            beam_angle_deg = new_angle;
        }
    }

    static float integral = 0.0f;

    float error = target_angle_deg - beam_angle_deg;
    bool saturated = (control >= MAX_CONTROL && error > 0.0f) ||
                     (control <= 0 && error < 0.0f);
    if (!imu_ok || !angle_zero_set || motor_hold) integral = 0.0f;
    else if (!saturated) integral += error * DT;
    if (Ki > 0.0f) {
        float imax = MAX_CONTROL / Ki;
        if (integral > imax) integral = imax;
        if (integral < -imax) integral = -imax;
    }
    else integral = 0.0f;
    float output = Kp * error - Kd * D_MULT * gyro[1] + Ki * integral;
    if (!imu_ok || !angle_zero_set || motor_hold) output = 0.0f;
    if (output < 0.0f) output = 0.0f;
    if (output > MAX_CONTROL) output = MAX_CONTROL;
    control = (int)output;
    pwm_set_chan_level(slice_num, PWM_CHAN_A, control);

    // Increment drawspeed controller
    throttle += 1 ;
    // Maintain a circular buffer for display
    if ((throttle >= threshold)) { 
        // Zero drawspeed controller
        throttle = 0 ;

        gy[head] = gyro[1] ;
        accel_angle_history[head] = accel_angle_deg;
        angle_history[head] = beam_angle_deg;
        // Store the clamped command actually written to the PWM hardware.
        pwm_history[head] = control;

        // Move the pointer to the oldest data
        head = ((head+1) == PLOT_WIDTH) ? 0 : (head + 1) ;

    }
}

static PT_THREAD (protothread_draw(struct pt *pt))
{
    PT_BEGIN(pt) ;

    static int start ;
    static int i ;

    // Draw axes
    drawAxes() ;
    // Copy to other buffer
    copy_buffer_to_other() ;

    while(1) {
        // Wait for a buffer swap
        PT_YIELD_UNTIL(pt, draw_start_signal()) ;
        
        // Clear the buffer, draw the axes
        clearLowFrame(0, BLACK) ;
        drawAxes() ;

        // Show IMU status on screen so wiring problems are obvious
        if (!imu_ok) {
            setTextColor(RED) ;
            setCursor(LEFT_EDGE + 10, 40) ;
            sprintf(screentext, "IMU NOT DETECTED - check wiring") ;
            writeString(screentext) ;
        }

        else {
            setTextColor(WHITE) ;
            setCursor(LEFT_EDGE + 10, 40) ;
            snprintf(screentext, sizeof(screentext), "Beam angle: %.1f  Target: %.0f  Kp %.3f  Ki %.3f  Kd %.3f",
                     (double)beam_angle_deg, (double)target_angle_deg,
                     (double)Kp, (double)Ki, (double)Kd) ;
            writeString(screentext) ;
        }

        // Plot data oldest-->newest
        start = head ;
        for (i = 0; i < PLOT_WIDTH; i++) {
            drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM -
                (int)(NEW_RANGE*((float)((fix2float15(gy[start]))-OLD_MIN)/OLD_RANGE))), RED) ;
            float aangle = accel_angle_history[start];
            if (aangle > 180.0f) aangle = 180.0f;
            if (aangle < 0.0f) aangle = 0.0f;
            drawPixel(LEFT_EDGE + i, BOTTOM_PLOT_BOTTOM -
                      (int)(aangle * NEW_RANGE / 180.0f), GREEN);
            float angle = angle_history[start];
            if (angle > 180.0f) angle = 180.0f;
            if (angle < 0.0f) angle = 0.0f;
            drawPixel(LEFT_EDGE + i, ANGLE_PLOT_BOTTOM -
                      (int)(angle *
                      (ANGLE_PLOT_BOTTOM - ANGLE_PLOT_TOP) / 180.0f), YELLOW);
            drawPixel(LEFT_EDGE + i, PWM_PLOT_BOTTOM -
                      pwm_history[start] * (PWM_PLOT_BOTTOM - PWM_PLOT_TOP)
                      / WRAPVAL, CYAN);
            start = ((start+1)==PLOT_WIDTH) ? 0 : (start+1) ;
        }
    }

    PT_END(pt) ;
}


// User input thread. User sets the target angle and proportional gain.
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static int test_in ;
    static int n ;
    static float kp_in ;
    static float kd_in ;
    static float ki_in ;

    while(1) {
        sprintf(pt_serial_out_buffer, "input a target angle: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        n = sscanf(pt_serial_in_buffer,"%d", &test_in);
        if (n == EOF) {
            sprintf(pt_serial_out_buffer, "target angle kept at %d deg\n", (int)target_angle_deg);
            serial_write ;
        }
        else if (n != 1 || test_in > 180 || test_in < -180) {
            sprintf(pt_serial_out_buffer, "target angle must be between -180 and 180\n");
            serial_write ;
        }
        else {
            target_angle_deg = test_in ;
            sprintf(pt_serial_out_buffer, "target angle set to %d deg\n", test_in);
            serial_write ;
        }

        //kp input
        sprintf(pt_serial_out_buffer, "input a Kp value: ");
        serial_write ;
        serial_read ;
        n = sscanf(pt_serial_in_buffer,"%f", &kp_in);
        if (n == EOF) {
            sprintf(pt_serial_out_buffer, "Kp kept at %f\n", (double)Kp);
        }
        else if (n == 1 && isfinite(kp_in) && kp_in >= 0.0f) {
            Kp = kp_in;
            sprintf(pt_serial_out_buffer, "Kp set to %f\n", (double)Kp);
        }
        else {
            sprintf(pt_serial_out_buffer, "Kp must be finite and >= 0\n");
        }
        serial_write ;

        //ki input
        sprintf(pt_serial_out_buffer, "input a Ki value: ");
        serial_write ;
        serial_read ;
        n = sscanf(pt_serial_in_buffer,"%f", &ki_in);
        if (n == EOF) {
            sprintf(pt_serial_out_buffer, "Ki kept at %f\n", (double)Ki);
        }
        else if (n == 1 && isfinite(ki_in) && ki_in >= 0.0f) {
            Ki = ki_in;
            sprintf(pt_serial_out_buffer, "Ki set to %f\n", (double)Ki);
        }
        else {
            sprintf(pt_serial_out_buffer, "Ki must be finite and >= 0\n");
        }
        serial_write ;

        //kd input
        sprintf(pt_serial_out_buffer, "input a Kd value: ");
        serial_write ;
        serial_read ;
        n = sscanf(pt_serial_in_buffer,"%f", &kd_in);
        if (n == EOF) {
            sprintf(pt_serial_out_buffer, "Kd kept at %f\n", (double)Kd);
        }
        else if (n == 1 && isfinite(kd_in) && kd_in >= 0.0f) {
            Kd = kd_in;
            sprintf(pt_serial_out_buffer, "Kd set to %f\n", (double)Kd);
        }
        else {
            sprintf(pt_serial_out_buffer, "Kd must be finite and >= 0\n");
        }
        serial_write ;
    }
    PT_END(pt) ;
}

// If the IMU wasn't found (or was plugged in late), keep trying once a second.
// Safe to touch I2C here: the ISR only uses I2C once imu_ok is set.
static PT_THREAD (protothread_imu_retry(struct pt *pt))
{
    PT_BEGIN(pt) ;
    while(1) {
        PT_YIELD_usec(1000000) ;
        if (!imu_ok) {
            i2c_bus_recover() ;
            imu_ok = imu_probe() ;
        }
    }
    PT_END(pt) ;
}

static PT_THREAD (protothread_button(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static bool seq_active = false ;
    static bool armed = false ;
    static uint32_t t0 ;
    static uint32_t t ;
    while(1) {
        PT_YIELD_usec(10000) ;
        if (!gpio_get(BUTTON_PIN)) {
            motor_hold = true ;
            armed = true ;
            seq_active = false ;
            target_angle_deg = 0.0f ;
        }
        else if (motor_hold && armed) {
            armed = false ;
            motor_hold = false ;
            seq_active = true ;
            t0 = time_us_32() ;
        }
        if (seq_active) {
            t = (time_us_32() - t0) / 1000000 ;
            if (t >= 15) { target_angle_deg = 90.0f ; seq_active = false ; }
            else if (t >= 10) target_angle_deg = 60.0f ;
            else if (t >= 5) target_angle_deg = 120.0f ;
            else target_angle_deg = 90.0f ;
        }
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_draw) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    printf("\nMPU6050 IMU demo booting\n") ;

    // Initialize VGA
    initVGA() ;

    // Start core 1 (VGA drawing) right away, so the display comes up
    // even if the IMU/I2C setup below has problems
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // ISR debug pin (toggled in on_pwm_wrap)
    gpio_init(14) ;
    gpio_set_dir(14, GPIO_OUT) ;
    gpio_put(14, 0) ;


    // control sequence GPIO 
    gpio_init(BUTTON_PIN) ;
    gpio_set_dir(BUTTON_PIN, GPIO_IN) ;
    gpio_pull_up(BUTTON_PIN) ;
    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    // Free a possibly stuck bus, init I2C (internal pull-ups stay on as a
    // fallback in case the breakout has none)
    i2c_bus_recover() ;

    // MPU6050 initialization
    imu_ok = imu_probe() ;
    if (imu_ok) {
        printf("MPU6050 found\n") ;
        mpu6050_read_raw(acceleration, gyro);
    }
    else {
        printf("MPU6050 NOT found on SDA=GPIO%d, SCL=GPIO%d -- will keep retrying\n",
               SDA_PIN, SCL_PIN) ;
    }


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

   
    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 0 (core 1 was started right after initVGA)
    pt_add_thread(protothread_serial) ;
    pt_add_thread(protothread_imu_retry) ;
    pt_add_thread(protothread_button) ;
    pt_schedule_start ;

}
