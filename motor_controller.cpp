#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define GEAR_RATIO 90UL
#define PPR 12UL
#define COUNTS_PER_REV (GEAR_RATIO * PPR) 

// KP, KI, KD tuning values
#define SETPOINT_RPM    60.0f
#define KP              1.2f
#define KI              0.8f
#define KD              0.02f
#define CONTROL_DT_S    0.2f      
#define PWM_MAX         255.0f
#define PWM_MIN         0.0f
#define INTEGRAL_CLAMP  300.0f    

volatile long encoderCount = 0;
volatile uint32_t millisCount = 0;

// 1ms system tick
static void timer0_init(void) {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
    millisCount++;
}

static uint32_t get_millis(void) {
    uint32_t m;
    uint8_t sreg = SREG;
    cli();
    m = millisCount;
    SREG = sreg;
    return m;
}

// hardware PWM
static void pwm_init(void) {
    DDRB |= (1 << PB1);
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
}

static void set_pwm(uint8_t duty) {
    OCR1A = duty;
}

// IN1 and IN2 direction pins
static void motor_dir_init(void) {
    DDRB |= (1 << PB0);
    DDRD |= (1 << PD7);
}

static void motor_forward(void) {
    PORTB |= (1 << PB0);
    PORTD &= ~(1 << PD7);
}

// encoder: INT0 on D2(channel A), D4 read(channel B) 
static void encoder_init(void) {
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
    DDRD &= ~(1 << PD4);
    PORTD |= (1 << PD4);

    EICRA |= (1 << ISC01) | (1 << ISC00);
    EIMSK |= (1 << INT0);
}

ISR(INT0_vect) {
    if (PIND & (1 << PD4)) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

//UART
static void uart_init(void) {
    UBRR0H = 0;
    UBRR0L = 8;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putchar(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_print(const char *s) {
    while (*s) uart_putchar(*s++);
}

static void uart_print_float(float val, uint8_t decimals) {
    char buf[16];
    dtostrf(val, 0, decimals, buf);
    uart_print(buf);
}

int main(void) {
    timer0_init();
    pwm_init();
    motor_dir_init();
    encoder_init();
    uart_init();
    sei();

    motor_forward();
    set_pwm(0);   //PID Loop starts immediately

    float integral = 0.0f;
    float prevError = 0.0f;

    uint32_t lastControlTime = get_millis();
    long lastEncoderCount = 0;

    while (1) {
        uint32_t now = get_millis();
        if (now - lastControlTime >= (uint32_t)(CONTROL_DT_S * 1000)) {
            uint8_t sreg = SREG;
            cli();
            long countsNow = encoderCount;
            SREG = sreg;

            long deltaCounts = countsNow - lastEncoderCount;
            float rpm = (deltaCounts / (float)COUNTS_PER_REV) / (CONTROL_DT_S / 60.0f);

            // PID
            float error = SETPOINT_RPM - rpm;

            integral += error * CONTROL_DT_S;
            if (integral > INTEGRAL_CLAMP)  integral = INTEGRAL_CLAMP;
            if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP;

            float derivative = (error - prevError) / CONTROL_DT_S;

            float output = (KP * error) + (KI * integral) + (KD * derivative);
            if (output > PWM_MAX) output = PWM_MAX;
            if (output < PWM_MIN) output = PWM_MIN;

            set_pwm((uint8_t)output);
            prevError = error;

            uart_print("Setpoint: ");
            uart_print_float(SETPOINT_RPM, 1);
            uart_print("  RPM: ");
            uart_print_float(rpm, 2);
            uart_print("  PWM: ");
            uart_print_float(output, 0);
            uart_print("\r\n");

            lastEncoderCount = countsNow;
            lastControlTime = now;
        }
    }
    return 0;
}