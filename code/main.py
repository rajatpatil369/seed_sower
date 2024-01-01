from micropython import const
from machine import Pin, Timer, ADC, PWM
from time import ticks_ms, ticks_diff, sleep_ms
from esp32_gpio_lcd import GpioLcd

HALL_SENSOR_PIN = const(5)
SWITCH_PIN = const(23)
POT_PIN = const(4)
LED_PIN = const(2)
LCD_RS_PIN = const(32)
LCD_EN_PIN = const(33)
LCD_D4_PIN = const(25)
LCD_D5_PIN = const(26)
LCD_D6_PIN = const(27)
LCD_D7_PIN = const(14)
MOTOR_PIN = const(19)

TRACTOR_WHEEL_PERIMETER = const(1.6)  # meter, diameter: honda livo
MPS_TO_KMPH = const(3.6)  # ((1000m / (69sec*60min)) ** -1
HALL_DEBOUNCE_PERIOD = const(49)  # vehicle speed should be strictly less than: ?? (bottleneck?)
SW_DEBOUNCE_PERIOD = const(225)
DELAY = const(200)
RPM = const(5.4)

timer = Timer(0)
speed = 0
prev_speed = 0
start_time = -1
flag = True
i = 0

switch = Pin(SWITCH_PIN, mode=Pin.IN, pull=Pin.PULL_UP)
pot = ADC(POT_PIN)

led = Pin(LED_PIN, mode=Pin.OUT)
lcd = GpioLcd(
    rs_pin=Pin(LCD_RS_PIN),
    enable_pin=Pin(LCD_EN_PIN),
    d4_pin=Pin(LCD_D4_PIN),
    d5_pin=Pin(LCD_D5_PIN),
    d6_pin=Pin(LCD_D6_PIN),
    d7_pin=Pin(LCD_D7_PIN),
)
motor = PWM(Pin(MOTOR_PIN), freq=5000, duty_u16=32768)


def record_speed(timer_instance_):
    global flag, start_time, speed, d_per  #, i
    if flag:  # or True:
        time_delta = ticks_diff(ticks_ms(), start_time) / 1000  # sec
        temp = speed
        speed = MPS_TO_KMPH * (TRACTOR_WHEEL_PERIMETER / time_delta)  # kmph
        start_time = ticks_ms()
        flag = False
    else:
        # *hall sensor gives a rising edge when 1) magnet comes close and 2) it moves away
        flag = True


def unknown_hall(pin_instance_):  # source of *this behaviour is unknown
    timer.init(mode=Timer.ONE_SHOT, period=HALL_DEBOUNCE_PERIOD, callback=record_speed)


hall_sensor = Pin(HALL_SENSOR_PIN, mode=Pin.IN)
_ = hall_sensor.irq(handler=unknown_hall, trigger=Pin.IRQ_RISING)


def pulse_led(ms):
    led(1)
    sleep_ms(ms)
    led(0)


def value_map(value, fromLow, fromHigh, toLow, toHigh):
    return int((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow)


def is_switch_pressed():
    if switch() == 0:
        pulse_led(16)
        sleep_ms(SW_DEBOUNCE_PERIOD)
        # pwm = PWM(Pin(LED_PIN), duty=512, freq=0)
        while switch() == 1:
            temp = pot.read()
            d = value_map(temp, 0, 4095, 1, 36)
            lcd.move_to(0, 1)
            lcd.putstr(f'{d:02}" {d/12:.2f}')
            # x = 1 / value_map(temp, 0, 4095, 0.2, 2)
            # pwm.freq(x)
            sleep_ms(DELAY)
        # pwm.deinit()
        # global led
        # led = Pin(LED_PIN, mode=Pin.OUT)
        pulse_led(16)
        sleep_ms(SW_DEBOUNCE_PERIOD)


lcd.clear()
lcd.putstr('Distance   Speed\n???')
motor.duty(0)
while True:
    is_switch_pressed()
    lcd.move_to(11, 1)
    lcd.putstr(f'{speed: 2.2f}')
    rpm = speed * RPM
    duty_cycle = value_map(rpm, 0, 500, 0, 1023)
    motor.duty(duty_cycle)
    if speed <= prev_speed:
        i += 1
    else:  # >
        i -= 1
    if i * DELAY >= 5000:
        speed *= 0.8
        i = 0
    prev_speed = speed
    sleep_ms(DELAY)
