# encoder_handler.py

from machine import Pin

# Encoder tick counts and last state
tick_count_1 = 0
last_a_1 = 0
tick_count_2 = 0
last_a_2 = 0

# Encoder ISR for channel 1
def update_encoder1(pin):
    global tick_count_1, last_a_1, pin_a1, pin_b1
    a = pin_a1.value()
    b = pin_b1.value()
    if a != last_a_1:
        direction = 1 if a != b else -1
        tick_count_1 += direction
        last_a_1 = a

# Encoder ISR for channel 2
def update_encoder2(pin):
    global tick_count_2, last_a_2, pin_a2, pin_b2
    a = pin_a2.value()
    b = pin_b2.value()
    if a != last_a_2:
        direction = 1 if a != b else -1
        tick_count_2 += direction
        last_a_2 = a

# Setup function to attach pins and interrupts
def setup_encoders(pin_map):
    global pin_a1, pin_b1, pin_a2, pin_b2
    pin_a1 = Pin(pin_map['a1'], Pin.IN)
    pin_b1 = Pin(pin_map['b1'], Pin.IN)
    pin_a2 = Pin(pin_map['a2'], Pin.IN)
    pin_b2 = Pin(pin_map['b2'], Pin.IN)

    pin_a1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder1)
    pin_a2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder2)

# Getter and reset functions
def read_and_reset_ticks():
    global tick_count_1, tick_count_2
    ticks_l = tick_count_1
    ticks_r = tick_count_2
    tick_count_1 = 0
    tick_count_2 = 0
    return ticks_l, ticks_r
