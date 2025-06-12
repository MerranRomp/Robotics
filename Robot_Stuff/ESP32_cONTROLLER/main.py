from machine import Pin, I2C, UART
from time import ticks_ms, ticks_diff, sleep, time, ticks_us
import VL53L0X
from Utils import SeeFunctions, ThinkFunctions, ActFunctions
import math



def update(pin):
    global tick_count, last_a
    a = pin_a.value()
    b = pin_b.value()
    if a != last_a:  # only on change
        direction = 1 if a != b else -1
        tick_count += direction
        last_a = a


        
pin_a = Pin(18, Pin.IN)
pin_b = Pin(19, Pin.IN)

scl = Pin(22)  # I2C SCL pin
sda = Pin(21)  # I2C SDA pin

print("test1")
#variables

tick_count = 0
last_a = 0
counter = 0
COUNTER_MAX = 5
COUNTER_STOP = 50
PPR = 11  # encoder pulses per motor revolution (check your encoder datasheet)
GEAR_RATIO = 30  # gearbox reduction ratio
WHEEL_DIAMETER_CM = 6.5
recognition_distance = 200 # Distance in mm to recognize an object

IR_sensor_pins = [32, 33, 34, 35, 25]  # Pins for IR sensors (adjust as needed)



# Set serial to UART1 using the same pins as UART0 to communicate via USB
#uart = UART(1, 115200, tx=1, rx=3)
print("test2")
#Setup Sensors
SeeFunctions.setup_ir_sensors(*IR_sensor_pins) # Pins for IR sensors (adjust as needed)
print("test3")
tof = SeeFunctions.setup_VL53L0X()
print("test4")
pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update)
print("test5")

#set varialbles and initial states
current_state = 'forward' # Initial state of the robot
object_detected = False # Flag to indicate if an object is detected
state_updated = True # Flag to indicate if the state has been updated



while True:
    
    ############################################
    #                  See                     #
    ############################################
    start_ticks = tick_count
    start_time = ticks_us()


    #distance_mm = SeeFunctions.filtered_distance()
    #if distance_mm < recognition_distance:  # If the distance is less than the recognition distance
    #    object_detected = True

    sensor_vals = SeeFunctions.read_raw_values()
    print("IR Sensor Values:", sensor_vals)
    sensor_vals = SeeFunctions.read_normalized_values()
    print("IR Norm Values:", sensor_vals)
    sensor_vals = SeeFunctions.read_binary_values()
    print("IR Binary Values:", sensor_vals)

    delta_ticks = tick_count - start_ticks
    delta_time = ticks_diff(ticks_us(), start_time) / 1_000_000  # seconds

    # Compute wheel speed
    motor_rps = delta_ticks / PPR / delta_time
    wheel_rps = motor_rps / GEAR_RATIO
    wheel_circ = math.pi * WHEEL_DIAMETER_CM
    speed_cm_s = wheel_rps * wheel_circ
    print (f"Speed: {speed_cm_s:.2f} cm/s")


    ############################################
    #                 Think                    #
    ############################################

    error = ThinkFunctions.compute_error(sensor_vals, method='binary')


    
    ############################################
    #                  Act                     #
    ############################################

    # Send the new state when updated
    if state_updated == True:
        print("test")
        print(counter)
        counter += 1    # increment counter
        sleep(0.5) #wait 0.02 seconds





    """
    # Implement the line-following state machine transitions
    if current_state == 'forward':
        counter = 0
        if line_right and not line_left:
            current_state = 'turn_right'
            state_updated = True
        elif line_left and not line_right:
            current_state = 'turn_left'
            state_updated = True
        elif line_left and line_right and line_center: # lost the line
            current_state = 'turn_left'
            state_updated = True
        elif line_left and line_center and not line_right:
            current_state = 'forward'
            state_updated = True
        elif button_right.value() == True:
            current_state = 'stop'
            state_updated = True
            
            
    if current_state == 'turn_right':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
        elif button_right.value() == True:
            current_state = 'stop'
            state_updated = True

    if current_state == 'turn_left':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
        elif button_right.value() == True:
            current_state = 'stop'
            state_updated = True
            
    if current_state == 'stop':
        led_board.value(1)
        if counter >= COUNTER_STOP:
            current_state = 'forward'
            state_update = True
            led_board.value(0)
"""           

