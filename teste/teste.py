import gpio

if gpio.init():
    gpio.pinMode(13, 'out')

    gpio.setPWM(13, 50.0)  # Start PWM at 50% duty cycle, 500 Hz
    input("PWM running. Press Enter to change duty...")

    # gpio.pwmChangeDutyCycle(13, 80.0)
    # input("Duty changed to 80%. Press Enter to stop PWM...")

    name = input()

    gpio.stopPWM(13)
    gpio.cleanup()
