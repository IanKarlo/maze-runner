import gpio

ENA = 12
IN1 = 27
IN2 = 22

ENB = 12
IN3 = 27
IN4 = 22

if gpio.init():
    gpio.pinMode(ENA, 'out')
    gpio.pinMode(IN1, 'out')
    gpio.pinMode(IN2, 'out')

    gpio.digitalWrite(IN1, 1)
    gpio.digitalWrite(IN2, 0)

    while True:
        user_input = input("Enter new duty cycle (0-100) as an integer or 'exit' to quit: ")

        if user_input.lower() == 'exit':
            break

        try:
            duty_cycle = int(user_input)
            if 0 <= duty_cycle <= 100:
                print('duty_cycle:', duty_cycle)
                gpio.setPWM(ENA, duty_cycle)
            else:
                print("Duty cycle must be an integer between 0 and 100.")
        except ValueError:
            print("Invalid input. Please enter an integer or 'exit'.")

    gpio.stopPWM(ENA)
    gpio.cleanup()
