from motor.motors_driver import MotorDriver


incrementatorA = 0
incrementatorB = 0
inc = 0


def callback_function(chip, gpio, level, timestamp):
    global inc
    inc += 1
    print(inc)

def stop_fn1(max_steps = 600):
    global incrementatorA
    print(incrementatorA)
    return incrementatorA >= max_steps

def stop_fn2(max_steps = 600):
    global incrementatorB
    return incrementatorB >= max_steps

driver = MotorDriver(callback_function, stop_fn1, stop_fn2)

# driver.run_motors('forward', 80, 'forward', 80)

input()

# print(incrementator)