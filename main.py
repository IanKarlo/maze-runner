from motor.motors_driver import MotorDriver


incrementatorA = 0
incrementatorB = 0
inc = 0


def callback_function(chip, gpio, level, timestamp):
    global incrementatorA, incrementatorB

    global inc
    # if gpio == 20 or gpio == 21:
    #     incrementatorA += 1
    # if gpio == 5 or gpio == 6:
    #     incrementatorB += 1
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