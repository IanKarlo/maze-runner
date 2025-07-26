import lgpio

# Global handle for the GPIO chip (usually /dev/gpiochip0)
CHIP = 0
handle = None
_pwm_pins = {}  # Track active PWM pins

def init(chip=0):
    """Initialize the GPIO chip and return True if successful."""
    global handle, CHIP
    CHIP = chip
    try:
        handle = lgpio.gpiochip_open(CHIP)
        return True
    except Exception as e:
        print(f"[GPIO INIT ERROR] {e}")
        return False

def pin_mode(pin, mode):
    """
    Set a GPIO pin as input or output.
    mode: 'in' or 'out'
    """
    if handle is None:
        raise RuntimeError("GPIO not initialized. Call init() first.")
    
    if mode == 'in':
        lgpio.gpio_claim_input(handle, pin)
    elif mode == 'out':
        lgpio.gpio_claim_output(handle, pin, 0)  # Start low
    else:
        raise ValueError("Mode must be 'in' or 'out'.")

def digital_write(pin, value):
    """Set pin HIGH (1) or LOW (0)."""
    if handle is None:
        raise RuntimeError("GPIO not initialized. Call init() first.")
    
    lgpio.gpio_write(handle, pin, 1 if value else 0)

def digital_read(pin):
    """Read and return digital value (0 or 1) from the pin."""
    if handle is None:
        raise RuntimeError("GPIO not initialized. Call init() first.")
    
    return lgpio.gpio_read(handle, pin)

def set_PWM(pin, duty_cycle_percent, frequency_hz=1000):
    """
    Start PWM on a pin.
    - duty_cycle_percent: 0.0 to 100.0
    - frequency_hz: PWM frequency (default 1000 Hz)
    """
    if handle is None:
        raise RuntimeError("GPIO not initialized. Call init() first.")

    lgpio.tx_pwm(handle, pin, frequency_hz, duty_cycle_percent)
    _pwm_pins[pin] = True

def stop_PWM(pin):
    """Stop PWM on a pin."""
    if handle is None:
        raise RuntimeError("GPIO not initialized. Call init() first.")
    if pin in _pwm_pins:
        set_PWM(pin, 0, 0)
        del _pwm_pins[pin]

def generic_cbf(chip, gpio, level, timestamp) :
    print(gpio, level, timestamp)


def setup_interruptions(encoder_pin, cbf = generic_cbf):
    lgpio.gpio_claim_alert(handle, encoder_pin, lgpio.RISING_EDGE)
    lgpio.callback(handle, encoder_pin, lgpio.RISING_EDGE, cbf)

def cleanup():
    """Release the GPIO handle and stop any PWM."""
    global handle
    for pin in list(_pwm_pins):
        stop_PWM(pin)
    if handle is not None:
        lgpio.gpiochip_close(handle)
        handle = None
