import lgpio
# Configuration
ENA = 12
IN1 = 23
IN2 = 24

ENC_MOT_A_1 = 5
ENC_MOT_A_2 = 6


FREQ = 1000
# Open the gpio chip
def cbf ( chip , gpio , level , timestamp ) :
    print (gpio , level , timestamp )

h = lgpio . gpiochip_open (0)

lgpio . gpio_claim_output (h , IN1)
lgpio . gpio_claim_output (h , IN2)
lgpio . gpio_claim_output (h , ENA)

lgpio . gpio_claim_alert (h , ENC_MOT_A_1 , lgpio . BOTH_EDGES )
cb1 = lgpio . callback (h , ENC_MOT_A_1 , lgpio . BOTH_EDGES , cbf )
lgpio . gpio_claim_alert (h , ENC_MOT_A_2 , lgpio . BOTH_EDGES )
cb1 = lgpio . callback (h , ENC_MOT_A_2 , lgpio . BOTH_EDGES , cbf )
# input("enter")

lgpio . gpio_write (h , IN1 , 0) 
lgpio . gpio_write (h , IN2 , 1) # MOTOR A - IN2 - ANTI-H - PRA TRAS

# Turn the PIN off

lgpio . tx_pwm (h , ENA , FREQ , 50)

input("enter")
# Turn the PIN to max voltage

# Close the gpio chip
lgpio . tx_pwm (h , ENA , 0 , 0)
lgpio . gpiochip_close ( h )

