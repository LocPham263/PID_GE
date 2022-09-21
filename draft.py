import matplotlib.pyplot as plt
from random import seed
from random import random
import numpy as np

time_list = []
temp_list = []
dc_list = []
error_list = []

"""
    Transfer funtion for the system
        input = duty cycle
        output = current temperature 
"""
def model(input, temp_prev, setpoint):
    # output = 3.73*input - 9.51*input*input/100.0 + 8.73*input*input*input/10000.0 + 12.1
    if input == 0:
        output = temp_prev - 0.9 * abs(setpoint-temp_prev)
    # elif setpoint < 40:
    #     if temp_prev > setpoint - 0.5:
    #         output = temp_prev + input*40
    #     else:
    #         output = temp_prev + input
    else:
        output = temp_prev + input
    return output 

"""
    PID funtion
        input = Kp, Kd, Ki, 
                error = setpoint - current_temp
                error_previous = setpoint - last_temp
                error_sun = error + error_previous
        output = duty cycle
"""
def PID(err, err_prev, err_sum, Kp, Ki, Kd):
    P = Kp * err 
    D = Kd * (err - err_prev)
    I = Ki * err_sum

    output = P + I + D

    if output > 100:
        output = 100
    if output < 0:
        output = 0

    return output

def exec(time=100, setpoint = 40.0, Kp = 0.25, Ki = 0.5, Kd = 0.08):
    err = 0.001
    err_prev = 0
    err_sum = 0
    temp_prev = 32

    for t in range (0, time, 1):
        DC = PID(err, err_prev, err_sum, Kp, Kd, Ki)
        temp = model(DC, temp_prev, setpoint)
        temp_prev = temp
        # Update parameters
        err_prev = err
        err = setpoint - temp
        err_sum = err_sum + err

        temp_list.append(temp)
        time_list.append(t)
        dc_list.append(DC)
        error_list.append(err)

    # print(time_list)
    # print(temp_list)
    # print(dc_list)
    # print(error_list)
    plt.plot(time_list, temp_list)
    plt.show()



if __name__ == "__main__":
    exec()
    