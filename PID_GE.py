import matplotlib.pyplot as plt
import numpy as np

"""
    Transfer funtion for the system
        input = duty cycle
        output = current temperature 
"""
def model(input, temp_prev, setpoint):
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

def exec(time=840, setpoint = 40.0, Kp = 0.04, Kd = 0, Ki = 0):
    err = 0.001
    err_prev = 0
    err_sum = 0
    temp_prev = 25

    time_list = []
    temp_list = []
    dc_list = []
    error_list = []

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
        error_list.append(abs(err))

    return time_list, temp_list, error_list
    # print(time_list)
    # print(temp_list)
    # print(dc_list)
    # print(error_list)
    # plt.plot(time_list, temp_list)
    # plt.show()

"""
    Initial Phase
    input:
        size = size of the population
        p_range, d_range, i_range = maximum values of P,I,D
    output:
        init = 1st gen population (2D list (size rows x 3 columns)
"""
def initial(size, p_range, d_range, i_range):
    init = []
    # for i in range (size//10):

    temp_p = np.random.uniform(0.0005, p_range, size=((size,)))
    temp_i = np.random.uniform(0, i_range, size=((size,)))
    temp_d = np.random.uniform(0, d_range, size=((size,)))
    
    for i in range (size):
        temp = []
        temp.append(temp_p[i])
        temp.append(temp_i[i])
        temp.append(temp_d[i])
        init.append(temp)
        
    return init


"""
    Fitness calculation phase
    input:
        gen = the last generation of the population (2D list)
    output:
        sorted = the current generation with sorted order of error (2D ndarray size rows x 4 columns (P I D error))
"""
def fitness_order(gen):
    for i in range (len(gen)):
        __, __, error_list = exec(Kp=gen[i][0], Ki=gen[i][1], Kd=gen[i][2])
        gen[i].append(sum(error_list)/len(error_list))

    gen = np.array(gen)
    sorted = gen[np.argsort(gen[:,3])]

    return sorted


"""
    Reproduction phase
    input:
        sorted_gen = the current generation with sorted order of error (2D array)
    output:
        next_gen = the next generation of population (2D list)
"""
def reproduction(sorted_gen, partition=5):
    next_gen = []
    sorted_gen = sorted_gen.tolist()
    size = len(sorted_gen)
    sel_num = size//partition
    for i in range (size):
        next_gen.append(sorted_gen[i][:3])

    # Crossover phase
    for i in range (sel_num):
        for j in range (1,partition):
            next_gen[i + (sel_num)*j][0] = sorted_gen[i][0]

    return next_gen

def display_result(gen):
    for i in range (len(gen)):
        print(gen[i])

if __name__ == "__main__":
    # exec()
    init = initial(size=20, p_range=0.5, d_range=0.05, i_range=1)
    # display_result(init)
    # print('\n')

    # gen = fitness_order(init)
    # print(gen)
    # print('\n')
    # display_result(gen)

    init[5][2] = 0.00001
    init[4][2] = init[4][2]/1000
    init[3][2] = init[3][2]/500
    init[2][2] = init[2][2]/100
    init[1][2] = init[1][2]/50
    init[0][2] = init[0][2]/10

    init[5][0] = init[5][0]/80
    init[4][0] = init[4][0]/50
    init[3][0] = init[4][0]/30
    init[2][0] = init[4][0]/10
    init[1][0] = init[4][0]/2

    display_result(init)
    print('\n')

    for i in range (5):
        fitness = fitness_order(init)
        # display_result(fitness)
        init = reproduction(fitness)
        # print('\n')
        # display_result(init)

        # if i==0 or i==9:
        #     print(fitness)
        #     print('\n')
        print(fitness[:5])
        print('\n')
