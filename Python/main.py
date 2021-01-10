import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import timeit
from scipy import signal
from directkeys import PressKey, ReleaseKey, E, SHIFT, L
#----------------------------------------------------------------------------------------------------------------------------------
# VARIABLES INIT
#----------------------------------------------------------------------------------------------------------------------------------
_tstart_stack = []

# sensor variables
sample_rate = 1000
accelerometer_scale = 16
gyroscope_scale = 1000

# Might need later
gyroscope_x_offset = 48
gyroscope_y_offset = -52
gyroscope_z_offset = -85

# data comms variables
baud_rate = 115200
buffer_length = 150
number_of_bytes = 7
button_state = 0

# accelerometer array
A = np.zeros((buffer_length,3))

# data array for past samples, fifth of data to be compared
memory_buffer = np.zeros((int(buffer_length/5),3))
memory_index = 0
sample_index = 0

# processing variables
from_ADC_g_to_metres = accelerometer_scale*9.81/32768
from_ADc_deg_to_deg = gyroscope_scale/32768
dt = 1/sample_rate
key_delay_in_s = 0.01 # delay between key press and key release
charge_pressed = 0

#----------------------------------------------------------------------------------------------------------------------------------
# LOADING TEMPLATES
#----------------------------------------------------------------------------------------------------------------------------------
template_number = 1
punch = np.loadtxt('Punch.txt')
charge = np.loadtxt('Charge.txt')
slam = np.loadtxt('Slam.txt')
uppercut = np.loadtxt('Uppercut.txt')

#----------------------------------------------------------------------------------------------------------------------------------
# FILTER DESIGN
#----------------------------------------------------------------------------------------------------------------------------------
highpass_cutoff = 0.1
lowpass_cutoff = 15

# Highpass filter for offsets like gravity, 0.1 Hz cutoff
DHPF = signal.butter(1, 2*highpass_cutoff/sample_rate, 'highpass', analog=False, output='ba')

# Lowpass filter for noise elimination, 10 Hz cutoff
DLPF = signal.butter(1, 2*lowpass_cutoff/sample_rate, 'lowpass', analog=False, output='ba')

# Bandpass filter
bandpass = signal.butter(1, [2*highpass_cutoff/sample_rate,2*lowpass_cutoff/sample_rate], 'bandpass', analog=False, output='ba')

#----------------------------------------------------------------------------------------------------------------------------------
# FUNCTIONS DEFINITION
#----------------------------------------------------------------------------------------------------------------------------------

def tic():
    _tstart_stack.append(timeit.default_timer())

def toc(fmt="Elapsed: %s s"):
    print(fmt % (timeit.default_timer() - _tstart_stack.pop()))

def start_com():
    # Open com port
    global ser
    ser = serial.Serial('COM6', baudrate=baud_rate, timeout=None,xonxoff=False)

    # Wait for serial com to ready up
    print("Opening serial comms port...")
    time.sleep(5)

    # clear serial com buffer just to be sure
    ser.reset_input_buffer()

    # ask for first data
    print("Asking for data...")
    ask_for_data()
    ser.reset_output_buffer()

    # read and discard first three samples
    print("Discarding first 3 samples...")
    ser.read(number_of_bytes*3)

    ser.reset_input_buffer()
    print("Comms and Sensors ready, it's SHOWTIME!")

def ask_for_data():
    # send a char R to arduino to let him know he is READY for data
    ser.write('R'.encode('utf-8'))

def read_data(i):
    global A, button_state, ser
    # read one sample of data (Acceleration x,y,z)
    # and read button_state byte if its zero
    temp_data = ser.read(number_of_bytes)
    A[i,0] = int.from_bytes(temp_data[0:2],'big', signed=True)
    A[i,1] = int.from_bytes(temp_data[2:4],'big', signed=True)
    A[i,2] = int.from_bytes(temp_data[4:6],'big', signed=True)
    if (button_state == 0):
        button_state = temp_data[6]

    # G[i,0] = int.from_bytes(temp_data[6:8],'big', signed=True) + gyroscope_x_offset
    # G[i,1] = int.from_bytes(temp_data[8:10],'big', signed=True) + gyroscope_y_offset
    # G[i,2] = int.from_bytes(temp_data[10:12],'big', signed=True) + gyroscope_z_offset

def process_sample(i):
    global A
    # transforming to proper units, (Acceleration -> m/s^2; Angular Velocity -> rad/s)
    A[i,:]= A[i,:] * from_ADC_g_to_metres

    # G[i,:] = np.deg2rad(G[i,:] * from_ADc_deg_to_deg)

    # if (abs(G[i,0]) <= 0.005):
    #     G[i,0] = 0

    # if (abs(G[i,1]) <= 0.005):
    #     G[i,1] = 0

    # if (abs(G[i,2]) <= 0.005):
    #     G[i,2] = 0

def save_to_memory_buffer():
    global memory_buffer, memory_index
    
    # keeping a memory of samples , length is always one fifth of the whole signal
    memory_buffer[memory_index,:] = A[0,:]
    memory_index += 1

    # if full, reset to start rewriting memory from oldest values
    if (memory_index >= len(memory_buffer)):
        memory_index = 0

def retrieve_memory():
    global A

    # retrieve the memory samples, np.roll shifts the vector so the samples
    # are aligned from oldest to newest
    A[0:len(memory_buffer),:] = np.roll(memory_buffer,-memory_index)

def process_sample_buffer():
    global A, sample_index, button_state

    # both low and high pass filtering acceleration to remove noise and gravity
    A[:,0] = signal.filtfilt(bandpass[0], bandpass[1], A[:,0]) # x acc filtered
    A[:,1] = signal.filtfilt(bandpass[0], bandpass[1], A[:,1]) # y acc filtered
    A[:,2] = signal.filtfilt(bandpass[0], bandpass[1], A[:,2]) # z acc filtered

    # # low pass filtering angular velocities to remove noise
    # G[:,0] = signal.filtfilt(DLPF[0],DLPF[1], G[:,0])
    # G[:,1] = signal.filtfilt(DLPF[0],DLPF[1], G[:,1])
    # G[:,2] = signal.filtfilt(DLPF[0],DLPF[1], G[:,2])

    # integrating for angle
    # angle_x = integrate.cumtrapz(G[:,0], dx=dt) + angle_x[-1]
    # angle_y = integrate.cumtrapz(G[:,1], dx=dt) + angle_y[-1]
    # angle_z = integrate.cumtrapz(G[:,2], dx=dt) + angle_z[-1]

    # G_full_x.extend(angle_x)
    # G_full_y.extend(angle_y)
    # G_full_z.extend(angle_z)

    

    # A_full_x.extend(A[:,0])
    # A_full_y.extend(A[:,1])
    # A_full_z.extend(A[:,2])

def compare():
    global sample_index, button_state, charge_pressed, ability
    # compare the input signal with pre-recorded gestures
    # subtract the signals from each other, abs(), then compare the error values
    # the one with MIN ERROR value is the wanted gesture
    charge_error = sum(abs(charge[:,0] - A[:,0])) + sum(abs(charge[:,1] - A[:,1])) + sum(abs(charge[:,2] - A[:,2]))
    punch_error = sum(abs(punch[:,0] - A[:,0])) + sum(abs(punch[:,1] - A[:,1])) + sum(abs(punch[:,2] - A[:,2]))
    slam_error = sum(abs(slam[:,0] - A[:,0])) + sum(abs(slam[:,1] - A[:,1])) + sum(abs(slam[:,2] - A[:,2]))
    uppercut_error = sum(abs(uppercut[:,0] - A[:,0])) + sum(abs(uppercut[:,1] - A[:,1])) + sum(abs(uppercut[:,2] - A[:,2]))

    ability = np.argmin([charge_error, punch_error, slam_error, uppercut_error])
    #print("Charge =", charge_error, "Punch =", punch_error, "Slam =", slam_error, "Uppercut =", uppercut_error)

def keyboard_input():
    global charge_pressed

    if ability == 0:
        #print("Charge it is")

        if(charge_pressed == 1):
            ReleaseKey(L)
            time.sleep(key_delay_in_s)
            charge_pressed = 0

        PressKey(L)
        time.sleep(key_delay_in_s)
        charge_pressed = 1 # indicator to reset the key later

    elif ability == 1:
        #print("Punch it is")
        ReleaseKey(L)
        time.sleep(key_delay_in_s)

    elif ability == 2:
        #print("Slam it is")

        if(charge_pressed == 1):
            ReleaseKey(L)
            time.sleep(key_delay_in_s)
            charge_pressed = 0

        PressKey(E)
        time.sleep(key_delay_in_s)
        ReleaseKey(E)
        time.sleep(key_delay_in_s)
        
    elif ability == 3:
        #print("Uppercut it is")

        if(charge_pressed == 1):
            ReleaseKey(L)
            time.sleep(key_delay_in_s)
            charge_pressed = 0

        PressKey(SHIFT)
        time.sleep(key_delay_in_s)
        ReleaseKey(SHIFT)
        time.sleep(key_delay_in_s)

def load_new_templates():
    global template_number, charge, punch, slam, uppercut, button_state, sample_index
    # creates new template signals for gestures

    if(template_number == 1):
        np.savetxt('Charge.txt',A)
        template_number += 1
        print("Charge template saved")
        charge = np.loadtxt('Charge.txt')
        print("Charge template loaded")
        sample_index = 0
        button_state = 0

    elif(template_number == 2):
        np.savetxt('Punch.txt',A)
        template_number += 1
        print("Punch template saved")
        punch = np.loadtxt('Punch.txt')
        print(("Punch template loaded"))
        sample_index = 0
        button_state = 0

    elif(template_number == 3):
        np.savetxt('Slam.txt',A)
        template_number += 1
        print("Slam template saved")
        slam = np.loadtxt('Slam.txt')
        print("Slam template loaded")
        sample_index = 0
        button_state = 0

    elif(template_number == 4):
        np.savetxt('Uppercut.txt',A)
        template_number = 5
        print("Uppercut template saved")
        uppercut = np.loadtxt('Uppercut.txt')
        print("Uppercut template loaded")
        sample_index = 0
        button_state = 0

def plotting():
    figure1, axs1 = plt.subplots(nrows=3, ncols=5, figsize=(12,8))
    cols = ['{}'.format(col) for col in ['Last Data','Charge','Punch','Slam','Uppercut']]
    rows = ['{}'.format(row) for row in ['X', 'Y', 'Z']]

    for ax, col in zip(axs1[0], cols):
        ax.set_title(col)

    for ax, row in zip(axs1[:,0], rows):
        ax.set_ylabel(row, rotation=0, size='large')

    figure1.tight_layout()

    # last data plot
    axs1[0,0].plot(np.linspace(0, dt*len(A[:,0]), len(A[:,0])), A[:,0])
    axs1[1,0].plot(np.linspace(0, dt*len(A[:,1]), len(A[:,1])), A[:,1])
    axs1[2,0].plot(np.linspace(0, dt*len(A[:,2]), len(A[:,2])), A[:,2])

    # charge plot
    axs1[0,1].plot(np.linspace(0, dt*len(charge[:,0]), len(charge[:,0])), charge[:,0])
    axs1[1,1].plot(np.linspace(0, dt*len(charge[:,1]), len(charge[:,1])), charge[:,1])
    axs1[2,1].plot(np.linspace(0, dt*len(charge[:,2]), len(charge[:,2])), charge[:,2])

    # punch plot
    axs1[0,2].plot(np.linspace(0, dt*len(punch[:,0]), len(punch[:,0])), punch[:,0])
    axs1[1,2].plot(np.linspace(0, dt*len(punch[:,1]), len(punch[:,1])), punch[:,1])
    axs1[2,2].plot(np.linspace(0, dt*len(punch[:,2]), len(punch[:,2])), punch[:,2])

    # slam plot
    axs1[0,3].plot(np.linspace(0, dt*len(slam[:,0]), len(slam[:,0])), slam[:,0])
    axs1[1,3].plot(np.linspace(0, dt*len(slam[:,1]), len(slam[:,1])), slam[:,1])
    axs1[2,3].plot(np.linspace(0, dt*len(slam[:,2]), len(slam[:,2])), slam[:,2])

    # uppercut plot
    axs1[0,4].plot(np.linspace(0, dt*len(uppercut[:,0]), len(uppercut[:,0])), uppercut[:,0])
    axs1[1,4].plot(np.linspace(0, dt*len(uppercut[:,1]), len(uppercut[:,1])), uppercut[:,1])
    axs1[2,4].plot(np.linspace(0, dt*len(uppercut[:,2]), len(uppercut[:,2])), uppercut[:,2])

#----------------------------------------------------------------------------------------------------------------------------------
# PROGRAM START
#----------------------------------------------------------------------------------------------------------------------------------

# choose a mode
mode = int(input("||| GAME TIME | NEW TEMPLATES | JUST GRAPHS ||| <--- ENTER ---> [ 0 | 1 | 2 ]: "))

#start serial comms
start_com()

while(1):
    # ask arduino for another sample
    ask_for_data()

    # read the sample bytes + button_state
    read_data(sample_index)

    # process a single sample --> correcting units
    process_sample(sample_index)

    # save into memory buffer
    save_to_memory_buffer()

    # if button pressed, retrieve memory
    if (button_state == 1):

        # grab the past samples
        retrieve_memory()

        sample_index = len(memory_buffer)

        while(sample_index < buffer_length):
            # read rest of samples of data, save it into A
            ask_for_data()

            # read the sample byte
            read_data(sample_index)
            
            # process each sample
            process_sample(sample_index)
            
            sample_index += 1

        # filter the whole buffer
        process_sample_buffer()

        if(mode == 1): # -------------- NEW TEMPLATES MODE----------------------------#
            if (template_number == 5): # break the loop after the last template is saved
                plotting()
                break

            load_new_templates()

        elif(mode == 2): # -------------- JUST GRAPHS MODE----------------------------#
            plotting() # plot the graphs
            break

        else: # -------------- GAME TIME MODE----------------------------#
            compare()   # compare the input signal with prerecorded gestures
            keyboard_input() # 

            # zero out the sample index
            sample_index = 0

            # zero out the button_state
            button_state = 0
            
ser.close()
plt.show()
