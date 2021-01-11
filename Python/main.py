import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import timeit
from scipy import signal, fft
from threading import Timer, Lock
from directkeys import PressKey, ReleaseKey, E, SHIFT, L, V
#----------------------------------------------------------------------------------------------------------------------------------
# VARIABLES INIT
#----------------------------------------------------------------------------------------------------------------------------------
_tstart_stack = []

# sensor variables
sample_rate = 1000
accelerometer_scale = 16
gyroscope_scale = 1000

# might need later
gyroscope_x_offset = 48
gyroscope_y_offset = -52
gyroscope_z_offset = -85

# data comms variables
baud_rate = 115200
buffer_length = 75
number_of_bytes = 7
button_state = 0

# accelerometer array
A = np.zeros((buffer_length,3))

# data array for past samples
memory_buffer = np.zeros((int(buffer_length/3),3))
memory_index = 0
sample_index = 0

# processing variables
from_ADC_g_to_metres = accelerometer_scale*9.81/32768
from_ADc_deg_to_deg = gyroscope_scale/32768
dt = 1/sample_rate

def release_charge_key_after_delay():

    global charge_pressed
    # timer function that gets called 3 sec after doing the charge ability
    if(charge_pressed == 1):
        ReleaseKey(L)
        print("CHARGE RELEASED")
        charge_pressed = 0

class RepeatableTimer(object):
    def __init__(self, interval, function, args=[], kwargs={}):
        self._interval = interval
        self._function = function
        self._args = args
        self._kwargs = kwargs
    def start(self):
        t = Timer(self._interval, self._function, *self._args, **self._kwargs)
        t.start()

# keyboard input variables
key_delay_in_s = 0.01 # delay between key press and key release
charge_pressed = 0 # indicator if the charge has been pressed or not
charge_timer = RepeatableTimer(3,release_charge_key_after_delay)

#----------------------------------------------------------------------------------------------------------------------------------
# LOADING TEMPLATES
#----------------------------------------------------------------------------------------------------------------------------------
template_data_number = 0
template_to_load = 0
choice = 0
template_array = np.zeros((buffer_length,3,10))
punch = np.loadtxt('Punch.txt')
charge = np.loadtxt('Charge.txt')
slam = np.loadtxt('Slam.txt')
uppercut = np.loadtxt('Uppercut.txt')
slap = np.loadtxt('Slap.txt')

#----------------------------------------------------------------------------------------------------------------------------------
# FILTER DESIGN
#----------------------------------------------------------------------------------------------------------------------------------
highpass_cutoff = 0.001
lowpass_cutoff = 15

# Highpass filter for offsets like gravity, 0.01 Hz cutoff
DHPF = signal.butter(1, 2*highpass_cutoff/sample_rate, 'highpass', analog=False, output='ba')

# Lowpass filter for noise elimination, 15 Hz cutoff
DLPF = signal.butter(1, 2*lowpass_cutoff/sample_rate, 'lowpass', analog=False, output='ba')

# Bandpass filter
bandpass = signal.butter(2, [2*highpass_cutoff/sample_rate,2*lowpass_cutoff/sample_rate], 'bandpass', analog=False, output='ba')

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
    print("Comms and Sensors ready")

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
  
    #low pass filtering acceleration to remove noise
    A[:,0] = signal.filtfilt(DLPF[0], DLPF[1], A[:,0]) # x acc filtered
    A[:,1] = signal.filtfilt(DLPF[0], DLPF[1], A[:,1]) # y acc filtered
    A[:,2] = signal.filtfilt(DLPF[0], DLPF[1], A[:,2]) # z acc filtered


    # OLD CODE PARTS, MIGHT USE IN THE FUTURE
    # _,ax = plt.subplots(3,3)

    # A_after_ifft = np.zeros((buffer_length,3))
    
    # ax[0,0].plot(np.linspace(0, dt*len(A[:,0]), len(A[:,0])), A[:,0])
    # ax[1,0].plot(np.linspace(0, dt*len(A[:,1]), len(A[:,1])), A[:,1])
    # ax[2,0].plot(np.linspace(0, dt*len(A[:,2]), len(A[:,2])), A[:,2])

    # A_after_ifft_x = fft.rfft(A[:,0])
    # A_after_ifft_y = fft.rfft(A[:,1])
    # A_after_ifft_z = fft.rfft(A[:,2])

    # A_after_ifft_x[0] = 0
    # A_after_ifft_y[0] = 0
    # A_after_ifft_z[0] = 0

    # A_after_ifft[:,0] = fft.irfft(A_after_ifft_x)
    # A_after_ifft[:,1] = fft.irfft(A_after_ifft_y)
    # A_after_ifft[:,2] = fft.irfft(A_after_ifft_z)
    
    # ax[0,1].plot(np.linspace(0, dt*len(A_after_ifft[:,0]), len(A_after_ifft[:,0])), A_after_ifft[:,0])
    # ax[1,1].plot(np.linspace(0, dt*len(A_after_ifft[:,1]), len(A_after_ifft[:,1])), A_after_ifft[:,1])
    # ax[2,1].plot(np.linspace(0, dt*len(A_after_ifft[:,2]), len(A_after_ifft[:,2])), A_after_ifft[:,2])

    # Bx = signal.filtfilt(DLPF[0], DLPF[1], A[:,0])le
    # By = signal.filtfilt(DLPF[0], DLPF[1], A[:,1])
    # Bz = signal.filtfilt(DLPF[0], DLPF[1], A[:,2])

    # ax[0,2].plot(np.linspace(0, dt*len(Bx), len(Bx)), Bx)evv
    # ax[1,2].plot(np.linspace(0, dt*len(By), len(By)), By)
    # ax[2,2].plot(np.linspace(0, dt*len(Bz), len(Bz)), Bz)

    # plt.show()

    # # low pass filtering angular velocities to remove noise
    # G[:,0] = signal.filtfilt(DLPF[0],DLPF[1], G[:,0])
    # G[:,1] = signal.filtfilt(DLPF[0],DLPF[1], G[:,1])
    # G[:,2] = signal.filtfilt(DLPF[0],DLPF[1], G[:,2])

    # integrating for angle
    # angle_x = integrate.cumtrapz(G[:,0], dx=dt) + angle_x[-1]
    # angle_y = integrate.cumtrapz(G[:,1], dx=dt) + angle_y[-1]
    # angle_z = integrate.cumtrapz(G[:,2], dx=dt) + angle_z[-1]

def compare():
    global sample_index, button_state, charge_pressed, ability
    # compare the input signal with pre-recorded gestures
    # subtract the signals from each other, abs(), then compare the error values
    # the one with MIN ERROR value is the wanted gesture
    charge_error = sum(abs(charge[:,0] - A[:,0])) + sum(abs(charge[:,1] - A[:,1])) + sum(abs(charge[:,2] - A[:,2]))
    punch_error = sum(abs(punch[:,0] - A[:,0])) + sum(abs(punch[:,1] - A[:,1])) + sum(abs(punch[:,2] - A[:,2]))
    slam_error = sum(abs(slam[:,0] - A[:,0])) + sum(abs(slam[:,1] - A[:,1])) + sum(abs(slam[:,2] - A[:,2]))
    uppercut_error = sum(abs(uppercut[:,0] - A[:,0])) + sum(abs(uppercut[:,1] - A[:,1])) + sum(abs(uppercut[:,2] - A[:,2]))
    slap_error = sum(abs(slap[:,0] - A[:,0])) + sum(abs(slap[:,1] - A[:,1])) + sum(abs(slap[:,2] - A[:,2]))

    ability_index = np.argmin([charge_error, punch_error, slam_error, uppercut_error, slap_error])

    if ability_index == 0:
        ability = "CHARGE"
    elif ability_index == 1:
        ability = "PUNCH"
    elif ability_index == 2:
        ability = "SLAM"
    elif ability_index == 3:
        ability = "UPPERCUT"
    elif ability_index == 4:
        ability = "SLAP"
    #print("Charge =", charge_error, "Punch =", punch_error, "Slam =", slam_error, "Uppercut =", uppercut_error)

def keyboard_input():
    global charge_pressed

    if ability == "CHARGE":
        if(charge_pressed == 1):
            charge_pressed = 0
            ReleaseKey(L)

        PressKey(L)
        charge_pressed = 1 # indicator to release the key later using a timer if not released
        charge_timer.start()
        time.sleep(key_delay_in_s)

    elif ability == "PUNCH":
        charge_pressed = 0
        ReleaseKey(L)

    elif ability == "SLAM":

        if(charge_pressed == 1):
            ReleaseKey(L)
            charge_pressed = 0

        PressKey(E)
        time.sleep(key_delay_in_s)
        ReleaseKey(E)

    elif ability == "UPPERCUT":

        if(charge_pressed == 1):
            ReleaseKey(L)
            charge_pressed = 0

        PressKey(SHIFT)
        time.sleep(key_delay_in_s)
        ReleaseKey(SHIFT)

    elif ability == "SLAP":

        if(charge_pressed == 1):
            ReleaseKey(L)
            charge_pressed = 0

        PressKey(V)
        time.sleep(key_delay_in_s)
        ReleaseKey(V)

    print(ability)

def load_new_templates():
    global template_data_number, charge, punch, slam, uppercut, button_state, sample_index, template_to_load, choice
    # loads 10 data sets for one GESTURE, calculates an average and saves that average as the new template for the GESTURE
    # this is done for all GESTURES

    if(template_to_load == "CHARGE"): # CHARGE CALIBRATION ROUTINE
        print(template_to_load, template_data_number+1)
        template_array[:,:,template_data_number] = A
        template_data_number += 1

        if(template_data_number >= 10):
            template_data_number = 0
            choice = 1
            averaged_template = np.sum(template_array,axis=2)/10
            np.savetxt('Charge.txt',averaged_template)
            print(template_to_load,"calibration done, sum of standard deviations for X Y Z is:", np.sum(np.std(template_array,axis=2),axis=0))
            time.sleep(0.5)

    elif(template_to_load == "PUNCH"):  # PUNCH CALIBRATION ROUTINE
        print(template_to_load, template_data_number+1)
        template_array[:,:,template_data_number] = A
        template_data_number += 1

        if(template_data_number >= 10):
            template_data_number = 0
            choice = 1
            averaged_template = np.sum(template_array,axis=2)/10
            np.savetxt('Punch.txt',averaged_template)
            print(template_to_load,"calibration done, sum of standard deviations for X Y Z is:", np.sum(np.std(template_array,axis=2),axis=0))
            time.sleep(0.5)


    elif(template_to_load == "SLAM"): # SLAM CALIBRATION ROUTINE
        print(template_to_load, template_data_number+1)
        template_array[:,:,template_data_number] = A
        template_data_number += 1

        if(template_data_number >= 10):
            template_data_number = 0
            choice = 1
            averaged_template = np.sum(template_array,axis=2)/10
            np.savetxt('Slam.txt',averaged_template)
            print(template_to_load,"calibration done, sum of standard deviations for X Y Z is:", np.sum(np.std(template_array,axis=2),axis=0))
            time.sleep(0.5)

    elif(template_to_load == "UPPERCUT"): # UPPERCUT CALIBRATION ROUTINE
        print(template_to_load, template_data_number+1)
        template_array[:,:,template_data_number] = A
        template_data_number += 1

        if(template_data_number >= 10):
            template_data_number = 0
            choice = 1
            averaged_template = np.sum(template_array,axis=2)/10
            np.savetxt('Uppercut.txt',averaged_template)
            print(template_to_load,"calibration done, sum of standard deviations for X Y Z is:", np.sum(np.std(template_array,axis=2),axis=0))
            time.sleep(0.5)

    elif(template_to_load == "SLAP"): # SLAM CALIBRATION ROUTINE
        print(template_to_load, template_data_number+1)
        template_array[:,:,template_data_number] = A
        template_data_number += 1

        if(template_data_number >= 10):
            template_data_number = 0
            choice = 1
            averaged_template = np.sum(template_array,axis=2)/10
            np.savetxt('Slap.txt',averaged_template)
            print(template_to_load,"calibration done, sum of standard deviations for X Y Z is:", np.sum(np.std(template_array,axis=2),axis=0))
            time.sleep(0.5)

def plotting():
    
    _, axs1 = plt.subplots(nrows=3, ncols=5, figsize=(12,8))
    cols = ['{}'.format(col) for col in ['Last move','Charge','Punch','Slam','Uppercut']]
    rows = ['{}'.format(row) for row in ['X', 'Y', 'Z']]

    for ax, col in zip(axs1[0], cols):
        ax.set_title(col)

    for ax, row in zip(axs1[:,0], rows):
        ax.set_ylabel(row, rotation=0, size='large')

    #figure1.tight_layout()

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

# CHOOSE A MODE
mode = int(input("||| GAME TIME | TEMPLATE CALIBRATION | JUST GRAPHS ||| <--- ENTER ---> [ 0 | 1 | 2 ]: "))

#start serial comms
while(1):
    try:
        start_com()
    except:
        print("Device not connected, please connect the device")
        time.sleep(5)
    else:
        break

if(mode == 0):
    print("IT'S PUNCH-TIME!")

elif(mode == 1):
    choice = 1
    print("Press a button to start the template calibration routine")
    
elif(mode == 2):
    print("Do any gesture to show graphs")

while(1): # ------- MAIN LOOP ------- #

    # ask arduino for another sample
    ask_for_data()

    # read the sample bytes + button_state
    read_data(sample_index)

    # process a single sample --> correcting units
    process_sample(sample_index)

    # save into memory buffer
    save_to_memory_buffer()

    if (button_state == 1): # ------- MEMORY RETRIEVAL AFTER BUTTON PRESS ------- #

        # grab the past samples
        retrieve_memory()

        sample_index = len(memory_buffer)

        while(sample_index < buffer_length): # ------- LOOP FOR READING THE LIVE GESTURE SAMPLES ------- #

            # ask arduino for another sample
            ask_for_data()

            # read samples of data, save it into A[] matrix
            read_data(sample_index)
            
            # process each sample to speed things up
            process_sample(sample_index)
            
            sample_index += 1

        # filter the whole buffer using butterworth bandpass filter
        process_sample_buffer()

        # -------------- MODE BEHAVIOUR BELOW---------------------------- #
        
        # -------------- NEW TEMPLATES MODE---------------------------- #
        if(mode == 1):
            if(choice == 1):
                template_to_load = input("Choose which template do you want to calibrate:| CHARGE | PUNCH | SLAM | UPPERCUT | SLAP | or EXIT: ")
                choice = 0
                if(template_to_load != "EXIT"):
                    print("Do 10 times", template_to_load)
                else:
                    print("Shutting down...")
            else:
                load_new_templates()

            if (template_to_load == "EXIT"):
                punch = np.loadtxt('Punch.txt')
                charge = np.loadtxt('Charge.txt')
                slam = np.loadtxt('Slam.txt')
                uppercut = np.loadtxt('Uppercut.txt')
                slap = np.loadtxt('Slap.txt')
                plotting()
                break


         # -------------- JUST GRAPHS MODE---------------------------- #
        elif(mode == 2):
            plotting() # plot the graphs
            break

         # -------------- GAME TIME MODE---------------------------- #
        else:
            compare()   # compare the input signal with prerecorded gestures
            keyboard_input() # 

        # zero out the sample index
        sample_index = 0

        # zero out the button_state
        button_state = 0
            
ser.close()
time.sleep(2)
plt.show()
