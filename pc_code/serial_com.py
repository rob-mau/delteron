import serial
import os

#-------------------------------------------------------------------------
#   DEFINIZIONE FUNZIONI
#-------------------------------------------------------------------------
def count_instr():
    
    file = open(file_name)
    count = 0
    while 1:
        line = file.readline()
        if not line:
            break
        if line[0] == 'G' and line[1] == '0':
            count += 1
    file.close
    return count

#-------------------------------------------------------------------------
def print_initial_info():

    for i in range(3):
        x = arduino.readline()
        print(x.decode())
    print('\n')
    
#-------------------------------------------------------------------------    
def print_state():

    if total_instr != 0:
        percentage = 100 * float(index_instr) / float(total_instr)  
    
    print('State of Progress: %3.1f %%\n' % percentage)
    print('Instruction: %d / %d\n' % (index_instr, total_instr))
    for i in range(3):
        x = arduino.readline()
        print(x.decode())
    print('\n')
 
#-------------------------------------------------------------------------
#    MAIN
#-------------------------------------------------------------------------  
baud_rate = 9600
port = 'COM6'
file_name = 'origin.ngc'

total_instr = count_instr()
index_instr = 1
percentage  = 0
flag  = 1

arduino = serial.Serial(port, 9600)
file = open(file_name)

print_initial_info()
while (index_instr <= total_instr):
    if flag == 1:
        line = file.readline()
        if line[0] == 'G' and line[1] == '0':
            arduino.write(line.encode())
            print_state()
            
            index_instr += 1
            flag = 0
    else:       
        msg = arduino.readline()
        if msg == b'Positioned\r\n':
            flag = 1

print("Completed!")            
file.close
