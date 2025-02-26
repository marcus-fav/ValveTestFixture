from math import fabs
import serial
import time
#import FlowSensorData as fs
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as tic
from sensirion_slf3s import slf3s,find_sensor_port
from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection, ShdlcDevice
import time
#from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection, ShdlcDevice
from sensirion_shdlc_driver.command import ShdlcCommand
from struct import unpack
import sys
import traceback
import configparser

#AllMotionBoard setup
#-------------------------------
#Valve 1 -> Address 1, junc 1
#Valve 2 -> Address 1, junc 2
#        -> Address 1, Stepper
#-------------------------------
#Pump    -> Address 2, junc 1
#        -> Address 2, junc 2
#Pres Reg-> Address 2, Stepper
#-------------------------------
#Vlv Pres-> Address 3, junc 1
#Vlv Vac -> Address 3, junc 2
#Vac  Reg-> Address 3, Stepper
#-------------------------------
valveTestAdr = 4
valve1TestJun = 1
valve2TestJun = 2
pumpAdr= 1
valveFlowAdr = 2

if "Parse sys.argv" != False:
    fileIdx = 0
    commandIdx = 1
    durationIdx = 2
    iterationIdx =3
    valveIdx = 4
    argCommands = sys.argv
    try: 
        if argCommands[fileIdx] != None:
            argCommands[fileIdx] == "None"
    except Exception as e:
        traceback.print_exc()
        print(e)
    try: 
        if argCommands[commandIdx] != None:
            argCommands[commandIdx] = str(argCommands[1])
    except Exception as e:
        traceback.print_exc()
        print(e)
    try: 
        if argCommands[durationIdx] != None:
            argCommands[durationIdx] = int(argCommands[2])
    except Exception as e:
        argCommands.append(5)
        traceback.print_exc()
        print(e)
    try: 
        if argCommands[iterationIdx] != None:
            argCommands[iterationIdx] = int(argCommands[iterationIdx])
    except Exception as e:
        argCommands.append(1)
        traceback.print_exc()
        print(e)
    try: 
        if argCommands[valveIdx] != None:
            argCommands[valveIdx] = str(argCommands[valveIdx])
    except Exception as e:
        argCommands.append('')
        traceback.print_exc()
        print(e)
    #print(argCommands)

#TEST LIST
if "define index for Test List" != False: 
    openFlowIdx = 0 
    closedFlowIdx = 1
    valveCycleIdx = 2
    openFlowRepeatIdx = 3
    closedFlowRepeatIdx = 4
    highPressureIdx = 5
    valveRampUpIdx = 6
    valveRampDownIdx = 7
    valveRampUpRepeatIdx = 8
    valveRampDownRepeatIdx = 9
    testResultArray = ['N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A']
    testValueArray = [0,0,0,0,0,0,0,0,0,0]

if "check command" != False:
    commandList = ['openflow',
                   'closedflow',
                   'valvecycle',
                   'openrepeat',
                   'closedrepeat',
                   'highpressure',
                   'rampup',
                   'rampdown',
                   'rampuprepeat',
                   'rampdownrepeat',
                   'initialize',
                   'all']
    commandBoolean = False
    for i in commandList:
        if argCommands[commandIdx] == i:
            commandBoolean = True
    if commandBoolean == False:
            print('Command not recognized.')
            print('Command List:')
            print(commandList)
            quit()

if 'load config.ini values' != False:
    config = configparser.ConfigParser()
    config.sections()
    config.read('config.ini')
    allmotionport = config.get('Port','allmotion')
    flowsensorport1 = config.get('Port','flowsensor1')
    flowsensorport2 = config.get('Port','flowsensor2')

    #LOAD SPEC VALUES
    specOF_meanFlowMinimum =  int(config.get('OpenFlowTest','MeanFlowMinimum'))
    specCF_meanFlowMaximum = int(config.get('ClosedFlowTest','MeanFlowMaximum'))
    specVC_cycleIteration = int(config.get('ValveCycleTest','CycleIteration'))
    specVC_meanFlowPercent = float(config.get('ValveCycleTest','MeanFlowPercent'))
    specOFR_meanFlowMinimum =  int(config.get('OpenFlowRepeatabilityTest','MeanFlowMinimum'))
    specOFR_meanFlowSlope=  float(config.get('OpenFlowRepeatabilityTest','MeanFlowSlope'))
    specCFR_meanFlowMinimum =  int(config.get('ClosedFlowRepeatabilityTest','MeanFlowMaximum'))
    specCFR_meanFlowSlope=  float(config.get('ClosedFlowRepeatabilityTest','MeanFlowSlope'))
    specHPCF_meanFlowMaximum = int(config.get('HighPressureClosedFlowTest', 'MeanFlowMaximum'))
    specVRUT_meanFlowMinimum =  int(config.get('ValveRampUpTimeTest','MeanFlowMinimum'))
    specVRUT_duration =  int(config.get('ValveRampUpTimeTest','Duration'))
    specVRDT_meanFlowMaximum =  int(config.get('ValveRampDownTimeTest','MeanFlowMaximum'))
    specVRDT_duration =  int(config.get('ValveRampDownTimeTest','Duration'))
    specVRUTR_runIteration =  int(config.get('ValveRampUpTimeRepeatabilityTest','RunIteration'))
    specVRUTR_rampUpSlope =  float(config.get('ValveRampUpTimeRepeatabilityTest','RampUpSlope'))
    specVRDTR_runIteration =  int(config.get('ValveRampDownTimeRepeatabilityTest','RunIteration'))
    specVRDTR_rampDownSlope =  float(config.get('ValveRampDownTimeRepeatabilityTest','RampDownSlope'))

if 'define regulator values' != False:
    #set regulator setpoints. consider calibration function to save values to config
    presRegMin = 0
    presRegMax = 2**19
    presRegAdr = 1
    presRegSP1 = 100500
    presRegSP2 = 145550
    presRegSP3 = 197850
    presRegSP4 = 220000
    presRegSP5 = 400000
    presRegCur = 7

    vacRegMin = 0
    vacRegMax = 2**20
    vacRegAdr = 2
    vacRegSP1 = 80000
    vacRegSP2 = 115750
    vacRegSP3 = 215000
    vacRegSP4 = 500000
    vacRegCur = 4

#List Connected Serial ports
def serial_ports():

    ports = ['COM%s' % (i + 1) for i in range(256)]

    result = []

    for port in ports:

        try:

            s = serial.Serial(port)

            s.close()

            result.append(port)

        except (OSError, serial.SerialException):

            pass

    return result

#For controlling all motion boards
def sendCommand(command):


        #command = input("Enter command: ")

        if command.strip().lower() == 'exit':

          return # Exit the loop if the user enters 'exit'

        

        # Add a prefix 'b' to encode the command as bytes

        ser.write(command.strip().encode() + b'\r\n')

        

        # Set a timeout for readline

        ser.timeout = 1.0  # 1 second timeout if no response

        

        # Reads the response on the serial port

        response = ser.readline().strip().decode('unicode-escape')

        

        # Reset timeout to None after readline

        ser.timeout = None

        

        # Remove specific characters from the response to clean up output

        response = response.translate({ord('y'): None, ord('\x03'): None})

        

        # Remove leading '/0' to clean up output

        if response.startswith('/0'):

            response = response[2:]

        

        # Check if response is "@" (move response) or "'" (terminate response) and print "Sent" instead

        if response in ["@", "`"]:

            print("Sent")
            
        elif response:

            # Remove specified prefixes at the beginning of the response to clean up output

            prefixes_to_remove = ["`", "@"]  # These appear at the beginning of some strings

            for prefix_to_remove in prefixes_to_remove:

                if response.startswith(prefix_to_remove):

                    response = response[len(prefix_to_remove):]

                    return 

        

            #print("Received response:", response)

        else:

            print("Error. No Response. Please check connections and settings or try another command.")

#connect to all motion boards
if 'connect to all motion boards' != False:

    print(serial_ports())

    ser = serial.Serial(

        port=allmotionport,

        baudrate=9600,

        parity=serial.PARITY_NONE,

        stopbits=serial.STOPBITS_ONE,

        bytesize=serial.EIGHTBITS

    )
    #FlowSensorData.initData()

#Connect to flow sensors
if 'Connect to flow sensors' != False: 
    port1 = ShdlcSerialPort(port = flowsensorport1, baudrate = 115200)
    port2 = ShdlcSerialPort(port = flowsensorport2, baudrate = 115200)
    device1 = ShdlcDevice(ShdlcConnection(port1), slave_address=0)
    device2 = ShdlcDevice(ShdlcConnection(port2), slave_address=0)
    # Device Reset (will reset devices if stuck on continuous measurement)
    raw_response = device1.execute(ShdlcCommand(
            id=0xD3,  # The command ID as specified in the device documentation
            data=b"",  # The payload data to send
            max_response_time=0.2,  # Maximum response time in Seconds
        ))
    print("Raw Response: {}".format(raw_response))
    raw_response = device2.execute(ShdlcCommand(
            id=0xD3,  # The command ID as specified in the device documentation
            data=b"",  # The payload data to send
            max_response_time=0.2,  # Maximum response time in Seconds
        ))
    print("Raw Response: {}".format(raw_response))
    #Create slf3s class
    device1 = slf3s(ShdlcConnection(port1), slave_address = 0)
    device2 = slf3s(ShdlcConnection(port2), slave_address = 0)

#Column labels
if 'define index for data columns' != False:
    dataTime = 0
    dataFlow1 = 1
    dataTemp1 = 2
    dataFlag1 = 3
    dataFlow2 = 4
    dataTemp2 = 5
    dataFlag2 = 6
    dataLength = 7
    statDat = 0
    statSum = 1
    statAvg = 2
    statDev = 3
    statVar = 4
    sensorData = np.zeros((1,dataLength))
    emptyrow = np.zeros((1,dataLength))

def shutdown():
    device1.stop()
    device2.stop()
    setPres(presRegMin,presRegAdr)
    setVac(vacRegMin,vacRegAdr)
    sendCommand(f'/{valveFlowAdr}J3R<CR>') #close flow
    sendCommand(f'/{pumpAdr}J0R<CR>') #close pump

def homePres(regAddress):
    #/4m5h1D1000000R
    print('Homing pressure regulator', end='...... ')
    sendCommand(f'/{regAddress}m10h1D10000R')
    time.sleep(1)
    sendCommand(f'/{regAddress}m10h1P5000R')
    time.sleep(1)
    sendCommand(f'/{regAddress}m{presRegCur}h1D800000R')
    time.sleep(10)
    sendCommand(f'/{regAddress}z0R')
    sendCommand(f'/{regAddress}?0')
    print('Pressure regulator home\'d')

def homeVac(regAddress):
    #/4m5h1D1000000R
    print('Homing vacuum regulator', end='...... ')
    sendCommand(f'/{regAddress}m10h1D10000R')
    time.sleep(1)
    sendCommand(f'/{regAddress}m10h1P5000R')
    time.sleep(1)
    sendCommand(f'/{regAddress}m{vacRegCur}h1D1000000R')
    time.sleep(12)
    sendCommand(f'/{regAddress}z0R')
    sendCommand(f'/{regAddress}?0')
    print('Vacuum regulator home\'d')

def setPres(setpointPosition, regAddress): #include min/max handling
    sendCommand(f'/{regAddress}m7h1A{setpointPosition}R')
    time.sleep(5)

def setVac(setpointPosition, regAddress): #include min/max handling
    sendCommand(f'/{regAddress}m7h1A{setpointPosition}R')
    time.sleep(5)

def setPumpState(state, pumpAddress):
    if state == True:
        sendCommand(f'/{pumpAddress}J3R<CR>')
    else:
        sendCommand(f'/{pumpAddress}J0R<CR>')

def setFlowState(state, valveAddress):
    if state == True:
        sendCommand(f'/{valveAddress}J0R<CR>')
    else:
        sendCommand(f'/{valveAddress}J3R<CR>')

def setTestValveState(v1state, v2state, valveAddress, v1junc, v2junc):
    if v2state == True and v1state == True:
        sendCommand(f'/{valveAddress}J0R<CR>')
    elif v2state == True and v1state == False:
        sendCommand(f'/{valveAddress}J{v1junc}R<CR>')
    elif v2state == False and v1state == True:
        sendCommand(f'/{valveAddress}J{v2junc}R<CR>')
    else:
        sendCommand(f'/{valveAddress}J3R<CR>')

def storeData(flowArray):
    #tempData = np.zeros((4,5))
    #print(tempData)
    flowArray = np.append(flowArray, np.zeros((1,dataLength)), axis = 0)
    flowArray[-1] = getData()
    return flowArray

def getData():
    flow1,temp1,flag1 = device1.get_last()
        #print(f'sensor: 1 flow rate: {flow1} ul/min, temperature: {temp1} C, flag: {flag1}')
        #time.sleep(0.1)
    flow2,temp2,flag2 = device2.get_last()
        #print(f'sensor: 2 flow rate: {flow} ul/min, temperature: {temp} C, flag: {flag}')
        #time.sleep(0.1)
    data_array = [time.time(), flow1, temp1, flag1, flow2, temp2, flag2]
    return data_array

def recordFlow(duration,sensorData):
    [startIndex,column_number] = np.shape(sensorData)
    start = time.time()
    current = time.time()
    end = start + duration
    while current<end:
        sensorData = storeData(sensorData)
        #time.sleep(0.1)
        current = time.time()
        #print(int(time.time()))
        #print(sensorData[-1])
    [endIndex,column_number] = np.shape(sensorData)
    return sensorData, startIndex, endIndex

def initialize():
    try:
        sendCommand(f'/{pumpAdr}J0R<CR>')
        device1.start(water=True)
        device2.start(water=True)
        return True
    except:
        print("Failed to initialize sensors. Retry sensor initialization")
        return False

def calcStat(sensorData,startIndex,endIndex):
    arrayStatData = np.zeros((1,6))
    arrayStatData = sensorData[startIndex:endIndex,:]
    arrayStatSum = np.sum(sensorData[startIndex:endIndex,:],axis=0)
    arrayStatMean = np.mean(sensorData[startIndex:endIndex,:],axis=0)
    arrayStatDev = np.std(sensorData[startIndex:endIndex,:],axis=0)
    arrayStatVar = np.var(sensorData[startIndex:endIndex,:],axis=0)
    return arrayStatData, arrayStatSum, arrayStatMean, arrayStatDev, arrayStatVar

def lineFlowInitialize(sensorData,duration):
    print("Priming--------------------------------------------------")
    try:
        valve1_label = argCommands[2]#input("Valve 1 Label: ")
    except:
        valve1_label = '1'
    try:
        valve2_label = argCommands[3]#input("Valve 2 Label: ")
    except:
        valve2_label = '2'
    zeroFlowTime = 2
    setFlowState(False,valveFlowAdr) #close flow
    setTestValveState(False,False, valveTestAdr, valve1TestJun, valve2TestJun) #close test valves
    setPumpState(True,pumpAdr)
    #initialization, high flow
    print('Pressurizing system--------------------------------------')
    setPres(presRegSP5,presRegAdr) #set pressure to high pressure
    setVac(vacRegSP4,vacRegAdr)
    time.sleep(2) #allow for pressurization


    fig = plt.figure("Initialization Flow")
    fig.set_size_inches(32, 18) # set figure's size manually to your full screen (32x18)
    #fig.savefig('filename.png', bbox_inches='tight') # bbox_inches removes extra white spaces
    sp1 = plt.subplot(2,2,1)
    sp2 = plt.subplot(2,2,2)
    sp3 = plt.subplot(2,2,3)
    sp4 = plt.subplot(2,2,4)

    print("Opening lines--------------------------------------------")
    setFlowState(True,valveFlowAdr) #open flow
    setTestValveState(True,True,valveTestAdr,valve1TestJun,valve2TestJun) #open both test valves
    time.sleep(1)
    sensorData,t0,t1 = recordFlow(duration,sensorData)

    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close both test valves
    time.sleep(1)
    sensorData,t2,t3 = recordFlow(zeroFlowTime,sensorData)

    setTestValveState(True,False,valveTestAdr,valve1TestJun,valve2TestJun) #open test valve1 
    time.sleep(1)
    sensorData,t4,t5 = recordFlow(duration,sensorData)

    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close both test valves
    time.sleep(1)
    sensorData,t6,t7 = recordFlow(zeroFlowTime,sensorData)

    setTestValveState(True,True,valveTestAdr,valve1TestJun,valve2TestJun) #open test valve2 
    time.sleep(1)
    sensorData,t8,t9 =recordFlow(duration,sensorData)

    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close both test valves
    time.sleep(1)
    sensorData,t10,t11 = recordFlow(zeroFlowTime,sensorData)

    setTestValveState(True,True,valveTestAdr,valve1TestJun,valve2TestJun) #open both test valves
    time.sleep(1)
    sensorData,t12,t13 =recordFlow(duration,sensorData)

    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close both test valves
    time.sleep(1)
    sensorData,t14,t15 = recordFlow(zeroFlowTime,sensorData)

    """
    print("Testing calcStat()")
    sample = calcStat(sensorData,t1,t2)
    print(calcStat(sensorData,t1,t2))
    print(sample[statDat][:,dataFlow1])
    print(sample[statAvg][dataFlow1])
    print(sample[statDat][:,dataTemp1])
    print(sample[statAvg][dataTemp1])
    """

    #fig = plt.figure()
    #ax1 = plt.subplot(2,2,2)
    #ax1.plot(sample[statDat][:,dataTime], sample[statDat][:,dataTemp1])
    #plt.subplot(222, sample[statDat][:,dataTime], sample[statDat][:,dataFlow1])
    #=sample[statAvg][dataFlow1])
    #ax1.plot([], [], ' ', label="Mean = {0:.2f}".format(sample[statAvg][dataTemp1]))
    #ax1.legend(loc='upper left', title='Legend')
    #plt.text("Flow 1 Mean = d",va='top', ha='left')
    
    valveAllData = calcStat(sensorData,t0,t15)
    valve1Data = calcStat(sensorData,t4,t5)
    valve2Data = calcStat(sensorData,t8,t9)
    valveBothData = calcStat(sensorData,t12,t13)

    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow1])
    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow2])

    sp2.plot(valveBothData[statDat][:,dataTime], valveBothData[statDat][:,dataFlow1])
    sp2.plot(valveBothData[statDat][:,dataTime], valveBothData[statDat][:,dataFlow2])
    sp2.plot([], [], ' ', label="Valve 1 Mean = {0:.2f}".format(valveBothData[statAvg][dataFlow1]))
    sp2.plot([], [], ' ', label="Valve 2 Mean = {0:.2f}".format(valveBothData[statAvg][dataFlow2]))
    sp2.legend(loc='upper left', title='Both Valves On Flow')

    sp3.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow1])
    sp3.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow2])
    sp3.plot([], [], ' ', label="Valve 1 Mean = {0:.2f}".format(valve1Data[statAvg][dataFlow1]))
    sp3.plot([], [], ' ', label="VAlve 2 Mean = {0:.2f}".format(valve1Data[statAvg][dataFlow2]))
    sp3.legend(loc='upper left', title='Valve 1 On Flow')

    sp4.plot(valve2Data[statDat][:,dataTime], valve2Data[statDat][:,dataFlow1])
    sp4.plot(valve2Data[statDat][:,dataTime], valve2Data[statDat][:,dataFlow2])
    sp4.plot([], [], ' ', label="Mean = {0:.2f}".format(valve2Data[statAvg][dataFlow1]))
    sp4.plot([], [], ' ', label="Mean = {0:.2f}".format(valve2Data[statAvg][dataFlow2]))
    sp4.legend(loc='upper left', title='Valve 2 On Flow')
    fig.savefig(f"Initialization Flow Graph {time.localtime().tm_year}{time.localtime().tm_mon}{time.localtime().tm_mday}{time.localtime().tm_hour}{time.localtime().tm_min}{time.localtime().tm_sec} {argCommands[2]} {argCommands[3]}.png")
    """
    print()
    print("1-------------------------------------------------------1")
    print(sample[4])
    print(sample[1][1])
    print("---------------------------------------------------------")
    """
    #------------------
    setFlowState(False,valveFlowAdr) #close flow
    print("Priming Complete-----------------------------------------")
    return sensorData

def saveData(testName, testValue,testResult,valveName = 'NA', testDetails = ""):
    testReport = open("TestReports.txt","a")
    timestamp = f"{time.localtime().tm_year}{time.localtime().tm_mon:02}{time.localtime().tm_mday:02}{time.localtime().tm_hour:02}{time.localtime().tm_min:02}{time.localtime().tm_sec:02}"
    testReport.write(f"\n{timestamp}, {testName:>40}, {testValue:>30}, {testResult:>15},{valveName:>12},{testDetails}")
    testReport.close()

#TEST ALGORITHM
def openFlowTest(sensorData,duration:int ,spec1:int):
    zeroFlowTime = 2
    buffer = 30
    setFlowState(False,valveFlowAdr) #close flow
    setTestValveState(False,False, valveTestAdr, valve1TestJun, valve2TestJun) #close test valves
    setPumpState(True,pumpAdr)
    #initialization, high flow
    print('Setting system pressure', end='....... ')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    print('Pressure set to SP1')
    print('Setting system vacuum', end='....... ')
    setVac(vacRegSP1,vacRegAdr)
    print('Vacuum set to 0')
    #time.sleep(10) #allow for pressurization

    fig = plt.figure("Open Flow Test")
    fig.set_size_inches(32, 18) # set figure's size manually to your full screen (32x18)
    #fig.savefig('filename.png', bbox_inches='tight') # bbox_inches removes extra white spaces
    sp1 = plt.subplot(1,2,1)
    sp2 = plt.subplot(1,2,2)

    print("Opening lines", end='...... ')
    setFlowState(True,valveFlowAdr) #open flow valves
    print('Lines open')
    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close both test valves
    time.sleep(1)
    #setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun)
    sensorData,t0,t1 = recordFlow(1,sensorData)

    setTestValveState(True,False,valveTestAdr,valve1TestJun,valve2TestJun) #open test valve 1
    #time.sleep(1)
    sensorData,t2,t3 = recordFlow(duration,sensorData)

    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close test valve1 
    #time.sleep(1)
    sensorData,t4,t5 = recordFlow(1,sensorData)

    valveAllData = calcStat(sensorData,t0,t5)
    valve1Data = calcStat(sensorData,t2+buffer,t3)

    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow1])
    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow2])

    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow1])
    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow2])
    sp2.plot([], [], ' ', label="Valve 1 Mean = {0:.2f}".format(valve1Data[statAvg][dataFlow1]))
    #sp2.plot([], [], ' ', label="Valve 2 Mean = {0:.2f}".format(valveBothData[statAvg][dataFlow2]))
    sp2.legend(loc='upper left', title='Test Valve On')
    
    fig.savefig(f"Open Flow Test Graph {time.localtime().tm_year}{time.localtime().tm_mon:02}{time.localtime().tm_mday:02}{time.localtime().tm_hour:02}{time.localtime().tm_min:02}{time.localtime().tm_sec:02} {argCommands[4]}.png")
    fig.clear()
    #------------------
    setFlowState(False,valveFlowAdr) #close flow

    print("Open flow test complete-----------------------------------------")
    print("Valve mean on flow: ", valve1Data[statAvg][dataFlow1], " uL/min")
    if valve1Data[statAvg][dataFlow1] > spec1:
        testPF = 'Pass'
    else:
        testPF = 'Fail'
    saveData(testName = "Open Flow",testValue = valve1Data[statAvg][dataFlow1], valveName = argCommands[valveIdx], testResult = testPF)
    return valve1Data[statAvg][dataFlow1],testPF

def closedFlowTest(sensorData, duration:int, spec1:int):
    zeroFlowTime = 2
    buffer = 30
    setFlowState(False,valveFlowAdr) #close flow
    setTestValveState(False,False, valveTestAdr, valve1TestJun, valve2TestJun) #close test valves
    setPumpState(True,pumpAdr)
    #initialization, high flow
    print('Setting system pressure', end='....... ')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    print('Pressure set to SP1')
    print('Setting system vacuum', end='....... ')
    setVac(vacRegSP1,vacRegAdr)
    print('Vacuum set to 0')
    #time.sleep(10) #allow for pressurization


    fig = plt.figure("Closed Flow Test")
    fig.set_size_inches(32, 18) # set figure's size manually to your full screen (32x18)
    #fig.savefig('filename.png', bbox_inches='tight') # bbox_inches removes extra white spaces
    sp1 = plt.subplot(1,2,1)
    sp2 = plt.subplot(1,2,2)

    print("Opening lines", end='...... ')
    setFlowState(True,valveFlowAdr) #open flow valves
    print('Lines open')
    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close both test valves
    #time.sleep(1)
    setTestValveState(True,False,valveTestAdr,valve1TestJun,valve2TestJun) #open test valve 1
    sensorData,t0,t1 = recordFlow(1,sensorData)

    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close test valve 1
    #time.sleep(1)
    sensorData,t2,t3 = recordFlow(duration,sensorData)

    setTestValveState(True,False,valveTestAdr,valve1TestJun,valve2TestJun) #open test valve1 
    #time.sleep(1)
    sensorData,t4,t5 = recordFlow(1,sensorData)
    
    valveAllData = calcStat(sensorData,t0,t5)
    valve1Data = calcStat(sensorData,t2+buffer,t3)

    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow1])
    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow2])

    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow1])
    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow2])
    sp2.plot([], [], ' ', label="Valve 1 Mean = {0:.2f}".format(valve1Data[statAvg][dataFlow1]))
    #sp2.plot([], [], ' ', label="Valve 2 Mean = {0:.2f}".format(valveBothData[statAvg][dataFlow2]))
    sp2.legend(loc='upper left', title='Test Valve Off')
    
    fig.savefig(f"Closed Flow Test Graph {time.localtime().tm_year}{time.localtime().tm_mon:02}{time.localtime().tm_mday:02}{time.localtime().tm_hour:02}{time.localtime().tm_min:02}{time.localtime().tm_sec:02} {argCommands[4]}.png")
    fig.clear()
    #------------------
    setFlowState(False,valveFlowAdr) #close flow
    print("Closed flow test complete-----------------------------------------")
    print("Valve mean off flow: ", valve1Data[statAvg][dataFlow1], " uL/min")
    testPF = 'Fail'
    if valve1Data[statAvg][dataFlow1] < spec1:
        testPF = "Pass"
    saveData(testName = "Closed Flow",testValue = valve1Data[statAvg][dataFlow1], valveName = argCommands[valveIdx], testResult = testPF)
    print('Total Volume: ' + str(valve1Data[statSum][dataFlow1]))
    return valve1Data[statAvg][dataFlow1], testPF

def valveCycleTest(sensorData,cycleNumber:int):
    zeroFlowTime = 2
    setFlowState(False,valveFlowAdr) #close flow
    setTestValveState(False,False, valveTestAdr, valve1TestJun, valve2TestJun) #close test valves
    setPumpState(False,pumpAdr)

    openFlowStartValue,openFlowStartResult = openFlowTest(sensorData,duration=argCommands[2],spec1=specOF_meanFlowMinimum)
    closedFlowStartValue,closedFlowStartResult = closedFlowTest(sensorData, duration=argCommands[2], spec1=specCF_meanFlowMaximum)

    for i in range(0,cycleNumber):
        setTestValveState(True,False,valveTestAdr, valve1TestJun, valve2TestJun)
        time.sleep(.25)
        setTestValveState(False,False,valveTestAdr, valve1TestJun, valve2TestJun)
        time.sleep(.25)

    openFlowEndValue,openFlowEndResult = openFlowTest(sensorData,duration=30,spec1=specOF_meanFlowMinimum)
    closedFlowEndValue,closedFlowEndResult = closedFlowTest(sensorData, duration=30, spec1=specCF_meanFlowMaximum)

    testPF = 'Fail'
    testSummary = ''
    if openFlowEndValue >= openFlowStartValue*(1-specVC_meanFlowPercent) or openFlowEndValue <= openFlowStartValue*(1+specVC_meanFlowPercent):
        if closedFlowEndValue >= closedFlowStartValue*(1-specVC_meanFlowPercent) or closedFlowEndValue <= closedFlowStartValue*(1+specVC_meanFlowPercent): 
            testPF = 'Pass'
        else:
            testSummary += 'Closed Flow Difference: '+ str(closedFlowEndValue - closedFlowStartValue)+', '

    else:
        testSummary += 'Open Flow Difference: '+ str(openFlowEndValue - openFlowStartValue)+', '
    print('Valve Cycle Test Complete: ', cycleNumber, ' cycles performed')
    print('Open Flow Test mean flow: ',openFlowEndValue, 'uL/min')
    print('Closed Flow Test mean flow',closedFlowEndValue, 'uL/min')
    saveData(testName = "Valve Cycle Test",testValue = openFlowEndValue/openFlowStartValue, testResult = testPF, valveName = argCommands[valveIdx], testDetails = testSummary)
    return openFlowEndValue/openFlowStartValue,testPF

def openflowrepeatability(sensorData,iteration:int, duration:int,spec1:int,spec2:float):
    zeroFlowTime = 2
    setFlowState(False,valveFlowAdr)
    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun)
    setPumpState(False,pumpAdr)
    print('Pressurizing system--------------------------------------')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    setVac(vacRegSP1,vacRegAdr)
    #time.sleep(10) #allow for pressurization

    meanFlow = np.zeros(0)
    flowPF = []
    actNum = np.zeros(0)
    for i in range (0,iteration):
        r1,r2,= openFlowTest(sensorData,duration,spec1=spec1)
        meanFlow = np.append(meanFlow,r1)
        flowPF = np.append(flowPF,r2)
        actNum = np.append(actNum,i)
        print(meanFlow)
        print(actNum)
    X_b = np.c_[np.ones((iteration,1)), actNum]
    theta = np.linalg.inv(X_b.T.dot(X_b)).dot(X_b.T).dot(meanFlow)
    Y = np.zeros(0)
    X = np.zeros(0)
    for i in range(0,iteration):
        X = np.append(X,i)
        Y = np.append(Y,theta[1]*i + theta[0])
    fig = plt.figure("Open Flow Repeatability Test")
    fig.set_size_inches(32, 18)
    plt.plot(actNum[:],meanFlow[:])
    plt.plot(X[:],Y[:])
    plt.plot([], [], ' ', label="Mean Flow [b] = {0:.2f}".format(theta[0]))
    plt.plot([], [], ' ', label="Mean Flow [m] = {0:.2f}".format(theta[1]))
    plt.legend(loc='upper left', title='Repeatability Info')
    fig.savefig(f"Open Flow Repeatability Test Graph {time.localtime().tm_year}{time.localtime().tm_mon:02}{time.localtime().tm_mday:02}{time.localtime().tm_hour:02}{time.localtime().tm_min:02}{time.localtime().tm_sec:02} {argCommands[4]}.png")
    fig.clear()
    testPF = 'Fail'
    testSummary = ''
    if theta[1] < spec2:
        testPF = "Pass"
    for i in flowPF:
        if i == 'Fail':
            testPF = 'Fail'
            testSummary = 'At least x1 Open Flow Test Failed'
    saveData(testName = "Open Flow Repeatability",testValue = theta[1], testResult = testPF, valveName = argCommands[valveIdx], testDetails = testSummary)
    return theta[1],testPF

def closedflowrepeatability(sensorData,iteration:int,duration:int,spec1:int,spec2:float):
    zeroFlowTime = 2
    setFlowState(False,valveFlowAdr)
    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun)
    setPumpState(False,pumpAdr)
    print('Pressurizing system--------------------------------------')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    setVac(vacRegSP1,vacRegAdr)
    #time.sleep(10) #allow for pressurization

    meanFlow = np.zeros(0)
    flowPF = []
    actNum = np.zeros(0)
    for i in range (0,iteration):
        r1,r2 = closedFlowTest(sensorData,duration,spec1)
        meanFlow = np.append(meanFlow,r1)
        flowPF = np.append(flowPF,r2)
        actNum = np.append(actNum,i)
    X_b = np.c_[np.ones((iteration,1)), actNum]
    theta = np.linalg.inv(X_b.T.dot(X_b)).dot(X_b.T).dot(meanFlow)
    Y = np.zeros(0)
    X = np.zeros(0)
    for i in range(0,iteration):
        X = np.append(X,i)
        Y = np.append(Y,theta[1]*i + theta[0])
    fig = plt.figure("Closed Flow Repeatability Test")
    fig.set_size_inches(32, 18)
    plt.plot(actNum[:],meanFlow[:])
    plt.plot(X[:],Y[:])
    plt.plot([], [], ' ', label="Mean Flow [b] = {0:.2f}".format(theta[0]))
    plt.plot([], [], ' ', label="Mean Flow [m] = {0:.2f}".format(theta[1]))
    plt.legend(loc='upper left', title='Repeatability Info')
    fig.savefig(f"Closed Flow Repeatability Test Graph {time.localtime().tm_year}{time.localtime().tm_mon:02}{time.localtime().tm_mday:02}{time.localtime().tm_hour:02}{time.localtime().tm_min:02}{time.localtime().tm_sec:02} {argCommands[4]}.png")
    fig.clear()
    testPF = 'Fail'
    testSummary = ''
    if theta[1] < spec2:
        testPF = "Pass"
    for i in flowPF:
        if i == 'Fail':
            testPF = 'Fail'
            testSummary = 'At least x1 Closed Flow Test Failed'
    saveData(testName = "Closed Flow Repeatability",testValue = theta[1], valveName = argCommands[valveIdx], testResult = testPF)
    return theta[1],testPF
 
def highPressureClosedFlowTest(sensorData,duration:int,spec1:int):
    zeroFlowTime = 2
    buffer = 30
    setFlowState(False,valveFlowAdr) #close flow
    setTestValveState(False,False, valveTestAdr, valve1TestJun, valve2TestJun) #close test valves
    setPumpState(True,pumpAdr)
    #initialization, high flow
    print('Setting system pressure', end='....... ')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    print('Pressure set to SP1')
    print('Setting system vacuum', end='....... ')
    setVac(vacRegMin,vacRegAdr)
    print('Vacuum set to 0')
    #time.sleep(10) #allow for pressurization
    time.sleep(10) #allow for pressurization


    fig = plt.figure("High Pressure Closed Flow Test")
    fig.set_size_inches(32, 18) # set figure's size manually to your full screen (32x18)
    #fig.savefig('filename.png', bbox_inches='tight') # bbox_inches removes extra white spaces
    sp1 = plt.subplot(1,2,1)
    sp2 = plt.subplot(1,2,2)

    print("Opening lines", end='...... ')
    setFlowState(True,valveFlowAdr) #open flow valves
    print('Lines open')
    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close both test valves
    time.sleep(1)
    setTestValveState(True,False,valveTestAdr,valve1TestJun,valve2TestJun) #open test valve 1
    sensorData,t0,t1 = recordFlow(1,sensorData)

    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close test valve 1
    #time.sleep(1)
    sensorData,t2,t3 = recordFlow(duration,sensorData)

    setTestValveState(True,False,valveTestAdr,valve1TestJun,valve2TestJun) #open test valve1 
    #time.sleep(1)
    sensorData,t4,t5 = recordFlow(1,sensorData)
    
    valveAllData = calcStat(sensorData,t0,t5)
    valve1Data = calcStat(sensorData,t2+buffer,t3)

    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow1])
    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow2])

    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow1])
    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow2])
    sp2.plot([], [], ' ', label="Valve 1 Mean = {0:.2f}".format(valve1Data[statAvg][dataFlow1]))
    #sp2.plot([], [], ' ', label="Valve 2 Mean = {0:.2f}".format(valveBothData[statAvg][dataFlow2]))
    sp2.legend(loc='upper left', title='Test Valve Off')
    
    fig.savefig(f"High Pressure Close Flow Test Graph {time.localtime().tm_year}{time.localtime().tm_mon}{time.localtime().tm_mday}{time.localtime().tm_hour}{time.localtime().tm_min}{time.localtime().tm_sec} {argCommands[4]}.png")
    fig.clear()

    setFlowState(False,valveFlowAdr) #close flow
    print("High Pressure Closed flow Test complete-----------------------------------------")
    setPumpState(False,pumpAdr)
    print("Valve mean off flow: ", valve1Data[statAvg][dataFlow1], " uL/min")
    print('Setting system pressure', end='....... ')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    print('Pressure set to SP1')
    print('Setting system vacuum', end='....... ')
    setVac(vacRegMin,vacRegAdr)
    print('Vacuum set to 0')
    testPF = 'Fail'
    if valve1Data[statAvg][dataFlow1] < spec1:
        testPF = "Pass"
    saveData(testName = "High Pressure Closed Flow",testValue = valve1Data[statAvg][dataFlow1], valveName = argCommands[valveIdx], testResult = testPF)
    return valve1Data[statAvg][dataFlow1], testPF
    
def valveRampUpTimeTest(sensorData,duration:int,spec1:int):
    zeroFlowTime = 2
    setFlowState(False,valveFlowAdr) #close flow
    setTestValveState(False,False, valveTestAdr, valve1TestJun, valve2TestJun) #close test valves
    setPumpState(True,pumpAdr)
    #initialization, high flow
    #print('Pressurizing system--------------------------------------')
    #setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    #setVac(vacRegSP1,vacRegAdr)
    print('Setting system pressure', end='....... ')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    print('Pressure set to SP1')
    print('Setting system vacuum', end='....... ')
    setVac(vacRegSP1,vacRegAdr)
    print('Vacuum set to SP1')
    #time.sleep(10) #allow for pressurization


    fig = plt.figure("Valve Ramp Up Test")
    fig.set_size_inches(32, 18) # set figure's size manually to your full screen (32x18)
    #fig.savefig('filename.png', bbox_inches='tight') # bbox_inches removes extra white spaces
    sp1 = plt.subplot(1,2,1)
    sp2 = plt.subplot(1,2,2)

    print("Opening lines--------------------------------------------")
    setFlowState(True,valveFlowAdr) #open flow valves
    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close both test valves
    #time.sleep(1)
    sensorData,t0,t1 = recordFlow(1,sensorData)

    setTestValveState(True,False,valveTestAdr,valve1TestJun,valve2TestJun) #open test valve 1
    #time.sleep(1)
    sensorData,t2,t3 = recordFlow(duration,sensorData)

    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #close test valve1 
    #time.sleep(1)
    sensorData,t4,t5 = recordFlow(1,sensorData)

    valveAllData = calcStat(sensorData,t0,t5)
    valve1Data = calcStat(sensorData,t2,t3)

    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow1])
    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow2])

    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow1])
    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow2])
    sp2.plot([], [], ' ', label="Valve 1 Mean = {0:.2f}".format(valve1Data[statAvg][dataFlow1]))
    #sp2.plot([], [], ' ', label="Valve 2 Mean = {0:.2f}".format(valveBothData[statAvg][dataFlow2]))
    sp2.legend(loc='upper left', title='Test Valve On')
    
    fig.savefig(f"Valve Ramp Up Time Test Graph {time.localtime().tm_year}{time.localtime().tm_mon}{time.localtime().tm_mday}{time.localtime().tm_hour}{time.localtime().tm_min}{time.localtime().tm_sec} {argCommands[4]}.png")
    fig.clear()
    #------------------
    setFlowState(False,valveFlowAdr) #close flow
    rampUpTime = 2
    for i in range(0,len(valve1Data[statDat])-3):
        if ((valve1Data[statDat][i,dataFlow1] > spec1) and (valve1Data[statDat][i+1,dataFlow1] > spec1) and (valve1Data[statDat][i+2,dataFlow1] > spec1) and (valve1Data[statDat][i+3,dataFlow1] > spec1))  : #openflowmax
            rampUpTime = valve1Data[statDat][i,dataTime] - valve1Data[statDat][0,dataTime]
            print(i)
            break
    print(valve1Data[statDat][:,dataFlow1])    
    print("Valve Ramp Up Time Test complete-----------------------------------------")
    print("Valve ramp up time: ", rampUpTime, " seconds")
    testPF = 'Fail'
    if rampUpTime < 2:
        testPF = "Pass"
    saveData(testName = "Valve Ramp Up Time",testValue = rampUpTime, valveName = argCommands[valveIdx], testResult = testPF)
    saveData(testName = "Open Flow",testValue = valve1Data[statAvg][dataFlow1], valveName = argCommands[valveIdx], testResult = testPF)
    return rampUpTime, testPF

def valveRampDownTimeTest(sensorData,duration:int, spec1:int):
    zeroFlowTime = 2
    setFlowState(False,valveFlowAdr) #close flow
    setTestValveState(False,False, valveTestAdr, valve1TestJun, valve2TestJun) #close test valves
    setPumpState(True,pumpAdr)

    print('Setting system pressure', end='....... ')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    print('Pressure set to SP1')
    print('Setting system vacuum', end='....... ')
    setVac(vacRegSP1,vacRegAdr)
    print('Vacuum set to SP1')


    fig = plt.figure("Valve Ramp Down Test")
    fig.set_size_inches(32, 18) # set figure's size manually to your full screen (32x18)
    #fig.savefig('filename.png', bbox_inches='tight') # bbox_inches removes extra white spaces
    sp1 = plt.subplot(1,2,1)
    sp2 = plt.subplot(1,2,2)

    print("Opening lines--------------------------------------------")
    setFlowState(True,valveFlowAdr) #open flow valves
    setTestValveState(True,False,valveTestAdr,valve1TestJun,valve2TestJun) #close both test valves
    sensorData,t0,t1 = recordFlow(1,sensorData)

    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun) #open test valve 1
    #time.sleep(1)
    sensorData,t2,t3 = recordFlow(duration,sensorData)

    setTestValveState(True,False,valveTestAdr,valve1TestJun,valve2TestJun) #close test valve1 
    #time.sleep(1)
    sensorData,t4,t5 = recordFlow(1,sensorData)

    valveAllData = calcStat(sensorData,t0,t5)
    valve1Data = calcStat(sensorData,t2,t3)

    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow1])
    sp1.plot(valveAllData[statDat][:,dataTime], valveAllData[statDat][:,dataFlow2])

    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow1])
    sp2.plot(valve1Data[statDat][:,dataTime], valve1Data[statDat][:,dataFlow2])
    sp2.plot([], [], ' ', label="Valve 1 Mean = {0:.2f}".format(valve1Data[statAvg][dataFlow1]))
    #sp2.plot([], [], ' ', label="Valve 2 Mean = {0:.2f}".format(valveBothData[statAvg][dataFlow2]))
    sp2.legend(loc='upper left', title='Test Valve Off')
    
    fig.savefig(f"Valve Ramp Down Time Test Graph {time.localtime().tm_year}{time.localtime().tm_mon}{time.localtime().tm_mday}{time.localtime().tm_hour}{time.localtime().tm_min}{time.localtime().tm_sec} {argCommands[4]}.png")
    fig.clear()
    #------------------
    setFlowState(False,valveFlowAdr) #close flow
    rampDownTime = 2
    for i in range(0,len(valve1Data[statDat])-3):
        if ((valve1Data[statDat][i,dataFlow1] < spec1) and (valve1Data[statDat][i+1,dataFlow1] < spec1) and (valve1Data[statDat][i+2,dataFlow1] < spec1) and (valve1Data[statDat][i+3,dataFlow1] < spec1)): #close flow minimum
            rampDownTime = valve1Data[statDat][i,dataTime] - valve1Data[statDat][0,dataTime]
            break
    print("Valve Ramp Down Time Test complete-----------------------------------------")
    print("Valve ramp down time: ", rampDownTime, " seconds")
    testPF = 'Fail'
    if rampDownTime < 2:
        testPF = "Pass"
    saveData(testName = "Valve Ramp Down Time",testValue = rampDownTime, valveName = argCommands[valveIdx], testResult = testPF)
    saveData(testName = "Closed Flow",testValue = valve1Data[statAvg][dataFlow1], valveName = argCommands[valveIdx], testResult = testPF)
    return rampDownTime, testPF

def valveRampUpTimeRepeatabilityTest(sensorData,iteration:int,spec1:int):
    zeroFlowTime = 2
    setFlowState(False,valveFlowAdr)
    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun)
    setPumpState(False,pumpAdr)
    print('Pressurizing system--------------------------------------')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    setVac(vacRegSP1,vacRegAdr)
    #time.sleep(10) #allow for pressurization

    rampUpTime = np.zeros(0)
    rampPF = []
    actNum = np.zeros(0)
    for i in range (0,iteration):
        #sensorData,meanFlowValue = openFlowTest(sensorData,1)
        r1,r2 = valveRampUpTimeTest(sensorData,duration = 2, spec1 = specVRUT_meanFlowMinimum)
        rampUpTime = np.append(rampUpTime, r1)
        rampPF = np.append(rampPF, r2)
        actNum = np.append(actNum,i)
        print(rampUpTime)
        print(actNum)
    X_b = np.c_[np.ones((iteration,1)), actNum]
    theta = np.linalg.inv(X_b.T.dot(X_b)).dot(X_b.T).dot(rampUpTime)
    Y = np.zeros(0)
    X = np.zeros(0)
    for i in range(0,iteration):
        X = np.append(X,i)
        Y = np.append(Y,theta[1]*i + theta[0])
    print(theta)
    #print(f"Intercept: {theta[0][0]:.2f}")
    #print(f"Slope: {theta[1][0]:.2f}")
    fig = plt.figure("Valve Ramp Up Time Repeatability Test")
    fig.set_size_inches(32, 18)
    plt.plot(actNum[:],rampUpTime[:])
    plt.plot(X[:],Y[:])
    plt.plot([], [], ' ', label="Ramp Up Time [b] = {0:.2f}".format(theta[0]))
    plt.plot([], [], ' ', label="Ramp Up Time [m] = {0:.2f}".format(theta[1]))
    plt.legend(loc='upper left', title='Repeatability Info')
    fig.savefig(f"Valve Ramp Up Time Repeatability Test Graph {time.localtime().tm_year}{time.localtime().tm_mon:02}{time.localtime().tm_mday:02}{time.localtime().tm_hour:02}{time.localtime().tm_min:02}{time.localtime().tm_sec:02} {argCommands[4]}.png")
    fig.clear()
    testPF = 'Fail'
    testSummary = ''
    if theta[1] < spec1:
        testPF = "Pass"
    for i in rampPF:
        if i == 'Fail':
            testPF = 'Fail'
            testSummary = 'at least x1 Ramp Up Time Test Failed'
    saveData(testName = "Valve Ramp Up Time Repeatability",testValue = theta[1], testResult = testPF, valveName = argCommands[valveIdx], testDetails = testSummary)
    return theta[1], testPF

def valveRampDownTimeRepeatabilityTest(sensorData, iteration: int, spec1:int):
    zeroFlowTime = 2
    setFlowState(False,valveFlowAdr)
    setTestValveState(False,False,valveTestAdr,valve1TestJun,valve2TestJun)
    setPumpState(False,pumpAdr)
    print('Pressurizing system--------------------------------------')
    setPres(presRegSP1,presRegAdr) #set pressure to high pressure
    setVac(vacRegSP1,vacRegAdr)
    #time.sleep(10) #allow for pressurization

    rampDownTime = np.zeros(0)
    rampPF = []
    actNum = np.zeros(0)
    for i in range (0,iteration):
        #sensorData,meanFlowValue = openFlowTest(sensorData,1)
        r1,r2 = valveRampDownTimeTest(sensorData,duration = 2, spec1 = specVRDT_meanFlowMaximum)
        rampDownTime = np.append(rampDownTime,r1)
        rampPF = np.append(rampPF,r2)
        actNum = np.append(actNum,i)
        print(rampDownTime)
        print(actNum)
    X_b = np.c_[np.ones((iteration,1)), actNum]
    theta = np.linalg.inv(X_b.T.dot(X_b)).dot(X_b.T).dot(rampDownTime)
    Y = np.zeros(0)
    X = np.zeros(0)
    for i in range(0,iteration):
        X = np.append(X,i)
        Y = np.append(Y,theta[1]*i + theta[0])
    print(theta)
    #print(f"Intercept: {theta[0][0]:.2f}")
    #print(f"Slope: {theta[1][0]:.2f}")
    fig = plt.figure("Valve Ramp Down Time Repeatability Test")
    fig.set_size_inches(32, 18)
    plt.plot(actNum[:],rampDownTime[:])
    plt.plot(X[:],Y[:])
    plt.plot([], [], ' ', label="Ramp Down Time [b] = {0:.2f}".format(theta[0]))
    plt.plot([], [], ' ', label="Ramp Down Time [m] = {0:.2f}".format(theta[1]))
    plt.legend(loc='upper left', title='Repeatability Info')
    fig.savefig(f"Valve Ramp Down Time Repeatability Test Graph {time.localtime().tm_year}{time.localtime().tm_mon:02}{time.localtime().tm_mday:02}{time.localtime().tm_hour:02}{time.localtime().tm_min:02}{time.localtime().tm_sec:02} {argCommands[4]} .png")
    fig.clear()
    testPF = 'Fail'
    testSummary = ''
    if theta[1] < spec1:
        testPF = "Pass"
    for i in rampPF:
        if i == 'Fail':
            testPF = 'Fail'
            testSummary = 'At least x1 Ramp Down Time Test Failed'
    saveData(testName = "Valve Ramp Down Time Repeatability",testValue = theta[1], valveName = argCommands[valveIdx], testResult = testPF)
    return theta[1], testPF

if __name__ == '__main__' :
    try:
        initialize()
        homePres(presRegAdr)
        homeVac(vacRegAdr)
        time.sleep(1)
    except Exception as e:
        print('Error. Stopping.')
        traceback.print_exc()
        print(e)
        shutdown()
    #saveData(testName = 'Test Valve ID', testValue = '', testResult='', testDetails = 'Test Valve: '+ str(argCommands[valveIdx]))
    testStartTime = time.time()
    if argCommands[1] == "initialize" or argCommands[1] == 'all':
        try:
            print('Running System Initialization')
            sensorData = lineFlowInitialize(sensorData,2)
            saveData(testName = 'Prime Lines', testValue = '', testResult = '', valveName = argCommands[valveIdx], testDetails = 'Prime lines for valve: '+ str(argCommands[valveIdx]))
        except Exception as e:
            print("initialization failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[openFlowIdx] or argCommands[1] == 'all':
        try:
            print('Running Open Flow Test')
            testValueArray[openFlowIdx], testResultArray[openFlowIdx] = openFlowTest(sensorData,duration = 30,spec1 = specOF_meanFlowMinimum)
        except Exception as e:
            print("Open Flow Test failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[closedFlowIdx] or argCommands[1] == 'all':
        try:
            print('Running Closed Flow Test')
            testValueArray[closedFlowIdx], testResultArray[closedFlowIdx] = closedFlowTest(sensorData,duration = 30, spec1 = specCF_meanFlowMaximum)
        except Exception as e:
            print("Closed Flow Test failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[valveCycleIdx] or argCommands[1] == 'all':
        try:
            print('Running Valve Cycle Test')
            testValueArray[valveCycleIdx],testResultArray[valveCycleIdx] = valveCycleTest(sensorData,specVC_cycleIteration)
        except Exception as e:
            print("Valve Cycle Test failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[openFlowRepeatIdx] or argCommands[1] == 'all':
        try:
            print('Running Open Flow Repeatability Test')
            testValueArray[openFlowRepeatIdx],testResultArray[openFlowRepeatIdx] = openflowrepeatability(sensorData,duration = argCommands[2],iteration = argCommands[3],spec1 = specOFR_meanFlowMinimum, spec2 = specOFR_meanFlowSlope)
        except Exception as e:
            print("Open Flow Repeatability Test failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[closedFlowRepeatIdx] or argCommands[1] == 'all':
        try:
            print('Running Closed Flow Repeatability Test')
            testValueArray[closedFlowRepeatIdx],testResultArray[closedFlowRepeatIdx] = closedflowrepeatability(sensorData,duration = argCommands[durationIdx],iteration = argCommands[iterationIdx],spec1 = specCFR_meanFlowMinimum, spec2 = specCFR_meanFlowSlope)
        except Exception as e:
            print("Open Flow Repeatability Test failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[highPressureIdx] or argCommands[1] == 'all':
        try:
            print('Running High Pressure Closed Flow Test')
            testValueArray[highPressureIdx],testResultArray[highPressureIdx] = highPressureClosedFlowTest(sensorData,duration = argCommands[durationIdx],spec1 = specHPCF_meanFlowMaximum)
        except Exception as e:
            print("High Pressure Closed Flow Test failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[valveRampUpIdx]  or argCommands[1] == 'all':
        try:
            print('Running Valve Ramp Up Time Test')
            testValueArray[valveRampUpIdx],testResultArray[valveRampUpIdx] = valveRampUpTimeTest(sensorData,duration = specVRUT_duration,spec1= specVRUT_meanFlowMinimum)
        except Exception as e:
            print("Valve Ramp Up Time Test failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[valveRampUpRepeatIdx] or argCommands[1] == 'all':
        try:
            print('Running Valve Ramp Up Time Repeatability Test')
            testValueArray[valveRampUpRepeatIdx],testResultArray[valveRampUpRepeatIdx] = valveRampUpTimeRepeatabilityTest(sensorData,iteration = specVRUTR_runIteration,spec1 = specVRUTR_rampUpSlope)
        except Exception as e:
            print("Valve Ramp Up Time Test Repeatability failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[valveRampDownIdx]  or argCommands[1] == 'all':
        try:
            print('Running Valve Ramp Down Time Test')
            testValueArray[valveRampDownIdx],testResultArray[valveRampDownIdx] = valveRampDownTimeTest(sensorData,duration = specVRDT_duration,spec1 = specVRDT_meanFlowMaximum)
        except Exception as e:
            print("Valve Ramp Down Time Test failed to run")
            traceback.print_exc()
            shutdown()
    if argCommands[1] == commandList[valveRampDownRepeatIdx]  or argCommands[1] == 'all':
        try:
            print('Running Valve Ramp Down Time Repeatability Test')
            testValueArray[valveRampDownRepeatIdx],testResultArray[valveRampDownRepeatIdx] = valveRampDownTimeRepeatabilityTest(sensorData,iteration = specVRDTR_runIteration,spec1= specVRDTR_rampDownSlope)
        except Exception as e:
            print("Valve Ramp Down Time Test Repeatability failed to run")
            traceback.print_exc()
            shutdown()
    testEndTime = time.time()
    testRunTime = testEndTime - testStartTime

    print('Test Run Time: ', int(testRunTime), ' seconds')
    print(testResultArray)
    setTestValveState(True,True,valveTestAdr,valve1TestJun,valve2TestJun) #de-energize test valves
    sendCommand(f'/{valveFlowAdr}J3R<CR>') #close flow
    sendCommand(f'/{pumpAdr}J0R<CR>') #close pump
    setPres(presRegMin,presRegAdr)
    setVac(vacRegMin,vacRegAdr)
    device1.stop()
    device2.stop()
