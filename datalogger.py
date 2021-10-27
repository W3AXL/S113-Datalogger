import serial
import argparse
import datetime
import struct
import numpy
from prefixed import Float
import matplotlib.pyplot as plt

# Add arguments and query
parser = argparse.ArgumentParser()
parser.add_argument("-p","--port",metavar="COM1",help="Serial device for communication with S113")
parser.add_argument("-r","--recall",metavar="4",help="Query saved trace, don't run new sweep. (0 = last sweep on screen)")
parser.add_argument("-f1","--freq-start",metavar="100M",help="Start frequency in Hz")
parser.add_argument("-f2","--freq-stop",metavar="1G",help="Stop frequency in Hz")
parser.add_argument("-v","--verbose",help="verbose logging",action="store_true")
parser.add_argument("-f","--csvfile",metavar="file.csv",help="Save data to CSV file")
parser.add_argument("-S","--show-plot",help="Show plot of retrieved data",action="store_true")

# Parse arguments
args = parser.parse_args()

portNo = ""
recall = None
verb = False
file = None
plot = False

# Verify we have a port first
if not args.port:
    print("ERROR: No serial port specified!")
    exit(1)
else:
    portNo = args.port

if args.recall:
    recall = int(args.recall)

if args.verbose:
    verb = True

if args.csvfile:
    file = args.csvfile

if args.show_plot:
    plot = True

if (not file) and (not plot):
    print("ERROR: not showing plot or saving file. Nothing to do!")
    exit(1)

# Create serial port
port = serial.Serial(portNo, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=5)

# Functions

def parseFreq(freqString):
    """
    Convert an SI string (100M, 3.2G, etc) to a float

    Args:
        freqString (string): number string

    Returns:
        float: float representation of number
    """
    return float(Float(freqString))

def sendBytes(bytes):
    """
    Sends the byte string to the S113

    Args:
        bytes (bytearray): array of bytes to send
    """
    if isinstance(bytes, int):
        singlebyte = bytes
        bytes = bytearray()
        bytes.append(singlebyte)

    port.write(bytes)

    if verb:
        print("Sent: {}".format(bytes))

def connect():
    """
    Sends remote enable command to S113
    """
    sendBytes(0x45)
    response = port.read(13)
    devInfo = response[2:].decode("ascii")
    if verb:
        print("Connected to {}".format(devInfo))

def disconnect():
    """
    Sends remote disable command to S113
    """
    sendBytes(0xFF)
    response = port.read(1)
    if 0xFF in response:
        print("Disconnected")
    else:
        print("Error disconnecting from S113")

def setupSystem(cwMode=False, keyLock=False, backlight=False, unit="English", cal=False):
    """
    Sends control byte 1 with specified data

    Args:
        cwMode (bool, optional): Enable or disable fixed CW mode. Defaults to False.
        keyLock (bool, optional): Enable or disable keypad lock. Defaults to False.
        backlight (bool, optional): Enable or disable backlight. Defaults to False.
        unit (str, optional): Set measurement units to English or Metric. Defaults to "English".
        cal (bool, optional): Enables or disable calibration. Defaults to False.
    """

    # start our data with all bits disabled
    data = 0

    # LSB is CW mode
    data |= (cwMode << 0)
    # Bit 1: Keyboard Lock
    data |= (keyLock << 1)
    # Bit 2: Backlight
    data |= (backlight << 2)
    # Bit 3: Units (0 = English, 1 = Metric)
    if unit == "Metric":
        data |= (1 << 3)
    # Bit 4: Cal On/Off
    data |= (cal << 4)

    # Send
    bytes = bytearray()
    bytes.append(0x01)
    bytes.append(data)
    sendBytes(bytes)

    # Recv
    resp = port.read(1)
    if 0xFF not in resp:
        print("Got error {} when sending control byte".format(resp))

def setFrequency(start, stop):
    """
    Sets frequency span for sweep

    Args:
        start (int): start frequency in Hz
        stop (int): stop frequency in Hz
    """

    # Convert ints to uint32 bytes
    startBytes = struct.pack('>I', start)
    stopBytes = struct.pack('>I', stop)

    bytes = bytearray()

    # Set freq command
    bytes.append(0x02)
    # Freq bytes
    bytes.append(startBytes)
    bytes.append(stopBytes)

    # Send
    sendBytes(bytes)

    # Wait for reply
    resp = port.read(1)

    # Process
    if 0xFF not in resp:
        return False
    else:
        return True

def setDomain(dtf=False, graph="SWR"):
    """
    Set domain format and graph type

    Args:
        dtf (bool, optional): Whether to use distance domain or frequency domain. Defaults to False.
        graph (str, optional): SWR, RL (Return Loss), or IL (Insertion Loss). Defaults to "SWR".
    """

    bytes = bytearray()

    # Set domain
    bytes.append(0x03)

    if dtf:
        bytes.append(0x01)
    else:
        bytes.append(0x00)

    if graph == "RL":
        bytes.append(0x01)
    elif graph == "IL":
        bytes.append(0x02)
    else:
        bytes.append(0x00)

    sendBytes(bytes)

    resp = port.read(1)

    if 0xFF not in resp:
        return False
    else:
        return True

def setScale(start, stop):
    """
    Sets y-axis scale of the graph. Numbers are always uint16. 1/1000th of a dB for RL/IL and 1/1000th of ratio for SWR

    NOTE:   RL scale is 0 to 54000 (0.0 dB to 54.0 dB)
            SWR scale is 1000 to 65535 (1.00 to 65.53)

    Args:
        start (uint16): Start value
        stop (uint16): Stop value
    """

    # Convert ints to bytes
    startBytes = struct.pack('>H', start)
    stopBytes = struct.pack('>H', stop)

    bytes = bytearray()

    # Set scale command
    bytes.append(0x04)

    # Add numbers
    bytes.append(startBytes)
    bytes.append(stopBytes)

    # Send
    sendBytes(bytes)

    resp = port.read(1)

    if 0xFF not in resp:
        return False
    else:
        return True

def setTime():
    """
    Sets time/date of S113 based on current date/time of PC
    """
    
    # Get time strings
    curTime = datetime.datetime.now()
    timeStr = curTime.strftime("%H:%M:%S")
    dateStr = curTime.strftime("%m/%d/%y")

    # Encode to ASCII bytes
    time = timeStr.encode('ascii')
    date = dateStr.encode('ascii')

    bytes = bytearray()

    # Command
    bytes.append(0x08)

    # Strings
    bytes.append(time)
    bytes.append(date)

    # Send
    sendBytes(bytes)

    resp = port.read(1)

    if 0xFF not in resp:
        return False
    else:
        return True

def recallData(idx):
    """
    Recall data from the S113 into the dataArray and then process it

    Args:
        idx (int): saved data index. 0 is current sweep on screen
    """
    
    print("Recalling data in index {}".format(idx))
    
    # Query current status first to get sweep parameters
    sendBytes(0x14)

    # Read 43 bytes back
    status = port.read(43)

    # Get all the info

    # Frequency or distance domain
    domain = status[0]

    # Frequencies in MHz
    freqStart = float(struct.unpack('>I', status[1:5])[0])/1e6
    freqStop = float(struct.unpack('>I', status[5:9])[0])/1e6

    # Scales
    scaleStart = float(struct.unpack('>H', status[9:11])[0])
    scaleStop = float(struct.unpack('>H', status[11:13])[0])

    # Frequency markers
    freqMkr1 = float(struct.unpack('>H', status[13:15])[0])
    freqMkr2 = float(struct.unpack('>H', status[15:17])[0])

    # Limit
    limit = float(struct.unpack('>H', status[17:19])[0])

    # Distances
    distStart = float(struct.unpack('>I', status[19:23])[0])
    distStop = float(struct.unpack('>I', status[23:27])[0])

    # Distance markers
    distMkr1 = float(struct.unpack('>H', status[27:29])[0])
    distMkr2 = float(struct.unpack('>H', status[29:31])[0])

    # Propogation velocity
    propVel = float(struct.unpack('>I', status[31:35])[0])

    # Cable loss
    cableLoss = float(struct.unpack('>I', status[35:39])[0])

    # Parse status byte 1
    statusByte1 = status[39]
    limitOn = statusByte1 & (1 << 0)
    mkr1On = statusByte1 & (1 << 1)
    mkr2On = statusByte1 & (1 << 2)
    limitBeep = statusByte1 & (1 << 3)
    rlOrSWR = statusByte1 & (1 << 4)
    wdTimer = statusByte1 & (1 << 5)
    singleSweep = statusByte1 & (1 << 6)

    # Parse status byte 2
    statusByte2 = status[40]
    fixedCW = statusByte2 & (1 << 0)
    keyLock = statusByte2 & (1 << 1)
    backlight = statusByte2 & (1 << 2)
    measUnit = statusByte2 & (1 << 3)
    calOn = statusByte2 & (1 << 4)
    printer = statusByte2 & 0b11100000

    # Parse status byte 3 (not supported below FW 3.00)
    statusByte3 = status[41]
    windowMode = statusByte3 & 0b00000011
    measMode = statusByte3 & 0b00110000

    # Serial port echo
    echo = status[42]

    # Send recall sweep command
    data = bytearray()
    data.append(0x11)
    data.append(idx)
    sendBytes(data)

    # Wait for 604 bytes
    resp = port.read(604)

    # Print the debug
    if verb:
        print("Got: {}".format(resp))

    # Verify we got all the bytes
    numBytes = struct.unpack('>H', resp[0:2])[0]
    if numBytes != 602:
        print("Error: did not get expected sweep data")
        return
    
    # Get model & SW version
    model = resp[4:11].decode('ascii')
    soft = resp[11:15].decode('ascii')
    print("Model: {}\nSW: {}".format(model, soft))

    # Get domain of sweep
    domain = resp[39]
    if domain == 1:
        print("TDR sweep")
    else:
        print("Frequency sweep")

    # Get frequency information
    freqStart = float(struct.unpack('>I', resp[40:44])[0])
    freqStop = float(struct.unpack('>I', resp[44:48])[0])
    freqStep = float(struct.unpack('>I', resp[48:52])[0])
    print("Sweep start: {} Hz, stop: {} Hz, step: {} Hz".format(freqStart, freqStop, freqStep))

    # Get scale information
    scaleStart = float(struct.unpack('>H', resp[52:54])[0])
    scaleStop = float(struct.unpack('>H', resp[54:56])[0])
    print("Sweep scale start: {}, stop {}".format(scaleStart, scaleStop))

    # Get limit
    limit = float(struct.unpack('>H', resp[60:62])[0])
    print("Limit: {}".format(limit))

    # Get actual data (130 points, 4 bytes per point)
    dataBytes = resp[84:604]
    dataArray = []
    for i in range(0,130):
        idx = 4*i
        gamma = float(struct.unpack('>H', dataBytes[idx:idx+2])[0])/1000
        phase = float(struct.unpack('>h', dataBytes[idx+2:idx+4])[0])/10
        dataArray.append([gamma, phase])

    # Parse & save data
    if rlOrSWR:
        mode = 'RL'
    else:
        mode = 'SWR'

    parseData(dataArray, freqStart, freqStop, scaleStart, scaleStop, mode, show=plot, file=file)


def parseData(data, freqStart, freqStop, scaleStart, scaleStop, mode="RL", show=False, file=None):
    """
    Parse gamma/phase data into RL or SWR and display/save to file

    Args:
        data (list): gamma/phase array
        freqStart (uint32): start frequency in Hz
        freqStop (uint32): stop frequency in Hz
        scaleStart (uint16): start scale
        scaleStop (uint16): stop scale
        mode (str, optional): Return loss, Insertion Loss, SWR. Defaults to "RL".
        show (bool): whether to show the matplotlib plot
        file (string): whether to save to a file. Defaults to None
    """

    # Calculate frequency bins
    freqs = numpy.arange(freqStart, freqStop, (freqStop - freqStart) / len(data))

    # Convert data to numpy arrays
    gamma = numpy.array([i[0] for i in data])
    phase = numpy.array([i[1] for i in data])

    # Calculate RL in dB
    returnLoss = 20 * numpy.log10(gamma)

    # Calculate SWR, and handle divide by zero as max SWR
    swr = numpy.divide(1 + gamma, 1 - gamma, out = (numpy.ones_like(gamma) * 65535), where=(1-gamma)!=0)

    # Save to CSV
    if file:
        # get timestamp
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S %Z")
        # combine data
        combined = numpy.vstack((freqs, returnLoss, phase, swr)).T
        # save
        numpy.savetxt(file, combined, delimiter=',', header="W3AXL S113 Sweep Data,\n{}\nFreq (MHz), Return Loss (dB), Phase (deg), SWR".format(timestamp), comments='')
        print("Saved data to {}".format(file))

    # Show plot if specified
    if show:
    
        # Setup plot params
        plt.style.use('dark_background')
        fig, ax1 = plt.subplots()
        ax1.set_xlabel("Frequency (Hz)")

        # Our graph depends on measurement format
        if mode == "RL":
            yBottom = float(scaleStop) / 1000.0
            yTop = float(scaleStart) / 1000.0
            data1 = returnLoss
            data2 = phase
            ax1.set_ylabel("Return Loss (dB)")
            ax1.plot(freqs, data1)
            ax1.set_ylim(yBottom, yTop)
            ax1.margins(0)
            ax2 = ax1.twinx()
            ax2.set_ylabel("Phase (Deg)")
            ax2.plot(freqs, data2)
            ax2.margins(0)

        elif mode == "SWR":
            yBottom = float(scaleStart) / 1000.0
            yTop = float(scaleStop) / 1000.0
            data1 = swr
            data2 = None
            ax1.set_ylabel("SWR")
            ax1.plot(freqs, data1)
            ax1.set_ylim(yTop, yBottom)
            ax1.margins(0)

        # Common axis coloring
        for ax in fig.get_axes():
            ax.tick_params(color='lightgrey')
            for spine in ax.spines.values():
                spine.set_edgecolor('lightgrey')

        fig.tight_layout()
        plt.grid(color='grey', linestyle='-.', linewidth=0.5)
        plt.show()

# Main runtime
if __name__ == "__main__":
    # Connect and setup
    print("Connecting...")
    connect()
    print("Connected!")
    setupSystem(cwMode=True, keyLock=False, backlight=True, cal=True)

    # Do something based on command line switches
    if recall != None:
        recallData(recall)
        print("Done!")

    print("Disconnecting...")
    disconnect()