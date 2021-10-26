import serial
import argparse
import datetime
import struct
import numpy

# Add arguments and query
parser = argparse.ArgumentParser()
parser.add_argument("-p","--port",metavar="COM1",help="Serial device for communication with S113")
parser.add_argument("-r","--recall",metavar="0",help="Data storage bin to query. 0 = current plot")
parser.add_argument("-v","--verbose",help="verbose logging",action="store_true")
parser.add_argument("-f","--csvfile",metavar="file.csv",help="Save data to CSV file")

# Parse arguments
args = parser.parse_args()

portNo = ""
recall = 0
verb = False
file = None

if not args.recall:
    print("Defaulting to recall index 0 (current data)")
else:
    recall = int(args.recall)

if not args.port:
    print("ERROR: No serial port specified!")
    exit(1)
else:
    portNo = args.port

if args.verbose:
    verb = True

if args.csvfile:
    file = args.csvfile

# Create serial port
port = serial.Serial(portNo, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=5)

# Functions

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

def recallData(idx):
    """
    Recall data from the S113 into the dataArray and then process it

    Args:
        idx (int): saved data index. 0 is current sweep on screen
    """
    
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
    freqStart = float(struct.unpack('>I', resp[40:44])[0])/1e6
    freqStop = float(struct.unpack('>I', resp[44:48])[0])/1e6
    freqStep = float(struct.unpack('>I', resp[48:52])[0])/1e3
    print("Sweep start: {} MHz, stop: {} MHz, step: {} kHz".format(freqStart, freqStop, freqStep))

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

    parseData(dataArray, freqStart, freqStop, scaleStart, scaleStop, mode)


def parseData(data, freqStart, freqStop, scaleStart, scaleStop, mode="RL"):

    # Calculate frequency bins
    freqs = numpy.arange(freqStart, freqStop, (freqStop - freqStart) / len(data))

    # Convert data to numpy arrays
    gamma = numpy.array([i[0] for i in data])
    phase = numpy.array([i[1] for i in data])

    # Calculate RL in dB and SWR
    returnLoss = 20 * numpy.log10(gamma)
    swr = (1 + gamma)/(1 - gamma)

    # Save to CSV
    if file:
        # get timestamp
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S %Z")
        # combine data
        combined = numpy.vstack((freqs, returnLoss, phase, swr)).T
        # save
        numpy.savetxt(file, combined, delimiter=',', header="W3AXL S113 Sweep Data,\n{}\nFreq (MHz), Return Loss (dB), Phase (deg), SWR".format(timestamp), comments='')

    print ("Done processing")

# Main runtime
if __name__ == "__main__":
    print("Connecting...")
    connect()
    setupSystem(cwMode=True, keyLock=True, backlight=True, cal=True)
    recallData(recall)
    print("Disconnecting...")
    disconnect()