import serial
from serial.tools import list_ports
from matplotlib import pyplot
import numpy as np

port_list = list_ports.grep('USB')
port = next(port_list).device #Will throw StopIteration if there are no ports


with serial.Serial(port, 115200, timeout=1) as ser:
    fig, ax = pyplot.subplots()
    angles=np.ndarray(0)
    distances=np.ndarray(0)
    while True:
        b = ser.read(47)
        def btoi(x): #Converts two bytes to integer
            return int.from_bytes(b[x:x+2], 'little')
        assert len(b) == 47, f"Only {len(b)} bytes received"
        if b[-1] != 0x54:
            b = ser.read_until(b'\x54') #Find the first header
            time_prev = btoi(len(b)-4)
            continue
        speed = btoi(1)
        start_angle = btoi(3)
        end_angle = btoi(41) 
        if end_angle < start_angle:
            angles *= np.pi/18000
            x = np.sin(angles) * distances
            y = np.cos(angles) * distances
            ax.clear()
            ax.plot(x,y, '.')
            fig.show(False)
            pyplot.pause(0.01)
            end_angle += 36000
            angles=np.ndarray(0)
            distances=np.ndarray(0)
        angles = np.append(angles, np.linspace(start_angle, end_angle, num=12))
        distances = np.append( distances, [btoi(x) for x in range(5,41,3)] )
        timestamp = btoi(43)
        dt = timestamp - time_prev
        confidence = [b[x] for x in range(7,41,3)]






