import serial
from serial.tools import list_ports
from matplotlib import pyplot
import numpy as np

port_list = list_ports.grep('USB')
port = next(port_list).device #Will throw StopIteration if there are no ports


with serial.Serial(port, 115200, timeout=1) as ser:
    fig, ax = pyplot.subplots()
    a=np.ndarray(0)
    d=np.ndarray(0)
    c=np.ndarray(0)
    max_lim = 260
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
            a *= np.pi/18000
            if True:
                x = np.sin(a) * d
                y = np.cos(a) * d
            else:
                x = np.sin(a) * c
                y = np.cos(a) * c
            ax.clear()
            ax.plot(x,y)
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            max_lim = max(map(abs, (max_lim,0) + xlim + ylim))
            ax.set_xlim((-max_lim, max_lim))
            ax.set_ylim((-max_lim, max_lim))
            fig.show(False)
            pyplot.pause(0.01)
            end_angle += 36000
            a=np.ndarray(0)
            d=np.ndarray(0)
            c=np.ndarray(0)
        a = np.append( a, np.linspace(start_angle, end_angle, num=12))
        d = np.append( d, [btoi(x) for x in range(5,41,3)] )
        c = np.append( c, [b[x] for x in range(7,41,3)])
        timestamp = btoi(43)
        dt = timestamp - time_prev






