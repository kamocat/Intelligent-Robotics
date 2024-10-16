import serial
import queue
import threading
from serial.tools import list_ports
from matplotlib import pyplot
import numpy as np
import time


class Lidar(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self,name="LDS-02 data acquisition")
        port_list = list_ports.grep('USB')
        self.port = next(port_list).device #Will throw StopIteration if there are no ports
        self.q = queue.Queue()
        self.want_abort = False
        self.start()

    def abort(self):
        self.want_abort = True

    def run(self):
        with serial.Serial(self.port, 115200, timeout=1) as ser:
            a=np.ndarray(0)
            d=np.ndarray(0)
            c=np.ndarray(0)
            while not self.want_abort:
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
                    end_angle += 36000
                    send = True
                else:
                    send = False
                a = np.append( a, np.linspace(start_angle, end_angle, num=12))
                d = np.append( d, [btoi(x) for x in range(5,41,3)] )
                c = np.append( c, [b[x] for x in range(7,41,3)])
                timestamp = btoi(43)
                dt = timestamp - time_prev
                if send:
                    self.q.put({
                        'angles':a,
                        'distances':d,
                        'confidences':c,
                        'speed':speed,
                        'tend':timestamp,
                        })
                    a=np.ndarray(0)
                    d=np.ndarray(0)
                    c=np.ndarray(0)



class Plotter():
    def __init__(self):
        self.li = Lidar()
        self.want_abort = False
        self.lim = 260
        self.fig, self.ax = pyplot.subplots()
        self.run()

    def abort(self):
        self.want_abort = True
    
    def __del__(self):
        self.li.abort()
        self.abort()
    
    def get(self):
        data = self.li.q.get()
        a = data['angles'] * np.pi/18000
        d = data['distances'] * 0.1 #Scale to centimeters
        x = (-1 * np.cos(a)) * d
        y = np.sin(a) * d
        return {'x':x, 'y':y}

    def fix_limits(self):
        max_lim = self.lim
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        xlim = max(map(abs, xlim + ylim))
        if xlim > max_lim:
            max_lim = xlim
        elif xlim * 1.5 < max_lim:
            max_lim *= 0.95
        self.ax.set_xlim((-max_lim, max_lim))
        self.ax.set_ylim((-max_lim, max_lim))
        self.lim = max_lim

    def run(self):
        pd = self.get()
        while not self.want_abort:
            nd = self.get()
            self.ax.clear()
            l = min(len(pd['x']), len(nd['x']))
            for i in range(l):
                self.ax.plot([pd['x'][i],nd['x'][i]],[pd['y'][i],nd['y'][i]])
            self.fix_limits()
            self.fig.show(False)
            pyplot.pause(0.01)
            pd = nd

p = Plotter()
