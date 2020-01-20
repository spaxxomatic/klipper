from Tkinter import *

import tkMessageBox
import serial
import ConfigParser
from thread import start_new_thread
import time
import dromacros as macros
import os
import pickle
import signal
import sys

def signal_handler(sig, frame):
        print('Ctrl+C! Will exit')
        on_closing()
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

from  spaxxpos.poslib import LinearPositionComm

myfolder = os.path.dirname(os.path.realpath(__file__))
staticfolder = os.path.join(myfolder, "static")

configfile = os.path.join(myfolder,"dro.ini")
os.chdir(myfolder)
print "Reading config file %s"%configfile
config = ConfigParser.ConfigParser()
config.read(configfile)

root = Tk()
#root.overrideredirect(True) #hides the title bar
#root.geometry("{0}x{1}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))
from drivecontrol import xmotor, ymotor
xdisplay = StringVar()
ydisplay = StringVar()
rpm_display = StringVar()
wdisplay = StringVar()
xerror = StringVar()
yerror = StringVar()

bdisplay = StringVar()
rpmdisplay = StringVar()
root.coords_display = StringVar()
debugstr = StringVar()
xstep, ystep, rpm_sensor_pin = None, None, None

try:
    xstep = float(config.get('RESOLUTION', 'x_step_size'))
    ystep = float(config.get('RESOLUTION', 'y_step_size'))
    rpm_sensor_pin = int(config.get('GENERAL', 'rpm_sensor_pin'))
    #zstep = float(config.get('RESOLUTION', 'z_step_size'))
except Exception, e:
    print "Invalid config file: %s"%(str(e))
    exit(1)

class Storage(list): pass
class Display:pass #forward declaration
state_persist_file = 'state.pkl'

def save_state():
    persist_state = {
        'xval_corr':display.xval_corr, 
        'yval_corr':display.yval_corr,
        'xmotor_state':xmotor.get_state(),
        'ymotor_state':ymotor.get_state()
    }
    with open(state_persist_file, 'w') as file:
        pickle.dump(persist_state, file)

persist_state = {}
def load_state():
    global persist_state
    try:
        with open(state_persist_file, 'r') as file:
            persist_state = pickle.load(file)
            if persist_state:
                if persist_state.has_key('xmotor_state'):
                    xmotor.set_state(persist_state['xmotor_state'])
                if persist_state.has_key('ymotor_state'):
                    ymotor.set_state(persist_state['ymotor_state'])
    except Exception, e:
        print "Could not load saved state: %s"%str(e)

def on_closing():
    #if tkMessageBox.askokcancel("Quit", "Do you want to quit?"):
    #save state
    save_state()
    Display.exit = 1
    time.sleep(1) #the dro reading thread must end
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

class Display():
    axis_stat = None
    def __init__(self):
        self.xval_corr = persist_state.get('xval_corr', 0)
        self.yval_corr = persist_state.get('yval_corr', 0)
    xstorage = Storage()
    ystorage = Storage()
    rpm = 0
    #zstorage = Storage()
    exit = 0 #flag for exit
    def refresh_threadfunc(self):
        while self.exit == 0:
            self.refresh()
            
    def refresh_dro(self):
        rpm_display.set(self.rpm)
        stat = self.axis_stat
        if stat:
            xdisplay.set("%.3f"%(stat.xposition - self.xval_corr))
            ydisplay.set("%.3f"%(stat.yposition - self.yval_corr))
            if stat.xerror > 0:
                xerror.set(SENSOR_ERRORS.get(stat.xerror, 'UNKNOWN ERR'))
            if stat.yerror > 0:
                yerror.set(SENSOR_ERRORS.get(stat.yerror, 'UNKNOWN ERR'))
            #zdisplay.set("%.3f"%(zstep*(self.zval - self.zval_corr)/ 1000))
                
    def history(self):
        self.xval_corr = self.xstorage.pull()
    def zeroX(self): 
        self.saveX()
        self.xval_corr = self.axis_stat.xposition
    def zeroY(self): 
        self.saveY()
        self.yval_corr = self.axis_stat.yposition
    def saveX(self): 
        self.xstorage.append(self.axis_stat.xposition)
    def saveY(self): 
        self.ystorage.append(self.axis_stat.yposition)
        
SENSOR_ERRORS = {
   #enum {
   #   OK,
   #   WARN_LIN_OR_COF,
   #   ERROR_MAG_FIELD_LOW,
   #   ERROR_NO_MAG_FIELD,
   #   ERROR_PARITY_ERROR,
   #   ERROR_OCF,
   #   ERROR_UNKNOWN
   # };
    1:'Lin warn',
    2:'Low mag field',
    3:'No mag field',
    4:'Parity error',
    5:'OCF error',
    6:'Unknown',
}

load_state()    
display = Display()

class Settings():
    def __init__(self, parent):
        self.parent = parent
        self.window = Toplevel(parent)
        self.window.geometry('+600+600')
        
    def open(self):
        e = Entry(self.window)
        e.pack()
        e.focus_set()
        self.window.transient(root)
        self.window.grab_set()
        self.parent.wait_window(self.window)
        #self.window.wm_title("Settings")
        #l = Label(self.window, text="This is window" )
        #l.pack(side="top", fill="both", expand=True, padx=100, pady=100)

keymap = {
    '<F1>':xmotor.inc_jog_speed,
    '<F2>':xmotor.dec_jog_speed,
    '<F5>':ymotor.inc_jog_speed,
    '<F6>':ymotor.dec_jog_speed,
}

def KeyboardEvent(event):
    if event.keysym_num > 0 and event.keysym_num < 60000:
        print('This is a printable key. The character is: %r keysym: %r' % \
            (event.char, event.keysym_num))
    else:
#269025041
#269025041
#269025043
#269025046
#269025047
    
        print('This key is unprintable. The character is: %r keysym: %r' % \
            (event.char, event.keysym_num))
            
class Application(Frame):
    def __init__(self, master, pos_display):
        self.master=master
        Frame.__init__(self, master)
        self.width, self.height = master.winfo_screenwidth(), master.winfo_screenheight()
        self._geom="%ix%i+0+0"%(self.width, self.height)
        self._thisrow = 0
        self.grid(row=2)
        self.pos_display = pos_display
        self.createWidgets()
        pad=3
        master.geometry("{0}x{1}+0+0".format(
            master.winfo_screenwidth()-pad, master.winfo_screenheight()-pad))
        master.bind('<Escape>',self.toggle_geom)        
        for k,v in keymap.iteritems():
            master.bind(k,v)
    
    def toggle_geom(self,event):
        geom=self.master.winfo_geometry()
        self.master.geometry(self._geom)
        self._geom=geom
    
    def settings(self):
        Settings(self).open()
    
    def histframe(self):
        return Frame(self.HISTORYFRAME, height=30, bd=1, relief=SUNKEN)
        
    def update_pos(self):
        self.master.after(60, self.update_pos)
        self.pos_display.refresh_dro()
        
    def nextrow(self):
        self._thisrow += 1
        return self._thisrow
    
    def place_next(self, obj):
        row = self.nextrow()
        if isinstance(obj, (list, tuple)):
            for idx, o in enumerate(obj): 
                o.grid(row=row, column = idx)
        else:
            obj.grid(row=row)
        return obj
    
    def create_axis_digitdisplay(self, axis, parent, var_position, var_errtxt, zero_cmd, abs_cmd):
        f = Frame(parent)
        Label(f, textvariable=var_errtxt, bg='#efe', relief=FLAT, height=1, width=10, font = "Calibri 16").pack({"side": "left"})
        Label(f, text=axis, bg='#efe', relief=FLAT, height=1, width=10, font = "Calibri 32").pack({"side": "left"})
        Button(f, text="Zero", command=zero_cmd).pack({"side": "right"})
        Button(f, text="Abs/Rel", command=abs_cmd).pack({"side": "right"})
        Label(f, textvariable=var_position, bg='#efe', relief=FLAT, height=1, width=10, font = "Arial 32 ").pack({"side": "left"})
        f.pack()
    
    def create_macro_button(self, imagefile, handler):
        img = PhotoImage(file=os.path.join(staticfolder, imagefile))
        btn = Button(self.MACROSFRAME, compound=LEFT, image=img, command=handler)
        btn.image = img
        btn.pack({"side": "left"})
        

        
    def createWidgets(self):
        self.MENUFRAME = Frame(root,  bg='#efe', width=self.width, height=30)
        
        self.QUIT = Button(self.MENUFRAME, text='QUIT', fg='red', command=on_closing).grid(row=0, column=0)
        self.Settings = Button(self.MENUFRAME, text='Settings', fg='green', command=self.settings).grid(row=0, column=1)
        self.place_next(self.MENUFRAME)
        
        self.dbg = Label(root, textvariable=debugstr, relief=FLAT, height=1, width=70, font="Arial 10 bold")
        self.dbg.grid(row=self.nextrow())
        self.RPMFRAME = Frame(root, width=self.width/2, height=30)
        self.HISTORYFRAME = Frame(root, width=self.width/2, height=30)
        
        Label(self.HISTORYFRAME, text="XMEM", bg='#efe',  height=1,  bd=1, width=12, font = "Calibri 12 bold").grid(row=0)
        #self.HISTORYFRAME_X = self.histframe().grid(row=0, column=0)
        
        Label(self.HISTORYFRAME, text="YMEM", bg='#efe',  height=1,  bd=1, width=12, font = "Calibri 12 bold").grid(row=1)
        #self.HISTORYFRAME_Y = self.histframe().grid(row=0, column=1)

        Label(self.RPMFRAME, text="RPM", bg='#efe',  height=1,  bd=1, width=4, font = "Calibri 12 bold").grid(column=0, row=0)
        Label(self.RPMFRAME, textvariable=rpm_display, bg='#efe',  height=1,  bd=1, width=12, font = "Calibri 12 bold").grid(column=1, row=0)
        tmp = Entry(self.RPMFRAME, text="Press key", bg='#efe',  bd=1, width=12, font = "Calibri 12 bold")
        tmp.bind('<KeyPress>', KeyboardEvent)            
        tmp.grid(column=2, row=0)
        
        #Label(self.HISTORYFRAME, text="ZYMEM", bg='#efe',  height=1,  width=12, font = "Calibri 12 bold").grid(row=2)
        #self.HISTORYFRAME_Z = self.histframe().grid(row=self.nextrow(), column=1) 
        #Label(self.HISTORYFRAME_Z, text="ZMEM", bg='#efe',  height=1, width=12, font = "Calibri 12 bold").pack(side=LEFT)
        self.place_next((self.RPMFRAME, self.HISTORYFRAME));
        
        self.DISPLAYFRAME = Frame(root, width=self.width)
        self.place_next(self.DISPLAYFRAME)
        
        #spacer
        self.place_next(Frame(root, bg='#efe',  height=20))
        
        self.TOOLSFRAME = Frame(root, bg='#efe',  height=220, width=self.width)
        self.TOOLSFRAME.columnconfigure(0, weight=1)
        self.place_next(self.TOOLSFRAME)
        
        self.XTOOLSFRAME = Frame(self.TOOLSFRAME, bg='#efe', height=220, width=self.width/2)
        self.YTOOLSFRAME = Frame(self.TOOLSFRAME, bg='#efe',  height=220, width=self.width/2)
        
        Label(self.XTOOLSFRAME, text="X Jog speed", font = "Calibri 16").pack({"side": "left"})
        Entry(self.XTOOLSFRAME, font = "Calibri 16", textvariable=xmotor.jogspeeddisplay, width=6).pack({"side": "left"})
        Button(self.XTOOLSFRAME, font = "Calibri 16", compound=LEFT, text="X+", command=xmotor.jogup).pack({"side": "left"})
        Button(self.XTOOLSFRAME, font = "Calibri 16", compound=LEFT, text="X-", command=xmotor.jogdn).pack({"side": "left"})
        
        Label(self.YTOOLSFRAME, font = "Calibri 16", text="Y Jog speed").pack({"side": "left"})
        Entry(self.YTOOLSFRAME, font = "Calibri 16", textvariable=ymotor.jogspeeddisplay, width=6).pack({"side": "left"})
        Button(self.YTOOLSFRAME, font = "Calibri 16", compound=LEFT, text="Y+", command=ymotor.jogup).pack({"side": "left"})
        Button(self.YTOOLSFRAME, font = "Calibri 16", compound=LEFT, text="Y-", command=ymotor.jogdn).pack({"side": "left"})
        
        self.create_axis_digitdisplay('X', self.DISPLAYFRAME, xdisplay, xerror, display.zeroX, display.saveX)
        self.create_axis_digitdisplay('Y', self.DISPLAYFRAME, ydisplay, yerror, display.zeroY, display.saveY)
        #self.create_axis_digitdisplay('Z', self.DISPLAYFRAME, zdisplay)
        self.XTOOLSFRAME.grid(row=0, column=0)
        self.YTOOLSFRAME.grid(row=0, column=1)
        
        #spacer
        self.place_next(Frame(root, bg='#efe',  height=20))
        
        self.MACROSFRAME=Frame(root, bg='#efe',  height=20)
        self.create_macro_button("face.gif", macros.TurningFuncs.face)
        self.create_macro_button("turn.gif", macros.TurningFuncs.turn)
        self.create_macro_button("thread.gif", macros.TurningFuncs.thread)
        
        canvas_height = 200
        c = self.canvas = Canvas(root, width=self.width, height=canvas_height)
        c.height = canvas_height
        c.width = self.width
        c.config(relief=GROOVE, bg="#aa9")
        c.grid(row=self.nextrow())
        macros.canvas = c
        macros.root = root
        #self.drillseries_rect_icon = PhotoImage(file="rect_drills.gif")
        #self.drillseries_circ_icon = PhotoImage(file="circle_drills.gif")
        #b = Button(self.TOOLSFRAME, compound=LEFT, image=self.drillseries_rect_icon, command=macros.Funcs.drillseries_rect).pack({"side": "left"})
        #b = Button(self.TOOLSFRAME, compound=LEFT, image=self.drillseries_circ_icon, command=macros.Funcs.drillseries_circ).pack({"side": "left"})
        file = os.path.join(staticfolder, "face.gif")
        #print os.path.exists(file)
        self.place_next(self.MACROSFRAME)
        
        self.cd = Label(root, textvariable=root.coords_display, relief=FLAT, height=1, width=70, font="Arial 10")
        self.cd.grid(row=self.nextrow())
    
        self.update_pos()

        

app = Application(master=root, pos_display=display)

title = "Spaxx Lathe DRO"
app.master.title(title)

class PosDataRefresh():    

    def __init__(self, display):
        self.port = config.get('GENERAL', 'comport')
        self.baudrate = config.get('GENERAL', 'baudrate')
        self.display = display

    def get_sensors(self):
        with LinearPositionComm(self.port, self.baudrate, rpm_sensor_pin) as comm:
            while not Display.exit:
                self.display.axis_stat = comm.pos_receiver_lib.get_axis_stat()
                self.display.rpm_display = comm.pos_receiver_lib.get_rpm()
                time.sleep(0.2)

try:
    start_new_thread(PosDataRefresh(display).get_sensors, ())
except Exception, e:
    debugstr.set(`e`)
    display.globalerr = 'INIT ERR'
        
app.mainloop()