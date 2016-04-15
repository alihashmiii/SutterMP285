# sutterMP285 : A python class for using the Sutter MP-285 positioner

# SUTTER285 implements a class for working with a Sutter MP-285
# micropositioner. The sutter must be connected with a serial cable.

# this class uses the python "serial" package which allows for
# with the serial devices through 'write' and 'read'.
# The communication properties (Baudrate, Terminator, etc.) are
# set when invoking the serial object with the serial. Serial(..) (1105,
# see Sutter Reference manual p23).

# -------- Methods ---------

# create the object. The object is opened with serial. Serial and the connection
# is tested to verify that the Sutter is responding.
#   obj = sutterMP285()

# Update the position display on the instrument panel (VFD)
#   updatePanel()

# Get the status information (step multiplier, velocity, resolution)
#   [stepMult, currentVelocity, vScaleFactor] = getStatus()

# Get the current absolute position in um
#   xyz_um = getPosition()

# Set the move velocity in steps/sec. vScaleFactor = 10|50 (default 10)/
#   setVelocity(velocity, vScaleFactor)

# Move to a specified position in um [x y z]. Returns the elapsed time
# for the move (command sent and acknowledged) in seconds.
#   moveTime = gotoPosition(xyz)

# Set the current position to be the new origin (0,0,0)
#   setOrigin()

# Reset the instrument
#   Reset()

# Terminate the connection
#   __del__()

# Properties:
#   verbose - The level of messages displayed (0 or 1). Default 1.

####################
####################
# ------------------------- Example code ------------------------

# >> import serial
# >> sutter = sutterMP285()
#    Serial<id=0x4548370, open=True>(port = 'COM1', baudrate = 9600, bytesize=8, parity='N', stopbits=1,timeout=30,xonxoff=False,rtscts=False, dsrdtr=False)
#    sutterMP285: get status info
#    (64, 0, 2, 4, 7, 0, 99, 0, 99, 0, 20, 0, 136, 19, 1, 120, 112, 23, 16, 39, 80, 0, 0, 0, 25, 0, 4, 0, 200, 0, 84, 1)
#    step_mul (usteps/um): 25
#    xspeed" [velocity] (usteps/sec): 200
#    velocity scale factor (usteps/step): 10
#    sutterMP285 ready
# >> pos = sutter.getPosition()
#    sutterMP285 : Stage Position
#    X : 3258.64 um
#    Y : 5561.32 um
#    Z : 12482.5 um
# >> posnew = (pos[0]+10., pos[1]+10., pos[2]+10.)
# >> sutter.gotoPosition(posmew)
#    sutterMP285 : Sutter move completed in (0.24 sec)
# >> status = sutter.getStatus()
#    sutterMP285: get status info
#    (64, 0, 2, 4, 7, 0, 99, 0, 99, 0, 20, 0, 136, 19, 1, 120, 112, 23, 16, 39, 80, 0, 0, 0, 25, 0, 4, 0, 200, 0, 84, 1)
#    step_mul (usteps/um): 25
#    xspeed" [velocity] (usteps/sec): 200
#    velocity scale factor (usteps/step): 10
# >> del sutter

########################## CODE ##################

import serial
import struct
import time
import sys
from numpy import *

class sutterMP285:
    'Class which allows interaction with the Sutter Manipulator 285'
    def __init__(self):
        self.verbose = 1. # level of messages
        self.timeOut = 30 # timeout in sec
        # initialize serial connection to controller
        try:
            self.ser = serial.Serial(port='COM3', baudrate=9600, bytesize = serial.EIGHTBITS, parity=serial.PARITY_NONE,stopbits= serial.STOPBITS_ONE, timeout= self.timeOut)
            self.connected = 1
            if self.verbose:
                print self.ser
        except serial.SerialException:
            print 'No connection to Sutter MP-285 could be established!'
            sys.exit(1)

        # set move velocity to 200
        self.setVelocity(200,10)
        self.updatePanel() # update controller panel
        (stepM,currentV, vScaleF) = self.getStatus()
        if currentV == 200:
            print 'sutterMP285 ready'
        else:
            print 'sutterMP285: WARNING sutter did not respond at startup.'

    # destructor
    def __del__(self):
        self.ser.close()
        if self.verbose:
            print 'Connection to Sutter MP285 terminated'

    # gives the position of the stage
    def getPosition(self):
        # send the command to get position
        self.ser.write('c\r')
        # read position from the controller
        xyzb = self.ser.read(13)
        # convert bytes into 'signed long' numbers
        xyz_um = array(struct.unpack('lll',xyzb[:12]))/self.stepMult

        if self.verbose:
            print 'sutterMP285 : Stage position '
            print ' X: %g um \n Y: %g um\n Z: %g um' %(xyz_um[0],xyz_um[1],xyz_um[2])

        return xyz_um

    # Moves the three axess to specified location.
    def gotoPosition(self,pos):
        if len(pos) != 3:
            print 'the length of the position argument has to be 3'
            sys.exit(1)
        xyzb = struct.pack('lll',int(pos[0]*self.stepMult),int(pos[1]*self.stepMult), int(pos[2]*self.stepMult)) # convert integer values into bytes
        startt = time.time() # start timer
        self.ser.write('m' + xyzb + '\r') # send position to controller; add the 'm' and the CR to create the move command
        cr = []
        cr = self.ser.read(1) # read carriage return and ignore
        endt = time.time() # stop timer
        if len(cr) == 0:
            print 'Sutter did not finish moving before timeout (%d sec).' % self.timeOut
        else:
            print 'sutterMP285: Sutter move completed in (%.2f sec)' %(endt-startt)
            
    # this function changes the velocity of the Sutter motions
    def setVelocity(self, Vel, vScalF = 10):
        # change velocity command 'V'xxCR where xx = unsigned short (16bit) int velocity
        # set by bits 14 to 0, and bit 15 indicates ustep resolution 0=10, 1=50 uSteps/step
        # V is ascii 86
        # convert velocity into unsigned short - 2-byte - integer
        velb = struct.pack('H',int(Vel))
        # change last bit of 2nd byte to 1 for ustep resolution = 50
        if vScalF == 50:
            velb2 = double(struct.unpack('B',velb[1])) + 128
            velb = velb[0] + struct.pack('B', velb2)
        self.ser.write('V'+velb+'\r')
        self.ser.read(1)

    # Update Panel
    # causes the Sutter to display the XYZ info on the front panel
    def updatePanel(self):
        self.ser.write('n\r') #Sutter replies with a CR
        self.ser.read(1) # read and ignore the carriage return

    # Set Origin
    # sets the origin of the coordinate system to the current position
    def setOrigin(self):
        self.ser.write('o\r') # Sutter replies with a CR
        self.ser.read(1) # read and ignore the carriage return

    # Reset controller
    def sendReset(self):
        print 'resetting connection'
        self.ser.write('r\r') # Sutter does not reply

    # Queries the status of the controller
    def getStatus(self):
        if self.verbose:
            print 'sutterMP285: get status info'
        self.ser.write('s\r') # send status command
        rrr = self.ser.read(32) # read retrun of 32 bytes without carriage return
        self.ser.read(1) # read and ignore the carriage return
        rrr
        statusbytes = struct.unpack(32*'B', rrr)
        print statusbytes
        # the value of STEP_MUL ("Multilplier yields msteps/nm") is at bytes 25 & 26
        self.stepMult = double(statusbytes[25])*256 + double(statusbytes[24])

        # the values of "XSPEED" and scale factor is at bytes 29 & 30
        if statusbytes[29] > 127:
            self.vScaleFactor = 50
        else:
            self.vScaleFactor = 10
        self.currentVelocity = double(127 & statusbytes[29])*256+double(statusbytes[28])
        if self.verbose:
            print 'step_mul (usteps/u,): %g' % self.stepMult
            print 'xspeed" [velocity] (usteps/sec): %g' % self.currentVelocity
            print 'velocity scale factor (usteps/step): %g' % self.vScaleFactor
        return (self.stepMult,self.currentVelocity,self.vScaleFactor)

########################## below is code for the GUI ##########################

from Tkinter import *
root = Tk()
import serial
import operator

# creating an object

MP285 = sutterMP285();

# save positions in a list
listpossave = [];
listpausesave = [];

# function definitions
def savepos():
    entries = [entry1, entry2, entry3];
    pause = float(entry4.get())
    listpausesave.append(pause)
    tup = ();
    for i in range(len(entries)):
        tup = tup + (float(entries[i].get()),)

    if len(listpossave) is 0:
        listpossave.append(tup);
    else:
        listpossave.append(tuple(map(operator.add, listpossave[-1], tup)))

    print listpossave ####
    textdisp()

def multirun():
    text_widget = Text(root)
    text_widget.grid(row = 16, column = 1)
    scrollbar = Scrollbar(root)
    scrollbar.grid(row=16, column=2)
    scrollbar.config(command = text_widget.yview)

    if listpossave == []:
        text_widget.insert('1.0', "the list is empty")        
    else:
        stri = "";
        for i in range(len(listpossave)):
            stri = 'processing step ' + str(i) + ' : ' + str(listpossave[i]) + ' um. pause: ' + str(listpausesave[i]) + ' seconds \n';
            text_widget.insert('1.0',stri)
            text_widget.update()
            MP285.gotoPosition(listpossave[i])
            time.sleep(listpausesave[i])
            if i == len(listpossave)-1:
                text_widget.insert('1.0', 'run completed \n')
                text_widget.update()
            
##    for i in range(len(listpossave)):
##        print 'processing step ' + str(i) + ' : ' + str(listpossave[i]) + ' um. pause: ' + str(listpausesave[i]) + ' seconds'
##        #MP285.gotoPosition(listpossave[i])
##        time.sleep(listpausesave[i])
##    print 'run completed'

def clearlist():
    global listpossave;
    global listpausesave;
    listpossave = [];
    listpausesave = [];
    textdisp()

def listinsert():
    if listpossave != []:
        j = int(entry5.get())
        entries = [entry1, entry2, entry3];
        tup = ()
        for i in range(len(entries)):
            tup = tup + (float(entries[i].get()),)
        
        listpossave.insert(j,tup)
        listpausesave.insert(j,float(entry4.get()))
        print listpossave
        print listpausesave
        textdisp()

def removeelem():
    j = int(entry5.get())
    if j < len(listpossave):
        del listpossave[j]
        del listpausesave[j]
        print listpossave ###
        textdisp()

def disconnect():
    MP285.__del__()
    stri = 'connection to Sutter MP285 has been terminated'
    textdisp2(stri)

def movearun():
    entries = [entry1, entry2, entry3];
    tupentries = ();

    for i in range(len(entries)):
        tupentries = tupentries + (float(entries[i].get()),)

    MP285.gotoPosition(tupentries)
    stri = 'the stage moved to [' + entry1.get() + ', ' + entry2.get() + ', ' + entry3.get() + ' ] um' 
    textdisp2(stri)
    
def setVel():
    #print int(entry6.get()),int(entry7.get())
    MP285.setVelocity(int(entry6.get()),int(entry7.get()))
    stri = 'Velocity : ' + entry6.get() + '\n' +'Scale factor : ' + entry7.get()
    textdisp2(stri)


def textdisp():
    text_widget = Text(root)

    scrollbar = Scrollbar(root)
    scrollbar.grid(row=16, column=2)

    if listpossave == []:
        text_widget.insert('1.0', "the list is empty")
        text_widget.grid(row = 16, column = 1)
        
    else:
        stri = "";
        for i in range(len(listpossave)):
            stri += 'step ' + str(i) + ' : ' + str(listpossave[i]) + ' ' + 'pause = ' + str(listpausesave[i]) +  " \n"
        text_widget.insert('1.0', stri)
        text_widget.grid(row = 16, column = 1)

    scrollbar.config(command = text_widget.yview)
           

def textdisp2(stri):
    text_widget = Text(root)
    text_widget.grid(row=16,column=1)
    text_widget.insert('1.0', str(stri))
    text_widget.update()  
    

def getposdisp():
    stri = MP285.getPosition()
    stri = '[x, y, z] = ' + str(stri)
    textdisp2(stri)

def setorig():
    MP285.setOrigin()
    stri = 'coordinates set to [0. , 0. , 0.]'
    textdisp2(stri)

def resetsutter():
    stri = 'Sutter has been reset .. '
    textdisp2(stri)
    MP285.sendReset()

def dispgetstat():
    stat = MP285.getStatus()
    stri = 'step_mul (usteps/um): ' + str(stat[0]) + '\n' + 'xspeed [velocity] (usteps/sec): ' + str(stat[1]) + '\n' + 'velocity scale factor (usteps/step): ' + str(stat[2])
    textdisp2(stri)
    
    
## initial label on the GUI
label_0 = Label(root, text = "Sutter MP285 Control Program", fg = "#0033cc", font = ("Helvetica",15))
label_0.grid(row = 0, column = 1)
label_0


class buttons(object):
    def __init__(self, root, text, func, fgcol,bgcol, row, col,entc):
        self.root = root
        self.text = text
        self.func = func
        self.fgcolour = fgcol
        self.bgcolour = bgcol
        self.r = row
        self.c = col
        self.entc = entc
    def createent(self):
        ent =Entry(self.root)
        ent.grid(row = self.r,column = self.entc)
        return ent
    def createbutton(self):
        button = Button(self.root, text = self.text, command = self.func, fg = self.fgcolour, bg=self.bgcolour)
        button.grid(row = self.r, column = self.c)
        return button

class Labels(object):
    def __init__(self, root, txt, r, c):
        self.root = root
        self.text = txt
        self.r = r
        self.c = c

    def label(self):
        item = Label(self.root, text = self.text)
        item.grid(row=self.r,sticky=E)

    def entry(self):
        item2= Entry(self.root)
        item2.grid(row = self.r, column = self.c)
        return item2


xpos = Labels(root, "x position (um)", 6,1)
xpos.label()
entry1 = xpos.entry()

ypos = Labels(root, "y position (um)", 7,1)
ypos.label()
entry2 = ypos.entry()

zpos = Labels(root, "z position (um)", 8,1)
zpos.label()
entry3 = zpos.entry()

pause = Labels(root, "pause (sec)", 9,1)
pause.label()
entry4 = pause.entry()

index = Labels(root, "index", 12,1)
index.label()
entry5 = index.entry()

velmagnitude = Labels(root, "velocity", 14,1)
velmagnitude.label()
entry6 = velmagnitude.entry()

scaleFactor = Labels(root,"scale-factor (10 or 50)",15,1)
scaleFactor.label()
entry7 = scaleFactor.entry()

getstat = buttons(root, "get Status", dispgetstat, 'pink','black',2,0,None) #
getstat.createbutton()

findpos = buttons(root, "get Position", getposdisp, 'pink','black',4,0,None) #
findpos.createbutton()

gotopos = buttons(root, "go to position", movearun, 'lightgreen','black',6,2,None)
gotopos.createbutton()

disconnect = buttons(root, "Disconnect link", disconnect, '#ff0000','black',2,1,None)
disconnect.createbutton()

origin = buttons(root, "set Origin", setorig, 'white','black',3,2,None) #
origin.createbutton()

reset = buttons(root, "Reset", resetsutter, '#ff1a1a','black',2,2,None) #
reset.createbutton()

setvelocity = buttons(root, "set velocity", setVel, '#000fff000','black',15,2,None)
setvelocity.createbutton()

updatepanel = buttons(root, "update panel", MP285.updatePanel, 'white','black',3,1,None) 
updatepanel.createbutton()

clearlistbutton = buttons(root, "clear list", clearlist, 'lightblue','black',8,2,None)
clearlistbutton.createbutton()

listshowbutton = buttons(root, "list show", textdisp, 'lightblue','black',8,3,None)
listshowbutton.createbutton()

savebutton = buttons(root, "Save", savepos, 'lightblue','black',10,1,None)
savebutton.createbutton()

runbutton = buttons(root, "Run", multirun, 'lightblue','black',10,2,None)
runbutton.createbutton()

insertelem = buttons(root, "insert",listinsert, 'orange', 'black',13,1,None)
insertelem.createbutton()

removeelement = buttons(root, "remove",removeelem, 'orange', 'black',13,2,None)
removeelement.createbutton()

root.mainloop()
