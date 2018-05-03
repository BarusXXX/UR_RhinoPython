"""
yoUR - Python library for UR robots

This library was initialy developed at ETH Zurich in 2011 at Gramazio Kohler Research.
Since then it was used by students in bachelor, master and MAS levels.
Initial framework was given by Ralph Baertschi, Michael Knauss and Silvan Oesterle.
Considerable contribution was made by Dr. Jason Lim as part of his PhD dissertation 
'YOUR: Robot Programming Tools for Architectural Education' at ETH Zurich in 2016.
This version is used since 2018 at Aalto University in Helsinki and is maintained by Luka Piskorec.

DESCRIPTION

This module manages sending of data from the robot to the computer
"""


import Rhino
import System.Drawing
import System.Windows.Forms
from System.Drawing import *
from System.Windows.Forms import *

class ListenForm(Form):
    def __init__(self):
        self.__listen_callback = None
        self._listen = False
        self._id = 1
        self.InitializeComponent()

    def InitializeComponent(self):
        self._components = System.ComponentModel.Container()
        self._labelID = System.Windows.Forms.Label()
        self._numericID = System.Windows.Forms.NumericUpDown()
        self._radioBtn_Interval1 = System.Windows.Forms.RadioButton()
        self._radioBtn_Interval2 = System.Windows.Forms.RadioButton()
        self._radioBtn_Interval3 = System.Windows.Forms.RadioButton()
        self._buttonListen = System.Windows.Forms.Button()
        self._timer1 = System.Windows.Forms.Timer(self._components)
        self._numericID.BeginInit()
        self.SuspendLayout()
        self.SuspendLayout()
        # 
        # labelID
        # 
        self._labelID.Font = System.Drawing.Font("Microsoft Sans Serif", 9.75, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0)
        self._labelID.Location = System.Drawing.Point(12, 17)
        self._labelID.Name = "labelID"
        self._labelID.Size = System.Drawing.Size(65, 23)
        self._labelID.TabIndex = 0
        self._labelID.Text = "Robot ID:"
        # 
        # numericID
        # 
        self._numericID.Font = System.Drawing.Font("Microsoft Sans Serif", 9.75, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0)
        self._numericID.Location = System.Drawing.Point(83, 13)
        self._numericID.Maximum = System.Int32(3)
        self._numericID.Minimum = System.Int32(1)
        self._numericID.Name = "numericID"
        self._numericID.Size = System.Drawing.Size(42, 22)
        self._numericID.TabIndex = 1
        self._numericID.Value = System.Int32(1)
        self._numericID.ValueChanged += self.NumericIDValueChanged
        # 
        # radioBtn_Interval1
        # 
        self._radioBtn_Interval1.Location = System.Drawing.Point(225, 10)
        self._radioBtn_Interval1.Name = "radioBtn_Interval1"
        self._radioBtn_Interval1.Size = System.Drawing.Size(82, 18)
        self._radioBtn_Interval1.TabIndex = 5
        self._radioBtn_Interval1.TabStop = True
        self._radioBtn_Interval1.Text = "50 ms"
        self._radioBtn_Interval1.UseVisualStyleBackColor = True
        self._radioBtn_Interval1.CheckedChanged += self.RadioBtn_Interval1CheckedChanged
        # 
        # radioBtn_Interval2
        # 
        self._radioBtn_Interval2.Location = System.Drawing.Point(225, 30)
        self._radioBtn_Interval2.Name = "radioBtn_Interval2"
        self._radioBtn_Interval2.Size = System.Drawing.Size(82, 18)
        self._radioBtn_Interval2.TabIndex = 6
        self._radioBtn_Interval2.TabStop = True
        self._radioBtn_Interval2.Text = "1000 ms"
        self._radioBtn_Interval2.UseVisualStyleBackColor = True
        self._radioBtn_Interval2.Checked = True
        self._radioBtn_Interval2.CheckedChanged += self.RadioBtn_Interval2CheckedChanged
        # 
        # radioBtn_Interval3
        # 
        self._radioBtn_Interval3.Location = System.Drawing.Point(225, 50)
        self._radioBtn_Interval3.Name = "radioBtn_Interval3"
        self._radioBtn_Interval3.Size = System.Drawing.Size(82, 18)
        self._radioBtn_Interval3.TabIndex = 7
        self._radioBtn_Interval3.TabStop = True
        self._radioBtn_Interval3.Text = "10000 ms"
        self._radioBtn_Interval3.UseVisualStyleBackColor = True
        self._radioBtn_Interval3.CheckedChanged += self.RadioBtn_Interval3CheckedChanged
        # 
        # buttonListen
        # 
        self._buttonListen.Font = System.Drawing.Font("Microsoft Sans Serif", 9.75, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, 0)
        self._buttonListen.Location = System.Drawing.Point(142, 11)
        self._buttonListen.Name = "buttonListen"
        self._buttonListen.Size = System.Drawing.Size(70, 55)
        self._buttonListen.TabIndex = 3
        self._buttonListen.Text = "Listen"
        self._buttonListen.BackColor = System.Drawing.Color.White
        self._buttonListen.UseVisualStyleBackColor = False
        self._buttonListen.Click += self.ButtonListenClick
        # 
        # timer
        # 
        self._timer1.Enabled = True
        self._timer1.Interval = 1000
        self._timer1.Tick += self.OnTimerTick
        #
        #register closing event
        #
        self.FormClosed += self.MyClosedHandler
        # 
        # MainForm
        # 
        self.ClientSize = System.Drawing.Size(325, 74)
        self.Controls.Add(self._buttonListen)
        self.Controls.Add(self._numericID)
        self.Controls.Add(self._labelID)
        self.Controls.Add(self._radioBtn_Interval3)
        self.Controls.Add(self._radioBtn_Interval2)
        self.Controls.Add(self._radioBtn_Interval1)
        self.Name = "MainForm"
        self.Text = "ListenerLite"
        self.MaximizeBox = False
        self._numericID.EndInit()
        self.ResumeLayout(False)
        self.PerformLayout()

    # Set Callbacks
    def SetListenCallback(self, listen_object):
        self.__listen_callback = listen_object

    def ButtonListenClick(self, sender, e):
        self._listen = [True, False][self._listen]  #nice shortcut
        self._buttonListen.BackColor = [System.Drawing.Color.White, System.Drawing.Color.LightGreen][self._listen]        
        
    def RadioBtn_Interval1CheckedChanged(self, sender, e):
        self._timer1.Interval = 50
        
    def RadioBtn_Interval2CheckedChanged(self, sender, e):
        self._timer1.Interval = 1000
        
    def RadioBtn_Interval3CheckedChanged(self, sender, e):
        self._timer1.Interval =  10000

    def NumericIDValueChanged(self, sender, e):
        self._id = self._numericID.Value
        
    def OnTimerTick(self, sender, e):
        if self.__listen_callback and self._listen: 
            self.__listen_callback(self._id)

    def MyClosedHandler(self, sender, e):
        self._listen = False
        self._timer1.Enabled = False

import scriptcontext
import socket
import traceback
import struct

##note need to open socket once only

class ListenFormController():

    def __init__(self, dialog):
        dialog.SetListenCallback(self.listen)
        # add event handlers
        dialog.FormClosed +=self.OnFormClosed
        # Geometry stuff
        self.myPoint = Rhino.Geometry.Point3d(0,0,0)
        self.prevPoint = Rhino.Geometry.Point3d(0,0,0)
        
    #Event handlers
    def OnFormClosed(self, sender, e):
        print("control off")

    def listen(self,id):
        mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        HOST = "192.168.10.%s3"%(id)
        try:
            mySocket.settimeout(1)
            mySocket.connect((HOST,30003))
            # message size on Matlab interface is 756
            bytes = mySocket.recv(756)
            tool_vector = bytes[588:636]
            pose = struct.unpack("!dddddd",tool_vector)
            self.update_position(pose[0],pose[1],pose[2])
            mySocket.close()
        except:
            # add exception
            traceback.print_exc()

    def update_position(self, x,y,z):
        self.myPoint = Rhino.Geometry.Point3d(x*1000, y*1000,z*1000)
        scriptcontext.doc.Objects.AddPoint(self.myPoint)
        _line = Rhino.Geometry.Line(self.prevPoint, self.myPoint)
        scriptcontext.doc.Objects.AddLine(_line)
        self.prevPoint = self.myPoint
        scriptcontext.doc.Views.Redraw()


def listen():
    dialog = ListenForm()
    control = ListenFormController(dialog)
    dialog.Show(Rhino.RhinoApp.MainApplicationWindow)
    

if __name__ == "__main__":
    listen() # Call the function defined above