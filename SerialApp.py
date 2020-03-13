import wx
import sys
import serial.tools.list_ports
import threading
import time

def ReadingThread(self):
    while True:
        if self.ser.in_waiting != 0:
            print(self.ser.readline())
        time.sleep(1)


class MyFrame(wx.Frame):
    def __init__(self):
        super().__init__(parent=None, title='Serial Comands V1.0', size=(550,300), style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER)
        self.dont_show = False
        self.connected = False

        self.SetBackgroundColour(wx.Colour(0, 25, 51))
        self.panel = wx.Panel(self)
        vBox = wx.BoxSizer(wx.VERTICAL)
        style = wx.TE_MULTILINE | wx.TE_READONLY | wx.HSCROLL
        self.log = wx.TextCtrl(self.panel, wx.ID_ANY, size=(500,200),
                          style=style)
        self.log.SetBackgroundColour(wx.Colour(64, 64, 64))
        self.log.SetForegroundColour(wx.Colour(255, 255, 255))
        vBox.Add(self.log, 0, wx.ALL | wx.CENTER, 5)
        self.hBox = wx.BoxSizer(wx.HORIZONTAL)
        self.text_ctrl = wx.TextCtrl(self.panel, size =(175, 25))
        self.text_ctrl.SetValue("COM4")
        self.hBox.Add(self.text_ctrl, 0, wx.ALL | wx.LEFT, 5)
        self.ButtonCreates()
        vBox.Add(self.hBox, 1, wx.ALL | wx.ALIGN_CENTER)
        self.panel.SetSizer(vBox)
        self.Show()

        # redirect text here
        sys.stdout = self.log


    def SerialConnect(self):
        ports = serial.tools.list_ports.comports()
        if len(ports) == 0:
            print("None COM available, Plug in device ")
            self.plugIn = False
        else:
            print("Available Ports: ")
            for port, desc, hwid in sorted(ports):
                print("{} | {}".format(port, desc))
            self.plugIn = True


    def ShowDialog(self, message='Default Message'):
        dlg = wx.RichMessageDialog(self, f"{message}")
        dlg.ShowModal()  # return value ignored as we have "Ok" only anyhow


    def ButtonCreates(self):
        #create first button
        self.my_btn = wx.Button(self.panel, label='Connect')
        self.my_btn.Bind(wx.EVT_BUTTON, self.OnPress1)
        self.hBox.Add(self.my_btn, 0, wx.ALL | wx.ALIGN_LEFT, 5)

        my_btn = wx.Button(self.panel, label='Command 1')
        my_btn.Bind(wx.EVT_BUTTON, self.OnPress2)
        self.hBox.Add(my_btn, 0, wx.ALL | wx.ALIGN_LEFT, 5)

        my_btn = wx.Button(self.panel, label='Command 2')
        my_btn.Bind(wx.EVT_BUTTON, self.OnPress3)
        self.hBox.Add(my_btn, 0, wx.ALL | wx.ALIGN_LEFT, 5)


    def OnPress1(self, event):
        if self.plugIn == False:
            self.SerialConnect()

        elif self.connected == True:
            command = b'\x42\x42\x42\x0a'  # \x42
            print(f'command sended: {command}')
            x = self.ser.write(command)

        elif self.plugIn and self.connected == False:
            value = self.text_ctrl.GetValue()
            self.ser = serial.Serial(f"{value}", 9600)
            x = threading.Thread(target=ReadingThread, args=(self,), daemon=True)
            x.start()
            print("Connedcted !")
            print("Reading Thread active !")
            self.my_btn.SetLabel("Send")
            self.connected = True


    def OnPress2(self, event):
        if self.connected:
             command = b'\x43\x43\x43\x0a'
             print(f'command sended: {command}')
             x = self.ser.write(command)
        else:
            self.SerialConnect()


    def OnPress3(self, event):
        if self.connected:
            command = b'\x44\x44\x44\x0a'
            print(f'command sended: {command}')
            x = self.ser.write(command)
        else:
            self.SerialConnect()


if __name__ == '__main__':
    app = wx.App()
    frame = MyFrame()
    frame.ShowDialog("Select available COM and write it down")
    frame.SerialConnect()

    app.MainLoop()