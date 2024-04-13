from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from ScaraControl import Ui_MainWindow
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtCore import QThread, pyqtSignal
from pyzbar import pyzbar
import serial.tools.list_ports
import numpy as np
import math
import cv2

class Main_Handle(Ui_MainWindow):   
    def __init__(self):

        self.setupUi(MainWindow)

        # Read Only
        self.edtProductID.setReadOnly(True)
        self.edtProductIDAuto.setReadOnly(True)
        self.edtXDisplay.setReadOnly(True)
        self.edtYDisplay.setReadOnly(True)
        self.edtZDisplay.setReadOnly(True)

        # Serial Port and Baud Rate (Selected from QComboBox)
        self.Com_Port = None
        self.Baud_Rate = 4800
        self.ser = None

        # Define:
        self.theta1 = 0
        self.theta2 = 0
        self.Zhight = 0
        self.thread = {}
        self.condition = 0
        self.preXpos = None
        self.preYpos = None
        self.preZpos = None

        # Connect modify event for QComboBox
        self.CboxSerialPort.currentIndexChanged.connect(self.update_com_port)
        self.CboxBaudRate.currentIndexChanged.connect(self.update_baud_rate)
        
        # Link to Function process event: button connect 
        self.btnConnectSP.clicked.connect(self.btnConnectSP_Handle)
    
        # Link to Function process event: button disconnect
        self.btnDisconnect.clicked.connect(self.btnDisconnect_Handle)

        # Link to Function process event: button X-
        self.btnXccw.clicked.connect(self.btnXccw_Handle)
    
        # Link to Function process event: button X+
        self.btnXcw.clicked.connect(self.btnXcw_Handle)

        # Link to Function process event: button Y-
        self.btnYccw.clicked.connect(self.btnYccw_Handle)
    
        # Link to Function process event: button Y+
        self.btnYcw.clicked.connect(self.btnYcw_Handle)

        # Link to Function process event: button Z-
        self.btnZccw.clicked.connect(self.btnZccw_Handle)
    
        # Link to Function process event: button Z+
        self.btnZcw.clicked.connect(self.btnZcw_Handle)

        # Link to Function process event: button Inverse
        self.btnInverse.clicked.connect(self.btnInverse_Handle)

        # Link to Function process event: button Reset
        self.btnReset.clicked.connect(self.btnReset_Handle)

        # Link to Function process event: button On camera
        self.btnOnCam.clicked.connect(self.start_capture_video)

        # Link to Function process event: button On camera
        self.btnOffCam.clicked.connect(self.stop_capture_video)

        # Link to Function process event: button A1
        self.btnA1.clicked.connect(self.btnA1_Handle)

        # Link to Function process event: button A2
        self.btnA2.clicked.connect(self.btnA2_Handle)

        # Link to Function process event: button A3
        self.btnA3.clicked.connect(self.btnA3_Handle)

        # Link to Function process event: button B1
        self.btnB1.clicked.connect(self.btnB1_Handle)

        # Link to Function process event: button B2
        self.btnB2.clicked.connect(self.btnB2_Handle)

        # Link to Function process event: button B3
        self.btnB3.clicked.connect(self.btnB3_Handle)

        # Link to Function process event: button C1
        self.btnC1.clicked.connect(self.btnC1_Handle)

        # Link to Function process event: button C2
        self.btnC2.clicked.connect(self.btnC2_Handle)

        # Link to Function process event: button C3
        self.btnC3.clicked.connect(self.btnC3_Handle)

        # Link to Function process event: button Reset System
        self.btnResetSys.clicked.connect(self.btnResetSys_Handle)

        # Link to Function process event: button Start
        self.btnStart.clicked.connect(self.btnStart_Handle)
        
        

#-------------------------------COMMUNICATION-----BEGIN--------------------------------#
    # Function update com port
    def update_com_port(self, index):
        self.Com_Port = self.CboxSerialPort.currentText()

    # Functin update baud rate
    def update_baud_rate(self, index):
        self.Baud_Rate = self.CboxBaudRate.currentText()

    # Function to check if the selected port is available
    def is_port_available(self, port):
        available_ports = [p.device for p in serial.tools.list_ports.comports()]
        return port in available_ports

    # Function process event: button connect    
    def btnConnectSP_Handle(self):
        if not self.is_port_available(self.Com_Port):
            QMessageBox.warning(MainWindow, "Error", "Selected port is not available")
            return
        try: 
            self.ser = serial.Serial(self.Com_Port, self.Baud_Rate, timeout=1)
            QMessageBox.information(MainWindow, "Success", "Connected Successfully")
        except serial.SerialException as e:
            QMessageBox.warning(MainWindow, "Error","Can't connect to Serial Port: " + str(self.Com_Port))

    # Function process event: button disconnect
    def btnDisconnect_Handle(self):
        if self.ser and self.ser.isOpen():
            self.ser.close()
            QMessageBox.information(MainWindow, "Success", "Disconnected Successfully")
        else:
            QMessageBox.warning(MainWindow, "Error","Can't do this. Please selected Serial Ports!")  
#-------------------------------COMMUNICATION-----END--------------------------------#




#-------------------------------JOINTS CONTROL-----BEGIN-----------------------------#
    # Function process event: button X-
    def btnXccw_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.theta1 -= 5 # uint: degree
                data_to_send = 'MQ'
                self.ser.write(data_to_send.encode())
                self.CurPos()
                self.DisplayCurPos(self.CurX, self.CurY, self.Zhight)
            else:
                QMessageBox.warning(MainWindow, "Error","Please selected Joint Control")
        else: 
            QMessageBox.warning(MainWindow, "Error","Please Connected Serial Ports")

    # Function process event: button X+
    def btnXcw_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.theta1 += 5 # uint: degree
                data_to_send = 'MW'
                self.ser.write(data_to_send.encode())
                self.CurPos()
                self.DisplayCurPos(self.CurX, self.CurY, self.Zhight)
            else:
                QMessageBox.warning(MainWindow, "Error","Please selected Joint Control")
        else: 
            QMessageBox.warning(MainWindow, "Error","Please Connected Serial Ports")
    
    # Function process event: button Y-
    def btnYccw_Handle(self):
        if self.ser and self.ser.isOpen(): 
            if self.radioJoint.isChecked() == True:
                self.theta2 -=5 # uint: degree
                data_to_send = 'ME'
                self.ser.write(data_to_send.encode())
                self.CurPos()
                self.DisplayCurPos(self.CurX, self.CurY, self.Zhight)                
            else:
                QMessageBox.warning(MainWindow, "Error","Please selected Joint Control")
        else: 
            QMessageBox.warning(MainWindow, "Error","Please Connected Serial Ports")

    # Function process event: button Y+
    def btnYcw_Handle(self):
        if self.ser and self.ser.isOpen(): 
            if self.radioJoint.isChecked() == True:
                self.theta2 += 5 # uint: degree
                data_to_send = 'MR'
                self.ser.write(data_to_send.encode())
                self.CurPos()
                self.DisplayCurPos(self.CurX, self.CurY, self.Zhight)                
            else:
                QMessageBox.warning(MainWindow, "Error","Please selected Joint Control")
        else: 
            QMessageBox.warning(MainWindow, "Error","Please Connected Serial Ports")

    # Function process event: button Z+
    def btnZccw_Handle(self):
        if self.ser and self.ser.isOpen(): 
            if self.radioJoint.isChecked() == True:
                self.Zhight += 2 # uint: mm
                data_to_send = 'MT'
                self.ser.write(data_to_send.encode())
                self.CurPos()
                self.DisplayCurPos(self.CurX, self.CurY, self.Zhight)
            else:
                QMessageBox.warning(MainWindow, "Error","Please selected Joint Control")
        else: 
            QMessageBox.warning(MainWindow, "Error","Please Connected Serial Ports")         

    # Function process event: button Z-
    def btnZcw_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.Zhight -= 2 # uint: mm
                data_to_send = 'MY'
                self.ser.write(data_to_send.encode())
                self.CurPos()
                self.DisplayCurPos(self.CurX, self.CurY, self.Zhight)
            else:
                QMessageBox.warning(MainWindow, "Error","Please selected Joint Control")
        else: 
            QMessageBox.warning(MainWindow, "Error","Please Connected Serial Ports")
 #-------------------------------JOINTS CONTROL------END--------------------------------#  




 #-------------------------------CURRENT POSTTON-----BEGIN------------------------------#    
    # Function Set Current posittion:
    def CurPos(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:

                self.theta1rad = self.theta1*math.pi/180
                self.theta2rad = self.theta2*math.pi/180
                self.CurX =  185*math.cos(self.theta1rad + self.theta2rad) + 332*math.cos(self.theta1rad)
                self.CurY =  185*math.sin(self.theta1rad + self.theta2rad) + 332*math.sin(self.theta1rad)
                self.CurZ = self.Zhight

            if self.radioPosittion.isChecked() == True:

                self.CurX = float(self.textXpos)
                self.CurY = float(self.textYpos)
                self.CurZ = float(self.textZpos)

        return self.CurX, self.CurY, self.CurZ

    # Fuction display Current Posittion:
    def DisplayCurPos(self, DisplayX, DisplayY, DisplayZ):
        try:

            display_x_text = "{:.3f}".format(DisplayX)
            display_y_text = "{:.3f}".format(DisplayY)
            display_z_text = "{:.3f}".format(DisplayZ)

            self.edtXDisplay.setPlainText(display_x_text)
            self.edtYDisplay.setPlainText(display_y_text)
            self.edtZDisplay.setPlainText(display_z_text)

        except ValueError:
            QMessageBox.warning(MainWindow, "Error", "Invalid Display Positions")

    # Funtion reset: move robot back home.
    def btnReset_Handle(self):

        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True :

                resetData = 'V ' + str(self.theta1rad) + ' ' + str(self.theta2rad) + ' ' + str(self.Zhight)
                self.ser.write(resetData.encode())

                self.theta1 = 0
                self.theta2 = 0
                self.Zhight = 0

                self.CurPos()
                self.DisplayCurPos(self.CurX, self.CurY, self.Zhight)

            if self.radioPosittion.isChecked() == True:

                self.costheta2 = (float(self.textXpos)*float(self.textXpos) + float(self.textYpos)*float(self.textYpos)- 332*332 - 185*185)/(2*332*185)
                self.sintheta2 = math.sqrt(math.fabs(1-self.costheta2*self.costheta2))

                self.costheta1 = float(self.textXpos)*(332 + 185*self.costheta2) + float(self.textYpos)*185*self.sintheta2
                self.sintheta1 = float(self.textYpos)*(332 + 185*self.costheta2) - float(self.textXpos)*185*self.sintheta2

                if self.costheta1 == 0:
                    self.costheta1 = 0.00000000000000000000000000000000001
                if self.costheta2 == 0:
                    self.costheta2 = 0.00000000000000000000000000000000001

                self.theta1rad = math.atan(self.sintheta1/self.costheta1)
                self.theta2rad = math.atan(self.sintheta2/self.costheta2)

                if self.costheta2 < 0:
                    self.theta2rad = math.pi + self.theta2rad
                if self.costheta1 < 0:
                    self.theta1rad = math.pi + self.theta1rad

                resetData = 'V ' + str(self.theta1rad) + ' ' + str(self.theta2rad) + ' ' + str(self.textZpos)
                self.ser.write(resetData.encode())

                self.theta1rad = 0
                self.theta2rad = 0

                self.condition = 0

                self.DisplayCurPos(517,0,0) 
        else:
            QMessageBox.warning(MainWindow, "Error", "Please Connected Serial Ports!")

    # Function reset all system: erase stm32 
    def btnResetSys_Handle(self):
        if self.ser and self.ser.isOpen():
            data_to_send = 'L'
            self.ser.write(data_to_send.encode())
#-------------------------------CURRENT POSTTON-----END------------------------------#  
            



#-------------------------------POSITTION CONTROL---BEGIN--------------------------# 
    # Function check input valid
    def validFloatInput(self):

        self.textXpos = self.edtXPos.toPlainText()
        self.textYpos = self.edtYPos.toPlainText()
        self.textZpos = self.edtZPos.toPlainText()

        try:
            self.Xpos = float(self.textXpos)
            self.Ypos = float(self.textYpos)
            self.Zpos = float(self.textZpos)
            return True
        
        except ValueError:
            QMessageBox.warning(MainWindow,"Error", "Input value is not vaild. Please write float number!")
            return False

    # Function process event: button Inverse
    def btnInverse_Handle(self):
                    
        self.condition += 1
    
        if self.ser and self.ser.isOpen():
            if self.radioPosittion.isChecked() == True:
                if self.validFloatInput():

                    if self.condition == 1:

                        self.Xpos = str(float(self.textXpos)) 
                        self.Ypos = str(float(self.textYpos))
                        self.Zpos = str(float(self.textZpos))

                        data_cur_pos = '517 0 0 '
                        data_real_pos = self.Xpos + ' ' + self.Ypos + ' ' + self.Zpos

                        data_to_send = 'A ' + data_cur_pos  + data_real_pos  
                        self.ser.write(data_to_send.encode())

                        self.CurPos()   
                        self.DisplayCurPos(self.CurX, self.CurY, self.CurZ)

                        self.preXpos = self.Xpos
                        self.preYpos = self.Ypos
                        self.preZpos = self.Zpos

                    if self.condition == 2:

                        self.Xpos = str(float(self.textXpos)) 
                        self.Ypos = str(float(self.textYpos))
                        self.Zpos = str(float(self.textZpos))

                        data_cur_pos = self.preXpos + ' ' + self.preYpos + ' ' + self.preZpos + ' '
                        data_real_pos = self.Xpos + ' ' + self.Ypos + ' ' + self.Zpos

                        data_to_send = 'A ' + data_cur_pos  + data_real_pos  
                        self.ser.write(data_to_send.encode())

                        self.CurPos()   
                        self.DisplayCurPos(self.CurX, self.CurY, self.CurZ)

                        self.preXpos = self.Xpos
                        self.preYpos = self.Ypos
                        self.preZPos = self.Zpos

                        self.condition -= 1
            else: 
                QMessageBox.warning(MainWindow, "Error", "Please selected Position control mode!")
        else:
            QMessageBox.warning(MainWindow, "Error", "Please Connected Serial Ports!") 

    # Function disable button in posittion control.
    def Permit_Button(self):
        if self.ser and self.ser.isOpen():
            if self.radioPosittion.isChecked() == True:

                self.btnA1.setEnabled(False)
                self.btnA2.setEnabled(False)
                self.btnA3.setEnabled(False)
                self.btnB1.setEnabled(False)
                self.btnB2.setEnabled(False)
                self.btnB3.setEnabled(False)
                self.btnC1.setEnabled(False)
                self.btnC2.setEnabled(False)
                self.btnC3.setEnabled(False)
#-------------------------------POSITTION CONTROL-----END-------------------------#        





#-------------------------------DETECT QR CODE -------BEGIN-----------------------#
    # fuction dispaly img on GUI.
    def show_wedcam1(self, cv_img):
        # Updates the image_label with a new opencv image
        qt_img = self.convert_cv_qt1(cv_img)
        self.lbCamera.setPixmap(qt_img)
        self.edtProductID.setPlainText(self.thread[1].get_qr_code())

    # fuction convert img to display on GUI.
    def convert_cv_qt1(self, cv_img):
        # Convert from an opencv image to QPixmap
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(480, 360, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)  

    # function on camera:    
    def start_capture_video(self):
        self.thread[1] = capture_video(index = 1)
        self.thread[1].start()
        self.thread[1].signal.connect(self.show_wedcam1)

        self.btnStart.setEnabled(False)

    # function off camera:
    def stop_capture_video(self):
        self.thread[1].stop()
#-------------------------------DETECT QR CODE -------END-----------------------#



#------------------------------IMPORT PRODUCTS--------BEGIN-----------------------#     
    # Function set pos of warehouse:
    def warehouse_pos(self, ProductId):

        self.LengthWH = 158
        self.disX = 0
        self.disY = 250

        if ProductId % 3 != 0:
            self.centerX = (ProductId%3)*(self.LengthWH/3) - (self.LengthWH/6) + self.disX;
            self.centerY = ((int)(ProductId/3) + 1)* (self.LengthWH /3) - (self.LengthWH/6) + self.disY;
        else:
            self.centerX = self.LengthWH - self.LengthWH/6 + self.disX
            self.centerY = (ProductId/3)*(self.LengthWH/3) - (self.LengthWH/6) +self.disY

        return self.centerX,self.centerY

    # Funtion disable and change color for button.
    def setButton_Disable(self, button):
        button.setStyleSheet("QPushButton{\n"
                                "    background-color: rgb(255, 124, 124);\n"
                                "    border: 2px solid;\n"
                                "    border-top-left-radius: 20px;\n"
                                "    border-top-right-radius: 20px;\n"
                                "    border-bottom-right-radius: 20px;\n"
                                "    border-bottom-left-radius: 20px;\n"
                                "}\n")
        button.setEnabled(False)

    # Function btn A1
    def btnA1_Handle(self):
        #if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:

                #  Caculate current posittion - send to stm32.
                data_to_send = 'I ' + str(self.CurX) + ' ' + str(self.CurY) + ' ' + str(self.CurZ) + ' 1'
                self.ser.write(data_to_send.encode())

                # reset variable 
                self.theta1 = 0
                self.theta2 = 0
                self.Zhight = 0

                # Display current posittion
                self.CurPos()
                self.DisplayCurPos(self.CurX,self.CurY,self.CurZ)

                self.setButton_Disable(self.btnA1)

    # Function btn A2
    def btnA2_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.CurPos()
                data_to_send = 'I ' + str(self.CurX) + ' ' + str(self.CurY) + ' ' + str(self.CurZ) + ' 2'
                self.ser.write(data_to_send.encode())
                self.theta1 = 0
                self.theta2 = 0
                self.Zhight = 0
                self.CurPos()
                self.DisplayCurPos(self.CurX,self.CurY,self.CurZ)

                self.setButton_Disable(self.btnA2)

    # Function btn A3
    def btnA3_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.ser and self.ser.isOpen():
                self.CurPos()
                data_to_send = 'I ' + str(self.CurX) + ' ' + str(self.CurY) + ' ' + str(self.CurZ) + ' 3'
                self.ser.write(data_to_send.encode())
                self.theta1 = 0
                self.theta2 = 0
                self.Zhight = 0
                self.CurPos()
                self.DisplayCurPos(self.CurX,self.CurY,self.CurZ)

                self.setButton_Disable(self.btnA3)

    # Function btn B1
    def btnB1_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.CurPos()
                data_to_send = 'I ' + str(self.CurX) + ' ' + str(self.CurY) + ' ' + str(self.CurZ) + ' 4'
                self.ser.write(data_to_send.encode())
                self.theta1 = 0
                self.theta2 = 0
                self.Zhight = 0
                self.CurPos()
                self.DisplayCurPos(self.CurX,self.CurY,self.CurZ)

                self.setButton_Disable(self.btnB1)

    # Function btn B2
    def btnB2_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.CurPos()
                data_to_send = 'I ' + str(self.CurX) + ' ' + str(self.CurY) + ' ' + str(self.CurZ) + ' 5'
                self.ser.write(data_to_send.encode())
                self.theta1 = 0
                self.theta2 = 0
                self.Zhight = 0
                self.CurPos()
                self.DisplayCurPos(self.CurX,self.CurY,self.CurZ)

                self.setButton_Disable(self.btnB2)

    # Function btn B3
    def btnB3_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.CurPos()
                data_to_send = 'I ' + str(self.CurX) + ' ' + str(self.CurY) + ' ' + str(self.CurZ) + ' 6'
                self.ser.write(data_to_send.encode())
                self.theta1 = 0
                self.theta2 = 0
                self.CurZ = 0
                self.CurPos()
                self.DisplayCurPos(self.CurX,self.CurY,self.CurZ)

                self.setButton_Disable(self.btnB3)

    # Function btn C1
    def btnC1_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.CurPos()
                data_to_send = 'I ' + str(self.CurX) + ' ' + str(self.CurY) + ' ' + str(self.CurZ) + ' 7'
                self.ser.write(data_to_send.encode())
                self.theta1 = 0
                self.theta2 = 0
                self.Zhight = 0
                self.CurPos()
                self.DisplayCurPos(self.CurX,self.CurY,self.CurZ)

                self.setButton_Disable(self.btnC1)

    # Function btn C2
    def btnC2_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.CurPos()
                data_to_send = 'I ' + str(self.CurX) + ' ' + str(self.CurY) + ' ' + str(self.CurZ) + ' 8'
                self.ser.write(data_to_send.encode())
                self.theta1 = 0
                self.theta2 = 0
                self.Zhight = 0
                self.CurPos()
                self.DisplayCurPos(self.CurX,self.CurY,self.CurZ)

                self.setButton_Disable(self.btnC2)

    # Function btn C3
    def btnC3_Handle(self):
        if self.ser and self.ser.isOpen():
            if self.radioJoint.isChecked() == True:
                self.CurPos()
                data_to_send = 'I ' + str(self.CurX) + ' ' + str(self.CurY) + ' ' + str(self.CurZ) + ' 9'
                self.ser.write(data_to_send.encode())
                self.theta1 = 0
                self.theta2 = 0
                self.Zhight = 0
                self.CurPos()
                self.DisplayCurPos(self.CurX,self.CurY,self.CurZ)

                self.setButton_Disable(self.btnC3)
#------------------------------IMPORT PRODUCTS--------END-----------------------#     




#------------------------------AUTO MODE --------BEGIN-----------------------#     
    # fuction dispaly img on GUI.
    def show_wedcam2(self, cv_img):
        # Updates the image_label with a new opencv image
        qt_img = self.convert_cv_qt2(cv_img)
        self.lbCamera.setPixmap(qt_img)
        self.edtProductIDAuto.setPlainText(self.thread[1].get_qr_code())

    # fuction convert img to display on GUI.
    def convert_cv_qt2(self, cv_img):
        # Convert from an opencv image to QPixmap
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(480, 360, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)  
    
    # Function start camera:
    def btnStart_Handle(self):
        self.thread[1] = capture_video(index = 1)
        self.thread[1].start()
        self.thread[1].signal.connect(self.show_wedcam2)

        self.btnOnCam.setEnabled(False)
#------------------------------AUTO MODE --------END-----------------------#   




#-------------------------------CLASS DETECT QR CODE -------BEGIN-----------------------#        
class capture_video(QThread):

    signal = pyqtSignal(np.ndarray) # export signal (image)

    def __init__(self, index):
        self.index = index
        super(capture_video, self).__init__()
        self.text_qrcode = ""
        
    def run(self):
        cap = cv2.VideoCapture(2)
        while True:
            ret, cv_img = cap.read()
            if ret:
                qrcodes = pyzbar.decode(cv_img)
                for qrcode in qrcodes:
                    (x, y, w, h) = qrcode.rect

                    # Tính toán tâm của mã QR code trong tệp đặc biệt so với khung hình
                    qr_center_x = x + w // 2
                    qr_center_y = y + h // 2
                    frame_center_x = cv_img.shape[1] // 2
                    frame_center_y = cv_img.shape[0] // 2

                    # Tính toán tọa độ tâm của mã QR code so với khung hình
                    qr_offset_x = qr_center_x - frame_center_x
                    qr_offset_y = qr_center_y - frame_center_y      

                    cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    self.qrcode_data = qrcode.data.decode("utf-8")
                    self.text_qrcode = "{}".format(self.qrcode_data)

                    # Vẽ điểm tâm của mã QR code trên khung hình
                    cv2.circle(cv_img, (qr_center_x, qr_center_y), 5, (255, 0, 0), -1)
                    cv2.putText(cv_img, self.text_qrcode, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    cv2.putText(cv_img, "Offset: ({}, {})".format(qr_offset_x, qr_offset_y), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                self.signal.emit(cv_img)

    def get_qr_code(self):
        return self.text_qrcode
    
    def stop(self):
        print("Stop Threading", self.index)
        self.terminate()
#-------------------------------CLASS DETECT QR CODE -------END-------------------------#
        


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Main_Handle()
    MainWindow.show()
    sys.exit(app.exec_())

