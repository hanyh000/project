# 3.9

import serial
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import *

global ser 
ser = serial.Serial('COM8',9600,timeout=0.05)      

class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt5_pyserial")

        central = QWidget()
        self.setCentralWidget(central)
        vbox = QVBoxLayout(central)
        hbox = QHBoxLayout()
        gbox = QGridLayout()

        self.echo = QLabel()
        self.echo.setAlignment(Qt.AlignCenter)
        self.echo.setText("......")
        vbox.addWidget(self.echo)

        self.button_STOP = QPushButton()
        self.button_STOP.setText("STOP")
        self.button_STOP.clicked.connect(self.STOP)
        hbox.addWidget(self.button_STOP)

        self.button_buzz = QPushButton()
        self.button_buzz.setText("BUZZER")
        self.button_buzz.clicked.connect(self.BUZZER)
        hbox.addWidget(self.button_buzz)
        hbox.addStretch(1)

        self.light = QLabel()
        self.light.setPixmap(QPixmap("g.png"))
        hbox.addWidget(self.light)

        self.button_CW = QPushButton()
        self.button_CW.setText("CW")
        self.button_CW.clicked.connect(self.CW)
        gbox.addWidget(self.button_CW,0,1)

        self.button_CCW = QPushButton()
        self.button_CCW.setText("CCW")
        self.button_CCW.clicked.connect(self.CCW)
        gbox.addWidget(self.button_CCW,1,1)

        self.button_LEFT = QPushButton()
        self.button_LEFT.setText("LEFT")
        self.button_LEFT.clicked.connect(self.LEFT)
        gbox.addWidget(self.button_LEFT,1,0)

        self.button_RIGHT = QPushButton()
        self.button_RIGHT.setText("RIGHT")
        self.button_RIGHT.clicked.connect(self.RIGHT)
        gbox.addWidget(self.button_RIGHT,1,2)

        vbox.addLayout(hbox)
        vbox.addLayout(gbox)
        
        if ser and ser.is_open:
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.receive)
            self.timer.setInterval(50)              
            self.timer.start()

    def receive(self):
        if ser and ser.is_open and ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line: 
                if line.startswith("DISTANCE : "):
                    distance_str = line.split(":")[1].strip()
                    try:
                        distance = int(distance_str)
                        self.echo.setText(f"{distance} CM")
                    except ValueError:
                        print(f"{distance_str} CM")

                elif line.startswith("DATA : "):
                    motor_status = line.split(":")[1].strip()
                    print(f"모터 상태 : {motor_status}")

                elif line.startswith("ALERT : "):
                    alert_level = line.split(":")[1].strip()

                    if alert_level == 'R':
                        self.light.setPixmap(QPixmap("r.png"))
                    elif alert_level == 'Y':
                        self.light.setPixmap(QPixmap("y.png"))
                    else :
                        self.light.setPixmap(QPixmap("g.png"))                

    def send(self,send_str):
        if ser and ser.is_open:
            if send_str:
                data1 = (send_str + '\n').encode('utf-8')
                ser.write(data1)
            else:
                return False

    def BUZZER(self):
        self.send("BUZZER")

    def CW(self):
        self.send("CW") 

    def CCW(self):
        self.send("CCW")

    def STOP(self):
        self.send("STOP")

    def LEFT(self):
        self.send("LEFT")
    
    def RIGHT(self):
        self.send("RIGHT")

    def closeEvent(self,event):
        if ser and ser.is_open:
            ser.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Window()
    w.resize(500,300)
    w.show()
    app.exec_()