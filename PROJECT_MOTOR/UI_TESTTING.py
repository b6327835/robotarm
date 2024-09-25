# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'UI_TESTTING.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(819, 540)
        MainWindow.setStyleSheet("background-color: rgb(170, 85, 0);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.stackedWidget = QtWidgets.QStackedWidget(self.centralwidget)
        self.stackedWidget.setGeometry(QtCore.QRect(150, 20, 661, 421))
        self.stackedWidget.setStyleSheet("background-color: rgb(255, 170, 127);")
        self.stackedWidget.setObjectName("stackedWidget")
        self.manpage = QtWidgets.QWidget()
        self.manpage.setObjectName("manpage")
        self.horizontalSlider_2 = QtWidgets.QSlider(self.manpage)
        self.horizontalSlider_2.setGeometry(QtCore.QRect(70, 240, 271, 31))
        self.horizontalSlider_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.horizontalSlider_2.setMouseTracking(False)
        self.horizontalSlider_2.setTabletTracking(False)
        self.horizontalSlider_2.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.horizontalSlider_2.setMaximum(200)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.home_bottom = QtWidgets.QPushButton(self.manpage)
        self.home_bottom.setGeometry(QtCore.QRect(460, 200, 111, 61))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.home_bottom.setFont(font)
        self.home_bottom.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.home_bottom.setObjectName("home_bottom")
        self.Z_core = QtWidgets.QLabel(self.manpage)
        self.Z_core.setGeometry(QtCore.QRect(160, 300, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.Z_core.setFont(font)
        self.Z_core.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.Z_core.setAlignment(QtCore.Qt.AlignCenter)
        self.Z_core.setObjectName("Z_core")
        self.horizontalSlider_3 = QtWidgets.QSlider(self.manpage)
        self.horizontalSlider_3.setGeometry(QtCore.QRect(70, 350, 271, 31))
        self.horizontalSlider_3.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.horizontalSlider_3.setMouseTracking(False)
        self.horizontalSlider_3.setTabletTracking(False)
        self.horizontalSlider_3.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.horizontalSlider_3.setMaximum(200)
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setObjectName("horizontalSlider_3")
        self.start_bottom = QtWidgets.QPushButton(self.manpage)
        self.start_bottom.setGeometry(QtCore.QRect(460, 340, 111, 61))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.start_bottom.setFont(font)
        self.start_bottom.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.start_bottom.setObjectName("start_bottom")
        self.label = QtWidgets.QLabel(self.manpage)
        self.label.setGeometry(QtCore.QRect(80, 300, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.label_4 = QtWidgets.QLabel(self.manpage)
        self.label_4.setGeometry(QtCore.QRect(300, 190, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_4.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.manpage)
        self.label_5.setGeometry(QtCore.QRect(300, 300, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_5.setFont(font)
        self.label_5.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_5.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.X_core = QtWidgets.QLabel(self.manpage)
        self.X_core.setGeometry(QtCore.QRect(160, 80, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.X_core.setFont(font)
        self.X_core.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.X_core.setAlignment(QtCore.Qt.AlignCenter)
        self.X_core.setObjectName("X_core")
        self.label_3 = QtWidgets.QLabel(self.manpage)
        self.label_3.setGeometry(QtCore.QRect(80, 80, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_3.setFont(font)
        self.label_3.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_3.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.Y_core = QtWidgets.QLabel(self.manpage)
        self.Y_core.setGeometry(QtCore.QRect(160, 190, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.Y_core.setFont(font)
        self.Y_core.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.Y_core.setAlignment(QtCore.Qt.AlignCenter)
        self.Y_core.setObjectName("Y_core")
        self.label_2 = QtWidgets.QLabel(self.manpage)
        self.label_2.setGeometry(QtCore.QRect(80, 190, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_2.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalSlider_1 = QtWidgets.QSlider(self.manpage)
        self.horizontalSlider_1.setGeometry(QtCore.QRect(70, 130, 271, 31))
        self.horizontalSlider_1.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.horizontalSlider_1.setMouseTracking(False)
        self.horizontalSlider_1.setTabletTracking(False)
        self.horizontalSlider_1.setFocusPolicy(QtCore.Qt.WheelFocus)
        self.horizontalSlider_1.setAutoFillBackground(False)
        self.horizontalSlider_1.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.horizontalSlider_1.setMaximum(300)
        self.horizontalSlider_1.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_1.setObjectName("horizontalSlider_1")
        self.label_6 = QtWidgets.QLabel(self.manpage)
        self.label_6.setGeometry(QtCore.QRect(300, 80, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_6.setFont(font)
        self.label_6.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_6.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.manpage)
        self.label_7.setGeometry(QtCore.QRect(140, 20, 141, 41))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.label_7.setFont(font)
        self.label_7.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.Initial_bottom = QtWidgets.QPushButton(self.manpage)
        self.Initial_bottom.setGeometry(QtCore.QRect(460, 100, 111, 61))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Initial_bottom.setFont(font)
        self.Initial_bottom.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.Initial_bottom.setObjectName("Initial_bottom")
        self.label_28 = QtWidgets.QLabel(self.manpage)
        self.label_28.setGeometry(QtCore.QRect(390, 40, 251, 41))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.label_28.setFont(font)
        self.label_28.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_28.setAlignment(QtCore.Qt.AlignCenter)
        self.label_28.setObjectName("label_28")
        self.stackedWidget.addWidget(self.manpage)
        self.visionpage = QtWidgets.QWidget()
        self.visionpage.setObjectName("visionpage")
        self.Display = QtWidgets.QLabel(self.visionpage)
        self.Display.setGeometry(QtCore.QRect(20, 30, 471, 361))
        self.Display.setStyleSheet("background-color: rgb(170, 255, 127);")
        self.Display.setAlignment(QtCore.Qt.AlignCenter)
        self.Display.setObjectName("Display")
        self.Gopoint = QtWidgets.QPushButton(self.visionpage)
        self.Gopoint.setGeometry(QtCore.QRect(500, 330, 151, 61))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.Gopoint.setFont(font)
        self.Gopoint.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.Gopoint.setObjectName("Gopoint")
        self.Vision_X = QtWidgets.QLabel(self.visionpage)
        self.Vision_X.setGeometry(QtCore.QRect(570, 100, 81, 51))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.Vision_X.setFont(font)
        self.Vision_X.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.Vision_X.setAlignment(QtCore.Qt.AlignCenter)
        self.Vision_X.setObjectName("Vision_X")
        self.Vision_Y = QtWidgets.QLabel(self.visionpage)
        self.Vision_Y.setGeometry(QtCore.QRect(570, 170, 81, 51))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.Vision_Y.setFont(font)
        self.Vision_Y.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.Vision_Y.setAlignment(QtCore.Qt.AlignCenter)
        self.Vision_Y.setObjectName("Vision_Y")
        self.label_16 = QtWidgets.QLabel(self.visionpage)
        self.label_16.setGeometry(QtCore.QRect(520, 100, 41, 51))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_16.setFont(font)
        self.label_16.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setObjectName("label_16")
        self.label_18 = QtWidgets.QLabel(self.visionpage)
        self.label_18.setGeometry(QtCore.QRect(520, 170, 41, 51))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_18.setFont(font)
        self.label_18.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.label_18.setAlignment(QtCore.Qt.AlignCenter)
        self.label_18.setObjectName("label_18")
        self.stackedWidget.addWidget(self.visionpage)
        self.jogpage = QtWidgets.QWidget()
        self.jogpage.setObjectName("jogpage")
        self.jogxu = QtWidgets.QPushButton(self.jogpage)
        self.jogxu.setGeometry(QtCore.QRect(526, 90, 91, 51))
        self.jogxu.setStyleSheet("background-color: rgb(170, 255, 127);")
        icon = QtGui.QIcon.fromTheme("Arrow")
        self.jogxu.setIcon(icon)
        self.jogxu.setObjectName("jogxu")
        self.jogyd = QtWidgets.QPushButton(self.jogpage)
        self.jogyd.setGeometry(QtCore.QRect(260, 200, 91, 51))
        self.jogyd.setStyleSheet("background-color: rgb(255, 85, 0);")
        self.jogyd.setObjectName("jogyd")
        self.jogxd = QtWidgets.QPushButton(self.jogpage)
        self.jogxd.setGeometry(QtCore.QRect(260, 90, 91, 51))
        self.jogxd.setStyleSheet("background-color: rgb(255, 85, 0);")
        self.jogxd.setObjectName("jogxd")
        self.jogyu = QtWidgets.QPushButton(self.jogpage)
        self.jogyu.setGeometry(QtCore.QRect(526, 200, 91, 51))
        self.jogyu.setStyleSheet("background-color: rgb(170, 255, 127);")
        self.jogyu.setObjectName("jogyu")
        self.jogzd = QtWidgets.QPushButton(self.jogpage)
        self.jogzd.setGeometry(QtCore.QRect(260, 310, 91, 51))
        self.jogzd.setStyleSheet("background-color: rgb(255, 85, 0);")
        self.jogzd.setObjectName("jogzd")
        self.jogzu = QtWidgets.QPushButton(self.jogpage)
        self.jogzu.setGeometry(QtCore.QRect(526, 310, 91, 51))
        self.jogzu.setStyleSheet("background-color: rgb(170, 255, 127);")
        self.jogzu.setObjectName("jogzu")
        self.label_9 = QtWidgets.QLabel(self.jogpage)
        self.label_9.setGeometry(QtCore.QRect(286, 260, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_9.setFont(font)
        self.label_9.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_9.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.jogpage)
        self.label_10.setGeometry(QtCore.QRect(286, 40, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_10.setFont(font)
        self.label_10.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_10.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.jogpage)
        self.label_11.setGeometry(QtCore.QRect(556, 260, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_11.setFont(font)
        self.label_11.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_11.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.label_12 = QtWidgets.QLabel(self.jogpage)
        self.label_12.setGeometry(QtCore.QRect(556, 150, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_12.setFont(font)
        self.label_12.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_12.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(self.jogpage)
        self.label_13.setGeometry(QtCore.QRect(286, 150, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_13.setFont(font)
        self.label_13.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_13.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.label_14 = QtWidgets.QLabel(self.jogpage)
        self.label_14.setGeometry(QtCore.QRect(556, 40, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_14.setFont(font)
        self.label_14.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_14.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.Z_core_j = QtWidgets.QLabel(self.jogpage)
        self.Z_core_j.setGeometry(QtCore.QRect(386, 310, 101, 51))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.Z_core_j.setFont(font)
        self.Z_core_j.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.Z_core_j.setAlignment(QtCore.Qt.AlignCenter)
        self.Z_core_j.setObjectName("Z_core_j")
        self.X_core_j = QtWidgets.QLabel(self.jogpage)
        self.X_core_j.setGeometry(QtCore.QRect(386, 90, 101, 51))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.X_core_j.setFont(font)
        self.X_core_j.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.X_core_j.setAlignment(QtCore.Qt.AlignCenter)
        self.X_core_j.setObjectName("X_core_j")
        self.Y_core_j = QtWidgets.QLabel(self.jogpage)
        self.Y_core_j.setGeometry(QtCore.QRect(386, 200, 101, 51))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.Y_core_j.setFont(font)
        self.Y_core_j.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.Y_core_j.setAlignment(QtCore.Qt.AlignCenter)
        self.Y_core_j.setObjectName("Y_core_j")
        self.X_core_j_2 = QtWidgets.QLabel(self.jogpage)
        self.X_core_j_2.setGeometry(QtCore.QRect(366, 30, 151, 51))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.X_core_j_2.setFont(font)
        self.X_core_j_2.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.X_core_j_2.setAlignment(QtCore.Qt.AlignCenter)
        self.X_core_j_2.setObjectName("X_core_j_2")
        self.lineEdit = QtWidgets.QLineEdit(self.jogpage)
        self.lineEdit.setGeometry(QtCore.QRect(30, 190, 161, 51))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.lineEdit.setFont(font)
        self.lineEdit.setCursorPosition(1)
        self.lineEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit.setObjectName("lineEdit")
        self.X_core_j_3 = QtWidgets.QLabel(self.jogpage)
        self.X_core_j_3.setGeometry(QtCore.QRect(40, 130, 151, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.X_core_j_3.setFont(font)
        self.X_core_j_3.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.X_core_j_3.setAlignment(QtCore.Qt.AlignCenter)
        self.X_core_j_3.setObjectName("X_core_j_3")
        self.selectbot = QtWidgets.QPushButton(self.jogpage)
        self.selectbot.setGeometry(QtCore.QRect(20, 260, 81, 51))
        self.selectbot.setStyleSheet("background-color: rgb(255, 85, 0);")
        self.selectbot.setObjectName("selectbot")
        self.Restsetbot = QtWidgets.QPushButton(self.jogpage)
        self.Restsetbot.setGeometry(QtCore.QRect(130, 260, 81, 51))
        self.Restsetbot.setStyleSheet("background-color: rgb(170, 255, 127);")
        self.Restsetbot.setObjectName("Restsetbot")
        self.stackedWidget.addWidget(self.jogpage)
        self.mannualMode = QtWidgets.QPushButton(self.centralwidget)
        self.mannualMode.setGeometry(QtCore.QRect(10, 100, 131, 61))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.mannualMode.setFont(font)
        self.mannualMode.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.mannualMode.setObjectName("mannualMode")
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setGeometry(QtCore.QRect(10, 30, 131, 41))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.label_8.setFont(font)
        self.label_8.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.jogMode = QtWidgets.QPushButton(self.centralwidget)
        self.jogMode.setGeometry(QtCore.QRect(10, 180, 131, 61))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.jogMode.setFont(font)
        self.jogMode.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.jogMode.setObjectName("jogMode")
        self.VSMode = QtWidgets.QPushButton(self.centralwidget)
        self.VSMode.setGeometry(QtCore.QRect(10, 260, 131, 61))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.VSMode.setFont(font)
        self.VSMode.setStyleSheet("background-color: rgb(255, 170, 0);")
        self.VSMode.setObjectName("VSMode")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 819, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.stackedWidget.setCurrentIndex(1)
        self.horizontalSlider_3.valueChanged['int'].connect(self.Z_core.setNum) # type: ignore
        self.horizontalSlider_2.valueChanged['int'].connect(self.Y_core.setNum) # type: ignore
        self.horizontalSlider_1.valueChanged['int'].connect(self.X_core.setNum) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.home_bottom.setText(_translate("MainWindow", "Home"))
        self.Z_core.setText(_translate("MainWindow", "0"))
        self.start_bottom.setText(_translate("MainWindow", "Start"))
        self.label.setText(_translate("MainWindow", "Z -"))
        self.label_4.setText(_translate("MainWindow", "Y +"))
        self.label_5.setText(_translate("MainWindow", "Z +"))
        self.X_core.setText(_translate("MainWindow", "0"))
        self.label_3.setText(_translate("MainWindow", "X -"))
        self.Y_core.setText(_translate("MainWindow", "0"))
        self.label_2.setText(_translate("MainWindow", "Y -"))
        self.label_6.setText(_translate("MainWindow", "X +"))
        self.label_7.setText(_translate("MainWindow", "Millimed"))
        self.Initial_bottom.setText(_translate("MainWindow", "Initial"))
        self.label_28.setText(_translate("MainWindow", "Mannual Mode"))
        self.Display.setText(_translate("MainWindow", "Display"))
        self.Gopoint.setText(_translate("MainWindow", "Go to taget"))
        self.Vision_X.setText(_translate("MainWindow", "0"))
        self.Vision_Y.setText(_translate("MainWindow", "0"))
        self.label_16.setText(_translate("MainWindow", "X:"))
        self.label_18.setText(_translate("MainWindow", "Y:"))
        self.jogxu.setText(_translate("MainWindow", "Up"))
        self.jogyd.setText(_translate("MainWindow", "Down"))
        self.jogxd.setText(_translate("MainWindow", "Down"))
        self.jogyu.setText(_translate("MainWindow", "Up"))
        self.jogzd.setText(_translate("MainWindow", "Down"))
        self.jogzu.setText(_translate("MainWindow", "Up"))
        self.label_9.setText(_translate("MainWindow", "Z -"))
        self.label_10.setText(_translate("MainWindow", "X -"))
        self.label_11.setText(_translate("MainWindow", "Z +"))
        self.label_12.setText(_translate("MainWindow", "Y +"))
        self.label_13.setText(_translate("MainWindow", "Y -"))
        self.label_14.setText(_translate("MainWindow", "X +"))
        self.Z_core_j.setText(_translate("MainWindow", "0"))
        self.X_core_j.setText(_translate("MainWindow", "0"))
        self.Y_core_j.setText(_translate("MainWindow", "0"))
        self.X_core_j_2.setText(_translate("MainWindow", "Millimed"))
        self.lineEdit.setText(_translate("MainWindow", "0"))
        self.X_core_j_3.setText(_translate("MainWindow", "Setting (mm)"))
        self.selectbot.setText(_translate("MainWindow", "Select"))
        self.Restsetbot.setText(_translate("MainWindow", "Reset"))
        self.mannualMode.setText(_translate("MainWindow", "Mannual"))
        self.label_8.setText(_translate("MainWindow", "MODE"))
        self.jogMode.setText(_translate("MainWindow", "Jogging"))
        self.VSMode.setText(_translate("MainWindow", "Vision"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
