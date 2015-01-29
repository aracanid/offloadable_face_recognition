# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/justin/offloadable_face_recognition_gui/offloadingwindow.ui'
#
# Created: Wed Jan 28 13:38:34 2015
#      by: PyQt5 UI code generator 5.2.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from qt_gui.plugin import Plugin

class Ui_MainWindow(Plugin):

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("Offloading GUI")
        MainWindow.resize(400, 260)
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")
        self.horizontalSlider = QtWidgets.QSlider(self.centralWidget)
        self.horizontalSlider.setGeometry(QtCore.QRect(10, 100, 351, 51))
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.radioButton = QtWidgets.QRadioButton(self.centralWidget)
        self.radioButton.setGeometry(QtCore.QRect(10, 40, 117, 22))
        self.radioButton.setObjectName("radioButton")
        self.buttonGroup = QtWidgets.QButtonGroup(MainWindow)
        self.buttonGroup.setObjectName("buttonGroup")
        self.buttonGroup.addButton(self.radioButton)
        self.radioButton_2 = QtWidgets.QRadioButton(self.centralWidget)
        self.radioButton_2.setGeometry(QtCore.QRect(130, 40, 117, 22))
        self.radioButton_2.setObjectName("radioButton_2")
        self.buttonGroup.addButton(self.radioButton_2)
        self.label = QtWidgets.QLabel(self.centralWidget)
        self.label.setGeometry(QtCore.QRect(10, 10, 161, 31))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralWidget)
        self.label_2.setGeometry(QtCore.QRect(10, 70, 181, 31))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.centralWidget)
        self.label_3.setGeometry(QtCore.QRect(10, 140, 31, 31))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.centralWidget)
        self.label_4.setGeometry(QtCore.QRect(170, 140, 41, 31))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.centralWidget)
        self.label_5.setGeometry(QtCore.QRect(320, 140, 41, 31))
        self.label_5.setObjectName("label_5")
        MainWindow.setCentralWidget(self.centralWidget)
        self.mainToolBar = QtWidgets.QToolBar(MainWindow)
        self.mainToolBar.setObjectName("mainToolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        MainWindow.insertToolBarBreak(self.mainToolBar)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)
        self.menuBar = QtWidgets.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 434, 25))
        self.menuBar.setObjectName("menuBar")
        self.menuOffloading = QtWidgets.QMenu(self.menuBar)
        self.menuOffloading.setObjectName("menuOffloading")
        MainWindow.setMenuBar(self.menuBar)
        self.menuBar.addAction(self.menuOffloading.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.radioButton.setText(_translate("MainWindow", "Automatic"))
        self.radioButton_2.setText(_translate("MainWindow", "Manual"))
        self.label.setText(_translate("MainWindow", "Offloading"))
        self.label_2.setText(_translate("MainWindow", "Offloading Percentage"))
        self.label_3.setText(_translate("MainWindow", "0%"))
        self.label_4.setText(_translate("MainWindow", "50%"))
        self.label_5.setText(_translate("MainWindow", "100%"))
        self.menuOffloading.setTitle(_translate("MainWindow", "Offloading Controls"))

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

def main(self):
    
