# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'offloading_controls.ui'
#
# Created: Tue Feb  3 00:27:00 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from offloadable_face_recognition.msg import OffloadCommand

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Offloading_Controls_Widget(QtGui.QWidget):

    def __init__(self):
        QtGui.QWidget.__init__(self)
        self.isAutomatic = True
        self.AUTOMATIC_COMMAND = True
        self.MANUAL_COMMAND = False
        self.NO_OFFLOAD_PERCENTAGE = 0
        self.scheduler_commands = rospy.Publisher("scheduler_commands", OffloadCommand, queue_size=1)
        setupUi(offloading_controls_widget)


    def setupUi(self, offloading_controls_widget):
        offloading_controls_widget.setObjectName(_fromUtf8("offloading_controls_widget"))
        offloading_controls_widget.resize(480, 160)
        self.horizontalLayout = QtGui.QHBoxLayout(offloading_controls_widget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.main_layout_view = QtGui.QGridLayout()
        self.main_layout_view.setObjectName(_fromUtf8("main_layout_view"))
        self.offloading_slider_view = QtGui.QVBoxLayout()
        self.offloading_slider_view.setObjectName(_fromUtf8("offloading_slider_view"))
        self.line = QtGui.QFrame(offloading_controls_widget)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.offloading_slider_view.addWidget(self.line)
        self.offloading_percentage_label = QtGui.QLabel(offloading_controls_widget)
        self.offloading_percentage_label.setObjectName(_fromUtf8("offloading_percentage_label"))
        self.offloading_slider_view.addWidget(self.offloading_percentage_label)
        self.offloading_percentage_slider = QtGui.QSlider(offloading_controls_widget)
        self.offloading_percentage_slider.setOrientation(QtCore.Qt.Horizontal)
        self.offloading_percentage_slider.setObjectName(_fromUtf8("offloading_percentage_slider"))
        self.offloading_slider_view.addWidget(self.offloading_percentage_slider)
        self.p = QtGui.QVBoxLayout()
        self.p.setObjectName(_fromUtf8("p"))
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.percentage_0_label = QtGui.QLabel(offloading_controls_widget)
        self.percentage_0_label.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignTop)
        self.percentage_0_label.setObjectName(_fromUtf8("percentage_0_label"))
        self.horizontalLayout_4.addWidget(self.percentage_0_label)
        self.percentage_50_label = QtGui.QLabel(offloading_controls_widget)
        self.percentage_50_label.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.percentage_50_label.setObjectName(_fromUtf8("percentage_50_label"))
        self.horizontalLayout_4.addWidget(self.percentage_50_label)
        self.percentage_100_label = QtGui.QLabel(offloading_controls_widget)
        self.percentage_100_label.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTop|QtCore.Qt.AlignTrailing)
        self.percentage_100_label.setObjectName(_fromUtf8("percentage_100_label"))
        self.horizontalLayout_4.addWidget(self.percentage_100_label)
        self.p.addLayout(self.horizontalLayout_4)
        self.offloading_slider_view.addLayout(self.p)
        self.main_layout_view.addLayout(self.offloading_slider_view, 1, 0, 1, 1)
        self.offloading_type_view_2 = QtGui.QVBoxLayout()
        self.offloading_type_view_2.setObjectName(_fromUtf8("offloading_type_view_2"))
        self.offloading_type_label = QtGui.QLabel(offloading_controls_widget)
        self.offloading_type_label.setObjectName(_fromUtf8("offloading_type_label"))
        self.offloading_type_view_2.addWidget(self.offloading_type_label)
        self.radio_button_view = QtGui.QHBoxLayout()
        self.radio_button_view.setObjectName(_fromUtf8("radio_button_view"))
        self.automatic_offloading_btn = QtGui.QRadioButton(offloading_controls_widget)
        self.automatic_offloading_btn.setObjectName(_fromUtf8("automatic_offloading_btn"))
        self.radio_button_view.addWidget(self.automatic_offloading_btn)
        self.manual_offloading_btn = QtGui.QRadioButton(offloading_controls_widget)
        self.manual_offloading_btn.setObjectName(_fromUtf8("manual_offloading_btn"))
        self.radio_button_view.addWidget(self.manual_offloading_btn)
        self.offloading_type_view_2.addLayout(self.radio_button_view)
        self.main_layout_view.addLayout(self.offloading_type_view_2, 0, 0, 1, 1)
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.main_layout_view.addLayout(self.verticalLayout_4, 2, 0, 1, 1)
        self.horizontalLayout.addLayout(self.main_layout_view)
        self.retranslateUi(offloading_controls_widget)
        QtCore.QMetaObject.connectSlotsByName(offloading_controls_widget)

    def retranslateUi(self, offloading_controls_widget):
        offloading_controls_widget.setWindowTitle(_translate("offloading_controls_widget", "Offloading Controls", None))
        self.offloading_percentage_label.setText(_translate("offloading_controls_widget", "Offloading Percentage", None))
        self.percentage_0_label.setText(_translate("offloading_controls_widget", "0%", None))
        self.percentage_50_label.setText(_translate("offloading_controls_widget", "50%", None))
        self.percentage_100_label.setText(_translate("offloading_controls_widget", "100%", None))
        self.offloading_type_label.setText(_translate("offloading_controls_widget", "Offloading Type", None))
        self.automatic_offloading_btn.setText(_translate("offloading_controls_widget", "Automatic", None))
        self.manual_offloading_btn.setText(_translate("offloading_controls_widget", "Manual", None))

        self.manual_offloading_btn.clicked.connect(self.set_manual_offloading)
        self.automatic_offloading_btn.clicked.connect(self.set_automatic_offloading)
        self.offloading_percentage_slider.setTickInterval(25)
        self.offloading_percentage_slider.setEnabled(False)
        self.offloading_percentage_slider.setMaximum(100)
        self.offloading_percentage_slider.valueChanged(self.slider_value_updated)

    def set_manual_offloading(self):
        if isAutomatic:
            self.offloading_percentage_slider.setEnabled(True)
            self.isAutomatic = False
            self.publish_offload_command(self.MANUAL_COMMAND, self.offloading_percentage_slider.value)


    def set_automatic_offloading(self):
        if not isAutomatic:
            self.offloading_percentage_slider.setEnabled(False)
            self.isAutomatic = True
            self.publish_offload_command(self.AUTOMATIC_COMMAND, self.NO_OFFLOAD_PERCENTAGE)

    def slider_value_updated(self):
        if not isAutomatic:
            self.publish_offload_command(self.MANUAL_COMMAND, self.offloading_percentage_slider.value)

    def publish_offload_command(self, type, percentage):
            command = OffloadCommand()
            command.type = type
            command.percentage = percentage
            self.scheduler_commands.publish(command)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    widget = Ui_Offloading_Controls_Widget()
    widget.show()
    sys.exit(app.exec_())