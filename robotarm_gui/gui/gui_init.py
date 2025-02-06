from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import Qt

class GUIInitializer:
    def gnc(self):
        self.stackedWidget.addWidget(self.manpage)
        self.stackedWidget.addWidget(self.jogpage)
        self.stackedWidget.addWidget(self.visionpage)

        self.mannualMode.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.manpage))
        self.jogMode.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.jogpage))
        self.VSMode.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.visionpage))
        self.Main_control_button.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.controlpage))

        self.Initial_bottom.clicked.connect(self.INITIAL_SET)
        self.home_bottom.clicked.connect(self.HOME_SET)
        self.start_bottom.clicked.connect(self.START_SET)

        self.xpos_slider.sliderReleased.connect(self.X_SET)
        self.ypos_slider.sliderReleased.connect(self.Y_SET)
        self.zpos_slider.sliderReleased.connect(self.Z_SET)

        self.jogxu.clicked.connect(self.Ux)
        self.jogxd.clicked.connect(self.Dx)

        self.jogyu.clicked.connect(self.Uy)
        self.jogyd.clicked.connect(self.Dy)

        self.jogzu.clicked.connect(self.Uz)
        self.jogzd.clicked.connect(self.Dz)

        self.selectbot.clicked.connect(self.Setting)
        self.Restsetbot.clicked.connect(self.Rest)
        self.lineEdit.returnPressed.connect(self.takeVal)
        
        self.Gopoint.clicked.connect(self.move_to)
        self.pick_target_bt.clicked.connect(self.picktarget)
        self.pnp_bt.clicked.connect(self.to_pnp)
        self.home_bt.clicked.connect(self.home)
        self.auto_bt.clicked.connect(self.move_to_grid_position)
        self.moveto_bt.clicked.connect(self.show_position_selector)
        
        #control page
        self.moveup_btn.clicked.connect(lambda: self.moveL("up"))
        self.movedown_btn.clicked.connect(lambda: self.moveL("down"))
        self.moveleft_btn.clicked.connect(lambda: self.moveL("left"))
        self.moveright_btn.clicked.connect(lambda: self.moveL("right"))
        self.moveup_left_btn.clicked.connect(lambda: self.moveL("upleft"))
        self.moveup_right_btn.clicked.connect(lambda: self.moveL("upright"))
        self.movedown_left_btn.clicked.connect(lambda: self.moveL("downleft"))
        self.movedown_right_btn.clicked.connect(lambda: self.moveL("downright"))
        
        self.xpos_current_slider.valueChanged.connect(self.update_current_position)
        self.ypos_current_slider.valueChanged.connect(self.update_current_position)
        self.zpos_current_slider.valueChanged.connect(self.update_current_position)
        
        self.auto_bt_2.clicked.connect(self.on_auto_bt_2_clicked)

