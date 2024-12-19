from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import Qt

class GUIInitializer:
    def gnc(self):
        self.stackedWidget.addWidget(self.manpage)
        self.stackedWidget.addWidget(self.jogpage)
        self.stackedWidget.addWidget(self.visionpage)

        self.mannualMode.clicked.connect(
            lambda: self.stackedWidget.setCurrentWidget(self.manpage)
        )
        self.jogMode.clicked.connect(
            lambda: self.stackedWidget.setCurrentWidget(self.jogpage)
        )
        self.VSMode.clicked.connect(
            lambda: self.stackedWidget.setCurrentWidget(self.visionpage)
        )

        self.Initial_bottom.clicked.connect(self.INITIAL_SET)
        self.home_bottom.clicked.connect(self.HOME_SET)
        self.start_bottom.clicked.connect(self.START_SET)

        self.horizontalSlider_1.sliderReleased.connect(self.X_SET)
        self.horizontalSlider_2.sliderReleased.connect(self.Y_SET)
        self.horizontalSlider_3.sliderReleased.connect(self.Z_SET)

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
        self.auto_bt.clicked.connect(self.auto_pnp)
