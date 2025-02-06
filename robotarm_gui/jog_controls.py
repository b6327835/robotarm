from time import sleep

class JogControls:
    def checkbottom(self):
        if self.x_post == 0:
            self.jogxd.setDisabled(1)
        else:
            self.jogxd.setEnabled(1)

        if self.y_post == 0:
            self.jogyd.setDisabled(1)
        else:
            self.jogyd.setEnabled(1)

        if self.z_post == 0:
            self.jogzd.setDisabled(1)
        else:
            self.jogzd.setEnabled(1)

        if self.x_post >= 300:
            self.jogxu.setDisabled(1)
            self.x_def = 300
            self.X_core_j.setNum(self.x_def)
        else:
            self.jogxu.setEnabled(1)

        if self.y_post >= 200:
            self.jogyu.setDisabled(1)
            self.y_def = 200
            self.Y_core_j.setNum(self.x_def)
        else:
            self.jogyu.setEnabled(1)

        if self.z_post >= 200:
            self.jogzu.setDisabled(1)
            self.z_def = 200
            self.Z_core_j.setNum(self.x_def)
        else:
            self.jogzu.setEnabled(1)

    def Rest(self):
        self.GetVal = 0
        self.lineEdit.setText("0")
        self.jogxu.setEnabled(1)
        self.jogxd.setEnabled(1)
        self.jogyu.setEnabled(1)
        self.jogyd.setEnabled(1)
        self.jogzu.setEnabled(1)
        self.jogzd.setEnabled(1)
        self.selectbot.setEnabled(1)
        self.lineEdit.setEnabled(1)

    def takeVal(self):
        try:
            value = int(self.lineEdit.text())
            if value > 10:
                value = 10
            elif value < 0:
                value = 0
            self.lineEdit.setText(str(value))
            self.GetVal = value
        except ValueError:
            self.GetVal = None

    def Setting(self):
        self.takeVal()
        if isinstance(self.GetVal, int):
            self.selectbot.setDisabled(1)
            self.lineEdit.setDisabled(1)
        else:
            self.lineEdit.setText("Try again.")
            self.jogxu.setDisabled(1)
            self.jogxu.setDisabled(1)
            self.jogxd.setDisabled(1)
            self.jogyu.setDisabled(1)
            self.jogyd.setDisabled(1)
            self.jogzu.setDisabled(1)
            self.jogzd.setDisabled(1)

    def Ux(self):
        self.x_def += int(self.GetVal)
        self.X_core_j.setNum(self.x_def)
        self.x_post = self.x_def
        print(self.x_def)
        self.horizontalSlider_1.setValue(self.x_def)

    def Dx(self):
        self.x_def -= int(self.GetVal)
        self.X_core_j.setNum(self.x_def)
        self.x_post = self.x_def
        print(self.x_def)
        self.horizontalSlider_1.setValue(self.x_def)

    def Uy(self):
        self.y_def += int(self.GetVal)
        self.Y_core_j.setNum(self.y_def)
        self.y_post = self.y_def
        print(self.y_def)
        self.horizontalSlider_2.setValue(self.y_def)

    def Dy(self):
        self.y_def -= int(self.GetVal)
        self.Y_core_j.setNum(self.y_def)
        self.y_post = self.y_def
        print(self.y_def)
        self.horizontalSlider_2.setValue(self.y_def)

    def Uz(self):
        self.z_def += int(self.GetVal)
        self.Z_core_j.setNum(self.z_def)
        self.z_post = self.z_def
        print(self.z_def)
        self.horizontalSlider_3.setValue(self.z_def)

    def Dz(self):
        self.z_def -= int(self.GetVal)
        self.Z_core_j.setNum(self.z_def)
        self.z_post = self.z_def
        print(self.z_def)
        self.horizontalSlider_3.setValue(self.z_def)

    def INITIAL_SET(self):
        self.horizontalSlider_1.setValue(0)
        self.horizontalSlider_2.setValue(0)
        self.horizontalSlider_3.setValue(0)

        self.x_post = self.horizontalSlider_1.value()
        self.y_post = self.horizontalSlider_2.value()
        self.z_post = self.horizontalSlider_3.value()

        self.x_post = 0
        self.y_post = 0
        self.z_post = 0

        self.x_def = 0
        self.y_def = 0
        self.z_def = 0

        self.X_core_j.setText(str(0))
        self.Y_core_j.setText(str(0))
        self.Z_core_j.setText(str(0))
        print(" ")
        print(f"X,Y,Z: {self.x_post},{self.y_post},{self.z_post}")
        print("INITIAL Check")

    def HOME_SET(self):
        self.horizontalSlider_1.setValue(150)
        self.horizontalSlider_2.setValue(100)
        self.horizontalSlider_3.setValue(100)

        self.x_post = self.horizontalSlider_1.value()
        self.y_post = self.horizontalSlider_2.value()
        self.z_post = self.horizontalSlider_3.value()

        self.x_post = 150
        self.y_post = 100
        self.z_post = 100

        self.X_core_j.setText(str(150))
        self.Y_core_j.setText(str(150))
        self.Z_core_j.setText(str(150))

        print(" ")
        print(f"X,Y,Z: {self.x_post},{self.y_post},{self.z_post}")
        print("HOME Check")

    def START_SET(self):
        self.x_post = self.horizontalSlider_1.value()
        self.y_post = self.horizontalSlider_2.value()
        self.z_post = self.horizontalSlider_3.value()
        print(" ")
        if self.x_post == 0 & self.y_post == 0 & self.z_post == 0:
            self.xp = "3x" + str(0)
            self.yp = "3y" + str(0)
            self.zp = "3z" + str(0)
            self.setINITIAL = self.xp + "," + self.yp + "," + self.zp
            sleep(0.2)
            print(self.setINITIAL)
            print("Set Initial")
        else:
            self.xpoint()
            self.ypoint()
            self.zpoint()
            self.setpost = self.xp + "," + self.yp + "," + self.zp
            sleep(0.2)
            print(self.setpost)

            print("Set Positions")

    def X_SET(self):
        self.x_vale = self.xpos_slider.value()
        self.X_core_j.setText(str(self.x_vale))
        print(f"X : {self.x_vale}")

    def Y_SET(self):
        self.y_vale = self.ypos_slider.value()
        self.Y_core_j.setText(str(self.y_vale))
        print(f"Y : {self.y_vale}")

    def Z_SET(self):
        self.z_vale = self.zpos_slider.value()
        self.Z_core_j.setText(str(self.z_vale))
        print(f"Z : {self.z_vale}")

    def xpoint(self):
        if self.x_post >= self.x_def:
            self.x_def = self.x_post - self.x_def
            print(f"X : +{self.x_def}")
            self.x_def = self.x_post
            self.xp = "1x" + str(self.x_def)

        elif self.x_post <= self.x_def:
            self.x_def = abs(self.x_post - self.x_def)
            print(f"X : -{self.x_def}")
            self.x_def = self.x_post
            self.xp = "2x" + str(self.x_def)

    def ypoint(self):
        if self.y_post >= self.y_def:
            self.y_def = self.y_post - self.y_def
            print(f"Y : +{self.y_def}")
            self.y_def = self.y_post
            self.yp = "1y" + str(self.y_def)

        elif self.y_post <= self.y_def:
            self.y_def = abs(self.y_post - self.y_def)
            print(f"Y : -{self.y_def}")
            self.y_def = self.y_post
            self.yp = "2y" + str(self.y_def)

    def zpoint(self):
        if self.z_post >= self.z_def:
            self.z_def = self.z_post - self.z_def
            print(f"Z : +{self.z_def}")
            self.z_def = self.z_post
            self.zp = "1z" + str(self.z_def)

        elif self.z_post <= self.x_def:
            self.z_def = abs(self.z_post - self.z_def)
            print(f"Z : -{self.z_def}")
            self.z_def = self.z_post
            self.zp = "2z" + str(self.z_def)
            
class MoveLControls:
    def _slider_to_real(self, slider_value):
        """Convert slider integer (0-300000) to real position (0-300.000)"""
        return slider_value * 0.001

    def _real_to_slider(self, real_value):
        """Convert real position (0-300.000) to slider integer (0-300000)"""
        return int(real_value * 1000)

    def moveL(self, direction):
        if direction == "up":
            current = self.xpos_target_slider.value()
            increment = self._real_to_slider(self.move_resolution_spinbox.value())
            self.xpos_target_slider.setValue(current + increment)
            self.xpos_target_label.setText(f"{self._slider_to_real(self.xpos_target_slider.value()):.3f}")
            print(f"Move up by {self.move_resolution_spinbox.value():.3f}mm")
        elif direction == "down":
            print("Move down")
        elif direction == "left":
            print("Move left")
        elif direction == "right":
            print("Move right")
        elif direction == "upleft":
            print("Move up-left")
        elif direction == "upright":
            print("Move up-right")
        elif direction == "downleft":
            print("Move down-left")
        elif direction == "downright":
            print("Move down-right")
        else:
            print("Invalid direction specified")
            
    def update_current_position(self):
        self.xpos_current_label.setText(f"{self._slider_to_real(self.xpos_current_slider.value()):.3f}")
        self.ypos_current_label.setText(f"{self._slider_to_real(self.ypos_current_slider.value()):.3f}")
        self.zpos_current_label.setText(f"{self._slider_to_real(self.zpos_current_slider.value()):.3f}")