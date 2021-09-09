from pymodaq_pid.utils import PIDModelGeneric, OutputToActuator
from scipy.ndimage import center_of_mass


class PIDModelBeamSteering(PIDModelGeneric):
    limits = dict(max=dict(state=True, value=10),
                  min=dict(state=True, value=-10),)
    konstants = dict(kp=1, ki=0.01, kd=0.001)

    actuators_name = ["Xaxis"]
    detectors_name = ['Camera']
    Nsetpoint = 1
    params = [{'title': 'Threshold', 'name': 'threshold', 'type': 'float', 'value': 10.}]

    def __init__(self, pid_controller):
        super().__init__(pid_controller)

    def update_settings(self, param):
        """
        Get a parameter instance whose value has been modified by a user on the UI
        Parameters
        ----------
        param: (Parameter) instance of Parameter object
        """
        if param.name() == '':
            pass

    def ini_model(self):
        pass
        # self.get_mod_from_name('Camera', 'det').settings.child('main_settings', 'wait_time').setValue(0)
        # self.get_mod_from_name('Xaxis', 'act').settings.child('move_settings', 'units').setValue('°C')

    def convert_input(self, measurements):
        """
        Convert the measurements in the units to be fed to the PID (same dimensionality as the setpoint)
        Parameters
        ----------
        measurements: (Ordereddict) Ordereded dict of object from which the model extract a value of the same units as the setpoint

        Returns
        -------
        float: the converted input

        """
        #print('input conversion done')
        image = measurements['Camera']['data2D']['Camera_Mock2DPID_CH000']['data']
        image = image - self.settings.child('threshold').value()
        image[image < 0] = 0
        x, y = center_of_mass(image)
        self.curr_input = y
        return y

    def convert_output(self, output, dt, stab=True):
        """
        Convert the output of the PID in units to be fed into the actuator
        Parameters
        ----------
        output: (float) output value from the PID from which the model extract a value of the same units as the actuator

        Returns
        -------
        list: the converted output as a list (if there are a few actuators)

        """
        #print('output converted')
        
        self.curr_output = output
        return OutputToActuator(mode='rel', values=[output])


def main():
    from pymodaq.dashboard import DashBoard
    from pymodaq.daq_utils.daq_utils import get_set_preset_path
    from pymodaq.daq_utils import gui_utils as gutils
    from pathlib import Path
    from PyQt5 import QtWidgets
    from pymodaq_pid.pid_controller import DAQ_PID

    import sys
    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QMainWindow()
    area = gutils.DockArea()
    win.setCentralWidget(area)
    win.resize(1000, 500)
    win.setWindowTitle('PyMoDAQ Dashboard')

    dashboard = DashBoard(area)
    file = Path(get_set_preset_path()).joinpath("BeamSteering.xml")
    if file.exists():
        dashboard.set_preset_mode(file)
        # prog.load_scan_module()
        pid_area = gutils.DockArea()
        pid_window = QtWidgets.QMainWindow()
        pid_window.setCentralWidget(pid_area)

        prog = DAQ_PID(pid_area, dashboard.modules_manager)
        pid_window.show()
        pid_window.setWindowTitle('PidController')
        QtWidgets.QApplication.processEvents()


    else:
        msgBox = QtWidgets.QMessageBox()
        msgBox.setText(f"The default file specified in the configuration file does not exists!\n"
                       f"{file}\n"
                       f"Impossible to load the DAQ_PID Module")
        msgBox.setStandardButtons(msgBox.Ok)
        ret = msgBox.exec()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()


