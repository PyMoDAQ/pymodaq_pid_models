import os
from pyqtgraph.parametertree import Parameter
from PyQt5.QtCore import pyqtSignal
from pymodaq.daq_utils.daq_utils import ThreadCommand, make_enum, PIDModelGeneric, check_modules
import time

DAQ_Viewer_Det_type=make_enum('daq_0Dviewer')
DAQ_Move_Stage_type=make_enum('daq_move')

class PIDModelMock(PIDModelGeneric):
    params = []

    status_sig = pyqtSignal(ThreadCommand)

    actuators = ['Mock']
    actuators_name = ['Axis test PID']
    for act in actuators:
        if act not in DAQ_Move_Stage_type.names('daq_move'):
            raise Exception('Invalid actuator plugin')

    detectors_type = ['DAQ0D'] # with entries either 'DAQ0D', 'DAQ1D' or 'DAQ2D'
    detectors = ['Mock']
    detectors_name = ['Testing PID']

    check_modules(detectors, detectors_type, actuators)

    def __init__(self, pid_controller):
        super().__init__(pid_controller)
        self.pid_controller = pid_controller
        self.water_temp = 20
        self.curr_time = time.perf_counter()



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
        self.pid_controller.detector_modules[0].settings.child('main_settings', 'wait_time').setValue(0)
        self.pid_controller.settings.child('move_settings', 'units').setValue('°C')

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
        self.curr_input = self.water_temp
        return self.water_temp

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

        if output > 0:
            # boiler can only produce heat, not cold
            pass
        self.water_temp += 1 * output * dt

        # some heat dissipation
        self.water_temp -= 0.02 * dt
        self.curr_output = output
        return [output]


if __name__ == '__main__':
    pass
