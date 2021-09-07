import os
from pyqtgraph.parametertree import Parameter
from PyQt5.QtCore import pyqtSignal
from pymodaq.daq_utils.daq_utils import ThreadCommand
from ..utils import PIDModelGeneric
import time

import numpy as np
from scipy.ndimage import center_of_mass


class PIDModelBeamSteering(PIDModelGeneric):

    actuators_name = ["Xaxis"]
    detectors_name = ['Camera']
    Nsetpoint = 1
    params = [{'title': 'Threshold', 'name': 'threshold', 'type': 'float', 'value': 10.}]

    def __init__(self, pid_controller):
        super().__init__(pid_controller)
        self.pid_controller = pid_controller
        self.curr_time = time.perf_counter()
        self.get_mod_from_name = pid_controller.module_manager.get_mod_from_name


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
        return [self.curr_input+self.curr_output if self.curr_input is not None else self.curr_output]


if __name__ == '__main__':
    pass
