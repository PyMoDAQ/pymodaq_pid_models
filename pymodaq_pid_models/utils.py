from PyQt5.QtCore import pyqtSignal
from pymodaq.daq_utils.daq_utils import ThreadCommand, get_names_simple


class PIDModelGeneric:
    params = []

    status_sig = pyqtSignal(ThreadCommand)
    actuators = []
    actuators_name = []
    detectors_type = []  # with entries either 'DAQ0D', 'DAQ1D' or 'DAQ2D'
    detectors = []
    detectors_name = []

    def __init__(self, pid_controller):
        self.pid_controller = pid_controller  # instance of the pid_controller using this model
        self.settings = self.pid_controller.settings.child('models', 'model_params')  # set of parameters
        self.data_names = None
        self.curr_output = None
        self.curr_input = None

    def update_detector_names(self):
        names = self.pid_controller.settings.child('main_settings', 'detector_modules').value()['selected']
        self.data_names = []
        for name in names:
            name = name.split('//')
            self.data_names.append(name)


    def update_settings(self, param):
        """
        Get a parameter instance whose value has been modified by a user on the UI
        To be overwritten in child class
        """
        if param.name() == '':
            pass

    def ini_model(self):
        pass

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
        return 0

    def convert_output(self, output, dt):
        """
        Convert the output of the PID in units to be fed into the actuator
        Parameters
        ----------
        output: (float) output value from the PID from which the model extract a value of the same units as the actuator
        dt: (float) ellapsed time in seconds since last call
        Returns
        -------
        list: the converted output as a list (in case there are a few actuators)

        """
        #print('output converted')

        return [output]

def check_modules(detectors, detectors_type, actuators):
    for ind_det, det in enumerate(detectors):
        if detectors_type[ind_det] == 'DAQ0D':
            if det not in get_names_simple('daq_0Dviewer'):
                raise Exception('Invalid actuator plugin')
        elif detectors_type[ind_det] == 'DAQ1D':
            if det not in get_names_simple('daq_1Dviewer'):
                raise Exception('Invalid actuator plugin')
        elif detectors_type[ind_det] == 'DAQ2D':
            if det not in get_names_simple('daq_2Dviewer'):
                raise Exception('Invalid actuator plugin')

    for act in actuators:
        if act not in get_names_simple('daq_move'):
            raise Exception('Invalid actuator plugin')