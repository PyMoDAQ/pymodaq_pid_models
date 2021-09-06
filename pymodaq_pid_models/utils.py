from PyQt5.QtCore import pyqtSignal
from pymodaq.daq_utils.daq_utils import ThreadCommand, get_plugins, set_logger, get_module_name

logger = set_logger(get_module_name(__file__))

DAQ_Move_Stage_type = get_plugins('daq_move')
DAQ_0DViewer_Det_types = get_plugins('daq_0Dviewer')
DAQ_1DViewer_Det_types = get_plugins('daq_1Dviewer')
DAQ_2DViewer_Det_types = get_plugins('daq_2Dviewer')
DAQ_NDViewer_Det_types = get_plugins('daq_NDviewer')


class PIDModelGeneric:
    params = []

    status_sig = pyqtSignal(ThreadCommand)
    actuators_name = []
    detectors_name = []

    def __init__(self, pid_controller):
        self.pid_controller = pid_controller  # instance of the pid_controller using this model
        self.settings = self.pid_controller.settings.child('models', 'model_params')  # set of parameters
        self.data_names = None
        self.curr_output = None
        self.curr_input = None

        self.check_modules(pid_controller.module_manager)

    def check_modules(self, module_manager):
        for act in self.actuators_name:
            if act not in module_manager.actuators_name:
                logger.warning(f'The actuator {act} defined in the PID model is not present in the Dashboard')
                return False
        for det in self.detectors_name:
            if det not in module_manager.detectors_name:
                logger.warning(f'The detector {det} defined in the PID model is not present in the Dashboard')

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

