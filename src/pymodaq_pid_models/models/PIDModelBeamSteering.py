from pymodaq.pid.utils import PIDModelGeneric, OutputToActuator, InputFromDetector, main
from scipy.ndimage import center_of_mass


class PIDModelBeamSteering(PIDModelGeneric):
    limits = dict(max=dict(state=True, value=10),
                  min=dict(state=True, value=-10),)
    konstants = dict(kp=1, ki=0.01, kd=0.001)

    setpoint_ini = [128, 128]

    actuators_name = ["Xaxis", "Yaxis"]
    detectors_name = ['Camera']

    Nsetpoints = 2
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
        super().ini_model()

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
        self.curr_input = [y, x]
        return InputFromDetector([y, x])

    def convert_output(self, outputs, dt, stab=True):
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
        
        self.curr_output = outputs
        return OutputToActuator(mode='rel', values=outputs)




if __name__ == '__main__':
    main("BeamSteering.xml")


