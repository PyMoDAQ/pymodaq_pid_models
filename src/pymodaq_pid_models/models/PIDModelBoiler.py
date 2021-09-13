
from pymodaq.pid.utils import PIDModelGeneric, OutputToActuator, main


class PIDModelBoiler(PIDModelGeneric):

    limits = dict(max=dict(state=False, value=10),
                  min=dict(state=False, value=0), )
    konstants = dict(kp=3, ki=1, kd=0.0001)

    actuators_name = ["Heater"]
    detectors_name = ['Thermometer']

    setpoint_ini = [20]

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
        self.get_mod_from_name('Thermometer', 'det').settings.child('main_settings', 'wait_time').setValue(0)

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
        self.curr_input = measurements['Thermometer']['data0D']['Thermometer_Boiler_CH000']['data']

        return self.curr_input

    def convert_output(self, output, dt, stab=True):
        """

        """
        out_put_to_actuator = OutputToActuator('abs', values=[output])

        return out_put_to_actuator


if __name__ == '__main__':
    main("Boiler.xml")


