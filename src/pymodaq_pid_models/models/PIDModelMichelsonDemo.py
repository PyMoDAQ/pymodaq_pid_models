import os
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, QTimer, pyqtSlot
from pyqtgraph.dockarea import Dock
from pymodaq.daq_utils.daq_utils import ThreadCommand, linspace_step, get_set_pid_path
from pymodaq.pid.utils import PIDModelGeneric, OutputToActuator, main
from pymodaq.daq_utils.plotting.viewer1D.viewer1D_main import Viewer1D
from pymodaq.daq_utils.math_utils import LSqEllipse
import time
import numpy as np
from collections import OrderedDict


class PIDModelMichelsonDemo(PIDModelGeneric):
    params = [
        {'title': 'Stabilization', 'name': 'stabilization', 'type': 'group', 'children': [
            {'title': 'Set as zero:', 'name': 'set_zero', 'type': 'bool', 'value': False},
            {'title': 'Laser wavelength (nm):', 'name': 'laser_wl', 'type': 'float', 'value': 632.0},
            {'title': 'Interfero phase (deg):', 'name': 'interfero_phase', 'type': 'float', 'value': 0},
            {'title': 'Interfero position (nm):', 'name': 'interfero_position', 'type': 'float', 'value': 0},
            {'title': 'phase sign:', 'name': 'phase_sign', 'type': 'int', 'value': 1, 'min': -1, 'max': 1, 'step': 2},
            {'title': 'Offsets', 'name': 'offsets', 'type': 'group', 'children': [
                {'title': 'Phase offset (deg):', 'name': 'phase_offset', 'type': 'float', 'value': 0},
                {'title': 'Interfero (nm):', 'name': 'interfero_offset', 'type': 'float', 'value': 0},
                {'title': 'Piezo (nm):', 'name': 'piezo_offset', 'type': 'float', 'value': 0},
            ]},
        ]},
        {'title': 'Calibration', 'name': 'calibration', 'type': 'group', 'expanded': True, 'visible': True, 'children': [
            {'title': 'Do calibration:', 'name': 'do_calibration', 'type': 'bool', 'value': False},
            {'title': 'Timeout:', 'name': 'timeout', 'type': 'int', 'value': 10000},

            {'title': 'Calibration', 'name': 'calibration_move', 'type': 'group', 'expanded': True, 'visible': True, 'children': [
                {'title': 'Start pos:', 'name': 'start', 'type': 'float', 'value': 7.},
                {'title': 'Stop pos:', 'name': 'stop', 'type': 'float', 'value': 8.},
                {'title': 'Step size:', 'name': 'step', 'type': 'float', 'value': 0.04},
                {'title': 'Averaging:', 'name': 'average', 'type': 'int', 'value': 1},
                ]},
            {'title': 'Ellipse params', 'name': 'calibration_ellipse', 'type': 'group', 'expanded': True, 'visible': True,
             'children': [
                 {'title': 'Dx:', 'name': 'dx', 'type': 'float', 'value': 0.0},
                 {'title': 'Dy:', 'name': 'dy', 'type': 'float', 'value': 0.0},
                 {'title': 'x0:', 'name': 'x0', 'type': 'float', 'value': 0.00},
                 {'title': 'y0:', 'name': 'y0', 'type': 'float', 'value': 0.00},
                 {'title': 'phi (°):', 'name': 'theta', 'type': 'float', 'value': 0.00},
             ]},
        ]},


    ]

    actuators = ['PI']
    actuators_name = ['Axis test PID']

    detectors_type = ['DAQ2D']  # with entries either 'DAQ0D', 'DAQ1D' or 'DAQ2D'or 'DAQND'
    detectors = ['OpenCVCam']
    detectors_name = ['Testing PID']

    def __init__(self, pid_controller):
        super().__init__(pid_controller)
        self.running = False
        self.det_done_flag = False
        self.move_done_flag = False
        self.timeout_scan_flag = False

        self.curr_time = time.perf_counter()

        self.lsqe = LSqEllipse()

        self.phases = np.zeros((100,))
        self.curr_phase = 0
        self.curr_position = 0

        self.dock_calib = Dock('Calibration')
        widget_calib = QtWidgets.QWidget()
        self.viewer_calib = Viewer1D(widget_calib)
        widget_ellipse = QtWidgets.QWidget()
        self.viewer_ellipse = Viewer1D(widget_ellipse)
        self.viewer_ellipse.show_data([np.zeros((10,)), np.zeros((10,))])
        self.dock_calib.addWidget(widget_calib)
        self.dock_calib.addWidget(widget_ellipse, row=0, col = 1)
        self.pid_controller.dock_area.addDock(self.dock_calib)
        self.dock_calib.float()

        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.timeout)

    def ini_model(self):
        """
        Defines all the action to be performed on the initialized modules (main pid, actuators, detectors)
        Either here for specific things (ROI, ...) or within the preset of the current model

        """
        model_name = self.pid_controller.settings.child('models', 'model_class').value()

        self.pid_controller.detector_modules[0].ui.viewers[0].ui.roiBtn.click()

        #try to get corresponding roi file
        filename = os.path.join(get_set_pid_path(), model_name + '.roi2D')
        if os.path.isfile(filename):
            self.pid_controller.detector_modules[0].ui.viewers[0].load_ROI(filename)

        QtWidgets.QApplication.processEvents()
        if self.pid_controller.detector_modules[0].Initialized_state:
            #self.pid_controller.detector_modules[0].settings.child('main_settings', 'wait_time').setValue(0)
            self.pid_controller.detector_modules[0].grab_data()
        QThread.msleep(500)
        QtWidgets.QApplication.processEvents()

        self.pid_controller.settings.child('main_settings', 'update_modules').setValue(True)
        QtWidgets.QApplication.processEvents()
        QThread.msleep(500)
        QtWidgets.QApplication.processEvents()

        items = self.pid_controller.settings.child('main_settings', 'detector_modules').value().copy()
        if items is not None:
            items['selected'] = items['all_items'][0:2]
            self.pid_controller.settings.child('main_settings', 'detector_modules').setValue(items)

        self.pid_controller.settings.child('main_settings', 'pid_controls', 'output_limits',
                                           'output_limit_min').setValue(0) #in microns
        self.pid_controller.settings.child('main_settings', 'pid_controls', 'output_limits',
                                           'output_limit_min_enabled').setValue(True)
        self.pid_controller.settings.child('main_settings', 'pid_controls', 'output_limits',
                                           'output_limit_max').setValue(15) #in microns
        self.pid_controller.settings.child('main_settings', 'pid_controls', 'output_limits',
                                           'output_limit_max_enabled').setValue(True)

        self.pid_controller.settings.child('main_settings', 'pid_controls', 'pid_constants', 'kp').setValue(0.5)
        self.pid_controller.settings.child('main_settings', 'pid_controls', 'pid_constants', 'ki').setValue(0.2)
        self.pid_controller.settings.child('main_settings', 'pid_controls', 'pid_constants', 'kd').setValue(0.0001)

        self.pid_controller.settings.child('main_settings', 'pid_controls', 'filter', 'filter_enable').setValue(True)
        self.pid_controller.settings.child('main_settings', 'pid_controls', 'filter',
               'filter_step').setValue(self.settings.child('stabilization', 'laser_wl').value()/1000/6/2) #in microns

        self.pid_controller.setpoint_sb.setValue(7.5)
        self.pid_controller.settings.child('main_settings', 'pid_controls', 'sample_time').setValue(50)

    def get_ellipse_fit(self, center, width, height, theta):

        t = np.linspace(0, 2 * np.pi, 1000)
        ellipse_x = center[0] + width * np.cos(t) * np.cos(theta) - height * np.sin(t) * np.sin(theta)
        ellipse_y = center[1] + width * np.cos(t) * np.sin(theta) + height * np.sin(t) * np.cos(theta)

        return ellipse_x, ellipse_y




    def update_settings(self, param):
        """
        Get a parameter instance whose value has been modified by a user on the UI
        Parameters
        ----------
        param: (Parameter) instance of Parameter object
        """
        if param.name() == 'do_calibration':
            if param.value():
                self.running = True
                self.do_calibration()
                QtWidgets.QApplication.processEvents()

            else:
                self.running = False
        elif param.name() == 'set_zero':
            self.phases = np.zeros((100,))
            position = self.pid_controller.actuator_modules[0].current_position
            self.settings.child('stabilization', 'offsets', 'phase_offset').setValue(self.curr_phase)
            self.settings.child('stabilization', 'offsets', 'interfero_offset').setValue(position*1000)
            self.settings.child('stabilization', 'offsets', 'piezo_offset').setValue(position*1000)
            data = self.pid_controller.detector_modules[0].data_to_save_export
            data_export = OrderedDict()
            data_export[data['name']] = data
            self.pid_controller.command_pid.emit(ThreadCommand('input', [data_export]))
            QtWidgets.QApplication.processEvents()
            self.settings.child('stabilization', 'set_zero').setValue(False)



    def do_calibration(self):
        Naverage = self.settings.child('calibration', 'calibration_move', 'average').value()

        steps = linspace_step(self.settings.child('calibration', 'calibration_move', 'start').value(),
                              self.settings.child('calibration', 'calibration_move', 'stop').value(),
                              self.settings.child('calibration', 'calibration_move', 'step').value())

        self.detector_data = np.zeros((steps.size,2))
        self.detector_data_average = np.zeros((steps.size, 2))

        for mod in self.pid_controller.actuator_modules:
            mod.move_done_signal.connect(self.move_done)
        for mod in self.pid_controller.detector_modules:
            mod.grab_done_signal.connect(self.det_done)

        self.viewer_calib.x_axis = steps




        for ind_average in range(Naverage):
            for ind_step, step in enumerate(steps):
                self.move_done_flag = False
                self.pid_controller.actuator_modules[0].move_Abs(step)
                self.wait_for_move_done()

                self.det_done_flag = False
                self.pid_controller.detector_modules[0].grab_data()
                self.wait_for_det_done()
                for ind_data, name in enumerate(self.data_names):
                    self.detector_data[ind_step, ind_data] = \
                    self.pid_controller.detector_modules[0].data_to_save_export[name[1]][name[2]]
                if not self.running:
                    break
            self.detector_data_average = (ind_average*self.detector_data_average+self.detector_data)/(ind_average+1)
            self.viewer_calib.show_data([self.detector_data_average[:,0], self.detector_data_average[:,1]])

            self.lsqe.fit([self.detector_data_average[:,0], self.detector_data_average[:,1]])
            center, width, height, theta = self.lsqe.parameters()
            ellipse_x, ellipse_y = self.get_ellipse_fit(center, width, height, theta)

            self.viewer_ellipse.plot_channels[0].setData(x=self.detector_data_average[:,0], y=self.detector_data_average[:,1])
            self.viewer_ellipse.plot_channels[1].setData(x=ellipse_x, y=ellipse_y)

            self.settings.child('calibration', 'calibration_ellipse', 'x0').setValue(center[0])
            self.settings.child('calibration', 'calibration_ellipse', 'y0').setValue(center[1])
            self.settings.child('calibration', 'calibration_ellipse', 'dx').setValue(width)
            self.settings.child('calibration', 'calibration_ellipse', 'dy').setValue(height)
            self.settings.child('calibration', 'calibration_ellipse', 'theta').setValue(np.rad2deg(theta))
            if not self.running:
                break

            QtWidgets.QApplication.processEvents()
        self.pid_controller.setpoint_sb.setValue(self.pid_controller.setpoint_sb.value())
        self.settings.child('calibration', 'do_calibration').setValue(False)
        for mod in self.pid_controller.actuator_modules:
            mod.move_done_signal.disconnect(self.move_done)
        for mod in self.pid_controller.detector_modules:
            mod.grab_done_signal.disconnect(self.det_done)

    def timeout(self):
        """
            Send the status signal *'Time out during acquisition'* and stop the timer.
        """
        self.timeout_scan_flag=True
        self.timer.stop()
        self.pid_controller.update_status("Timeout during acquisition",log_type='log')



    def wait_for_det_done(self):
        self.timeout_scan_flag=False
        self.timer.start(self.settings.child('calibration','timeout').value())
        while not(self.det_done_flag or  self.timeout_scan_flag):
            #wait for grab done signals to end
            QtWidgets.QApplication.processEvents()
        self.timer.stop()

    pyqtSlot(OrderedDict) #edict(name=self.title,data0D=None,data1D=None,data2D=None)
    def det_done(self,data):
        """
            | Initialize 0D/1D/2D datas from given data parameter.
            | Update h5_file group and array.
            | Save 0D/1D/2D datas.

            =============== ============================== ======================================
            **Parameters**    **Type**                      **Description**
            *data*          Double precision float array   The initializing data of the detector
            =============== ============================== ======================================
        """
        #print(data['data0D']['OpenCV_Integrated_ROI_00'])
        if not (self.settings.child('calibration', 'do_calibration').value() or self.pid_controller.run_action.isChecked()):
            det_done_datas = OrderedDict()
            det_done_datas[data['name']] = data
            self.convert_input(det_done_datas)

        self.det_done_flag = True

    def wait_for_move_done(self):
        self.timeout_scan_flag=False
        self.timer.start(self.settings.child('calibration','timeout').value())
        while not(self.move_done_flag or  self.timeout_scan_flag):
            #wait for move done signals to end
            QtWidgets.QApplication.processEvents()
        self.timer.stop()

    pyqtSlot(str,float)
    def move_done(self,name,position):
        #print(position)
        self.curr_position = position
        self.move_done_flag=True


    def get_phi_from_xy(self,x,y):
        x0 = self.settings.child('calibration', 'calibration_ellipse', 'x0').value()
        y0 = self.settings.child('calibration', 'calibration_ellipse', 'y0').value()
        dx = self.settings.child('calibration', 'calibration_ellipse', 'dx').value()
        dy = self.settings.child('calibration', 'calibration_ellipse', 'dy').value()
        theta = np.deg2rad(self.settings.child('calibration', 'calibration_ellipse', 'theta').value())

        #apply rotation of -theta to get ellipse on main axes
        v = np.array([x, y])
        rot = rot = np.array([[np.cos(-theta), -np.sin(-theta)],[np.sin(-theta),np.cos(-theta)]])
        vprim = rot @ v

        phi = np.arctan2((vprim[1]+x0*np.sin(theta)-y0*np.cos(theta))/dy,
                         (vprim[0]-x0*np.cos(theta)-y0*np.sin(theta))/dx)
        self.settings.child('stabilization', 'interfero_phase').setValue(phi)
        self.curr_phase = phi
        return phi


    def interfero_from_phase(self, phi):
        laser_wl = self.settings.child('stabilization', 'laser_wl').value()
        phi_offset = self.settings.child('stabilization', 'offsets', 'phase_offset').value()
        interfero_offset = self.settings.child('stabilization', 'offsets', 'interfero_offset').value()
        phase_sign = self.settings.child('stabilization', 'phase_sign').value()

        self.phases = np.append(self.phases, [phi])
        self.phases = np.unwrap(self.phases[1:])

        interfero_position = interfero_offset+phase_sign*laser_wl/(2*np.pi*2)*(self.phases[-1]-phi_offset)
        return interfero_position

    def convert_input(self, measurements):
        """
        Convert the measurements in the units to be fed to the PID (same dimensionality as the setpoint)
        Parameters
        ----------
        measurements: (list) List of object from which the model extract a value of the same units as the setpoint

        Returns
        -------
        float: the converted input

        """
        #print('input conversion done')

        x = measurements[self.data_names[0][0]][self.data_names[0][1]][self.data_names[0][2]]
        y = measurements[self.data_names[1][0]][self.data_names[1][1]][self.data_names[1][2]]
        phi = self.get_phi_from_xy(x, y)

        pos = self.interfero_from_phase(phi) #in nm
        self.settings.child('stabilization', 'interfero_position').setValue(pos)
        self.curr_input = pos/1000
        return pos/1000 # in microns for the PI stage

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
        #no conversion needed
        self.curr_output = output
        if stab:
            offset = 0#self.settings.child('stabilization', 'offsets', 'piezo_offset').value()
        else:
            offset = 0
        return [output+offset]


if __name__ == '__main__':
    main('Michelson.xml')
