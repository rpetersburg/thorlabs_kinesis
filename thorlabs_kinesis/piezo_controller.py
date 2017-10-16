"""
piezo_controller.py was written by Ryan Petersburg for use with Thorlabs KPZ101
piezo controller and the NanoMax 300 piezo stage. This module uses ideas from
the Instrumental and thorlabs_apt python packages.
"""

from __future__ import division
from time import sleep, time
from ctypes import *
from ctypes.wintypes import WORD, DWORD
from warnings import warn
import os
import sys

PIEZO_DLL = 'Thorlabs.MotionControl.KCube.Piezo.dll'
DEVICE_MANAGER_DLL = 'Thorlabs.MotionControl.DeviceManager.dll'
MODULE_FOLDER = os.path.dirname(os.path.realpath(__file__))

class KinesisError(Exception):
    messages = {
        0: 'Success',
        1: 'The FTDI functions have not been initialized',
        2: 'The device could not be found. Make sure to call TLI_BuildDeviceList().',
        3: 'The device must be opened before it can be accessed',
        4: 'An I/O Error has occured in the FTDI chip',
        5: 'There are insufficient resources to run this application',
        6: 'An invalid parameter has been supplied to the device',
        7: 'The device is no longer present',
        8: 'The device detected does not match that expected',
        32: 'The device is already open',
        33: 'The device has stopped responding',
        34: 'This function has not been implemented',
        35: 'The device has reported a fault',
        36: 'The function could not be completed because the device is disconnected',
        41: 'The firmware has thrown an error',
        42: 'The device has failed to initialize',
        43: 'An invalid channel address was supplied',
        37: 'The device cannot perform this function until it has been Homed',
        38: 'The function cannot be performed as it would result in an illegal position',
        39: 'An invalid velocity parameter was supplied. The velocity must be greater than zero',
        44: 'This device does not support Homing. Check the Limit switch parameters are correct',
        45: 'An invalid jog mode was supplied for the jog function',
    }

    def __init__(self, action=None, code=-1, msg=''):
        err_msg = ''
        if action is not None:
            err_msg += 'Method {} failed'.format(action)
        if msg:
            err_msg += ': {}'.format(msg)
        if code > -1:
            err_msg += '\nKinesis Error: [Error {}] {}'.format(code, self.messages[code])
        super(KinesisError, self).__init__(err_msg)
        self.code = code

def list_devices():
    """
    Lists all devices connected to the computer.

    Returns
    -------
    device_list : list
        List of available devices.
    """
    lib = load_dll(DEVICE_MANAGER_DLL)
    lib.TLI_BuildDeviceList()
    devices = c_buffer(255)
    code = lib.TLI_GetDeviceListExt(devices, len(devices))
    if code:
        raise KinesisError(code, 'TLI_GetDeviceListExt')
    device_list = devices.value.decode().strip(',').split(',')
    while '' in device_list:
        device_list.remove('')
    return device_list

def load_dll(dll, deps=[], folder=''):
    """
    Returns a dynamically linked library (lib) as a Python object

    Args
    ----
    dll : str
        Name of the dll including the extension. Must be contained in the same
        folder as this module or in 'C:\\Program Files\\Thorlabs\\Kinesis'
    deps : list(str), optional
        Relative paths of any dependencies of the return dll library that need
        to be initialized before the primary library.
    """
    if not folder:
        folders = [MODULE_FOLDER,
                   'C:\\Program Files\\Thorlabs\\Kinesis',
                   'C:\\Program Files (x86)\\Thorlabs\\Kinesis']
        for f in folders:
            try: dirs = os.listdir(f)
            except FileNotFoundError: continue
            if dll in dirs:
                folder = f
                break
        else:
            folder = input('Thorlabs Kinesis dll folder:\n')
    for dep in deps:
        load_dll(dep, folder=folder)
    return windll.LoadLibrary(folder + os.sep + dll)

def _if_open(func):
    """
    Decorator for PiezoController methods to test if the device is open before
    executing the command. Prevents errors when interfacing with the dll.
    """
    def wrapper(self, *args, **kwargs):
        if self.is_open:
            return func(self, *args, **kwargs)
        else:
            warn('Cannot execute function {}: Piezo is not connected.'.format(func.__name__))
    return wrapper

class PiezoController(object):
    """
    Object used to control a Thorlabs motor.

    Parameters
    ----------
    serial : int
        Serial number identifying device
    """
    _dll = PIEZO_DLL
    _dll_deps = [DEVICE_MANAGER_DLL]
    _lib = load_dll(_dll, _dll_deps)
    _max_voltage_bit = 32767
    _message_type = 1

    def __init__(self, serial_number=''): 
        self._is_open = False
        self.voltage_step = 1.0       
        self.serial_number = serial_number

    def __del__(self):
        self.close()

    def __repr__(self):
        if self.is_open:
            return '<PiezoController object for {} sn:{}>'.format(
                    self.hardware_info['model'], self.serial_number)
        else:
            return '<Uninitialized PiezoController object>'

    @property
    @_if_open
    def hardware_info(self):
        model = c_buffer(255)
        hw_type = c_long()
        num_channels = c_long()
        hw_notes = c_buffer(255)
        firmware_ver = c_long()
        hw_ver = c_long()
        mod_state = c_long()
        code = self._lib.PCC_GetHardwareInfo(self._serial,
                byref(model), len(model),
                byref(hw_type), byref(num_channels),
                hw_notes, len(hw_notes),
                byref(firmware_ver), byref(hw_ver), byref(mod_state))
        if code:
            raise KinesisError('PCC_GetHardwareInfo', code)
        return {
            'serial number': self.serial_number,
            'model': model.value.decode(),
            'hardware type': hw_type.value,
            'number of channels': num_channels.value,
            'hardware notes': hw_notes.value.decode(),
            'firmware version': firmware_ver.value,
            'hardware version': hw_ver.value,
            'modification state': mod_state.value
        }

    def get_serial_number(self):
        if self._serial:
            return int(self._serial.value)
        return self._serial

    def set_serial_number(self, serial):
        if serial:
            self._serial = c_char_p(str(serial).encode())
            try:
                self.open()
                self.enable()
            except KinesisError:
                warn('Invalid Serial Number. Piezo not set.')
                self._serial = ''
        else:
            self._serial = serial

    serial_number = property(get_serial_number, set_serial_number)

    def open(self):
        code = self._lib.PCC_Open(self._serial)
        if code:
            raise KinesisError('PCC_Open', code)
        self._is_open = True

    def close(self):
        if self._is_open:
            # if self.is_enabled:
            #     self.disable()
            self._lib.PCC_Close(self._serial)
            self._is_open = False

    @property
    def is_open(self):
        return self._is_open

    @_if_open
    def enable(self):
        if not self.is_enabled:
            code = self._lib.PCC_Enable(self._serial)
            if code:
                raise KinesisError('PCC_Enable', code)
            self.start_polling()
            self.wait_for_message(1,2)

    @_if_open
    def disable(self):
        if self.is_enabled:
            if self.is_polling:
                self.stop_polling()
            code = self._lib.PCC_Disable(self._serial)
            if code:
                raise KinesisError('PCC_Disable', code)

    @_if_open
    def get_max_voltage(self):
        success = self._lib.PCC_RequestMaxOutputVoltage(self._serial)
        if not success:
            raise KinesisError('PCC_RequestMaxOutputVoltage')
        return self._lib.PCC_GetMaxOutputVoltage(self._serial) / 10.0

    @_if_open
    def set_max_voltage(self, max_voltage):
        if max_voltage not in [75.0, 100.0, 150.0]:
            raise Exception('Max Voltage must be 75V, 100V, or 150V')
        c_max_voltage = c_short(int(max_voltage*10))
        code = self._lib.PCC_SetMaxOutputVoltage(self._serial, max_voltage*10)
        if code:
            raise KinesisError('PCC_SetMaxOutputVoltage', code)

    max_voltage = property(get_max_voltage, set_max_voltage)

    @_if_open
    def get_voltage(self):
        if not self.is_enabled:
            return None
        return self.voltage_from_bit(self.voltage_bit)

    @_if_open
    def set_voltage(self, voltage):
        if self.is_enabled:
            self.voltage_bit = self.bit_from_voltage(voltage)

    voltage = property(get_voltage, set_voltage)

    def bit_from_voltage(self, voltage):
        return int(round(voltage / self.max_voltage * self._max_voltage_bit))

    def voltage_from_bit(self, voltage_bit):
        return voltage_bit / self._max_voltage_bit * self.max_voltage

    @_if_open
    def get_voltage_bit(self):
        if not self.is_polling:
            self._request_voltage()
            sleep(0.5)
            self._request_voltage() # For some reason, it needs to be twice
        return self._lib.PCC_GetOutputVoltage(self._serial)

    @_if_open
    def _request_voltage(self):
        code = self._lib.PCC_RequestOutputVoltage(self._serial)
        if code:
            raise KinesisError('PCC_RequestOutputVoltage', code)

    @_if_open
    def set_voltage_bit(self, voltage_bit):
        if voltage_bit > self._max_voltage_bit:
            voltage_bit = self._max_voltage_bit
        elif voltage_bit < 0:
            voltage_bit = 0
        self._set_voltage_bit(voltage_bit)
        # Attempt to get closer to actual value. Takes much too long to execute
        # i = 0
        # new_voltage_bit = voltage_bit
        # while abs(self.voltage_bit - voltage_bit) > 5 and i < 100:
        #     new_voltage_bit  = round(new_voltage_bit * voltage_bit / self.voltage_bit)
        #     self._set_voltage_bit(new_voltage_bit)
        #     i += 1

    @_if_open
    def _set_voltage_bit(self, voltage_bit):
        code = self._lib.PCC_SetOutputVoltage(self._serial, c_short(voltage_bit))
        if code:
            raise KinesisError('PCC_SetOutputVoltage', code)
        # Best way I could find to wait for piezo voltage to settle
        while abs(self.voltage_bit - voltage_bit) > 100:
            sleep(.1)
        sleep(.1)

    voltage_bit = property(get_voltage_bit, set_voltage_bit)

    def get_voltage_step(self):
        return self._voltage_step

    def set_voltage_step(self, voltage_step):
        self._voltage_step = voltage_step

    voltage_step = property(get_voltage_step, set_voltage_step)

    @_if_open
    def increase_voltage(self, step=None):
        if step is None:
            step = self.voltage_step
        self.voltage += step

    @_if_open
    def decrease_voltage(self, step=None):
        if step is None:
            step = self.voltage_step
        self.voltage -= step

    @_if_open
    def start_polling(self, interval_ms=100):
        interval_ms = c_int(interval_ms)
        success = self._lib.PCC_StartPolling(self._serial, interval_ms)
        if not success:
            raise KinesisError('PCC_StartPolling')

    @_if_open
    def stop_polling(self):
        self._lib.PCC_StopPolling(self._serial)

    @property
    @_if_open
    def polling_duration(self):
        return self._lib.PCC_PollingDuration(self._serial) / 1000.0

    @property
    @_if_open
    def _status_bits(self):
        self._request_status_bits()
        return self._lib.PCC_GetStatusBits(self._serial)

    @_if_open
    def _request_status_bits(self):
        code = self._lib.PCC_RequestStatusBits(self._serial)
        if code:
            raise KinesisError('PCC_RequestStatusBits', code)

    @_if_open
    def get_LED_brightness(self):
        return self._lib.PCC_GetLEDBrightness(self._serial)

    @_if_open
    def set_LED_brightness(self, brightness):
        brightness = c_short(int(brightness))
        code = self._lib.PCC_SetLEDBrightness(self._serial, brightness)
        if code:
            raise KinesisError('PCC_SetLEDBrightness', code)

    LED_brightness = property(get_LED_brightness, set_LED_brightness)

    @property
    @_if_open
    def is_active(self):
        return bool(self._status_bits & 0x20000000)

    @property
    @_if_open
    def is_enabled(self):
        return bool(self._status_bits & 0x80000000)

    @property
    @_if_open
    def is_zeroing(self):
        return bool(self._status_bits & 0x00000020)

    @property
    @_if_open
    def is_zeroed(self):
        return bool(self._status_bits & 0x00000010)

    @property
    @_if_open
    def is_connected(self):
        return bool(self._status_bits & 0x00000001)

    @property
    @_if_open
    def is_polling(self):
        return bool(self.polling_duration > 0)

    @_if_open
    def wait_for_message(self, message_type=1, message_id=2):
        c_message_type = WORD()
        c_message_id = WORD()
        c_message_data = DWORD()
        while (c_message_type.value != message_type and c_message_id.value != message_id):
            success = self._lib.PCC_WaitForMessage(self._serial,
                    byref(c_message_type), byref(c_message_id),
                    byref(c_message_data))
            if not success:
                raise KinesisError('PCC_WaitForMessage')

    @property
    @_if_open
    def message_queue_size(self):
        return self._lib.PCC_MessageQueueSize(self._serial)

    @_if_open
    def get_next_message(self):
        message_type = WORD()
        message_id = WORD()
        message_data = DWORD()
        success = self._lib.PCC_GetNextMessage(self._serial,
                byref(message_type), byref(message_id),
                byref(message_data))
        if not success:
            raise KinesisError('PCC_GetNextMessage')
        return message_type.value, message_id.value, message_data.value

    @property
    @_if_open
    def messages(self):
        messages = []
        for i in range(self.message_queue_size):
            messages.append(self.get_next_message)
        return messages
