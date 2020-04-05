import smbus
import time
import struct
import logging
import datetime

logger = logging.getLogger(__name__)

BQ27441_I2C_ADDRESS = 0x55  # Default I2C address of the BQ27441-G1A
BQ27441_UNSEAL_KEY = 0x8000  # Secret code to unseal the BQ27441-G1A
BQ27441_DEVICE_ID = 0x0421  # Default device ID

#
# Standard Command 标准命令
#
# The fuel gauge uses a series of 2-byte standard commands to enable system
# reading and writing of battery information. Each command has an associated
# sequential command-code pair.
BQ27441_COMMAND_CONTROL = 0x00
BQ27441_COMMAND_TEMP = 0x02
BQ27441_COMMAND_VOLTAGE = 0x04
BQ27441_COMMAND_FLAGS = 0x06
BQ27441_COMMAND_NOM_CAPACITY = 0x08
BQ27441_COMMAND_AVAIL_CAPACITY = 0x0A
BQ27441_COMMAND_REM_CAPACITY = 0x0C
BQ27441_COMMAND_FULL_CAPACITY = 0x0E
BQ27441_COMMAND_AVG_CURRENT = 0x10
BQ27441_COMMAND_STDBY_CURRENT = 0x12
BQ27441_COMMAND_MAX_CURRENT = 0x14
BQ27441_COMMAND_AVG_POWER = 0x18
BQ27441_COMMAND_SOC = 0x1C
BQ27441_COMMAND_INT_TEMP = 0x1E
BQ27441_COMMAND_SOH = 0x20
BQ27441_COMMAND_REM_CAP_UNFL = 0x28
BQ27441_COMMAND_REM_CAP_FIL = 0x2A
BQ27441_COMMAND_FULL_CAP_UNFL = 0x2C
BQ27441_COMMAND_FULL_CAP_FIL = 0x2E
BQ27441_COMMAND_SOC_UNFL = 0x30
BQ27441_COMMAND_TRUEREM_CAPACITY = 0x6A
#
# Control Sub-commands 控制子命令
#
# Issuing a Control(self) command requires a subsequent 2-byte subcommand. These
# additional bytes specify the particular control function desired. The
# Control(self) command allows the system to control specific features of the fuel
# gauge during normal operation and additional features when the device is in
# different access modes.

BQ27441_CONTROL_STATUS = 0x00
BQ27441_CONTROL_DEVICE_TYPE = 0x01
BQ27441_CONTROL_FW_VERSION = 0x02
BQ27441_CONTROL_DM_CODE = 0x04
BQ27441_CONTROL_PREV_MACWRITE = 0x07
BQ27441_CONTROL_CHEM_ID = 0x08
BQ27441_CONTROL_BAT_INSERT = 0x0C
BQ27441_CONTROL_BAT_REMOVE = 0x0D
BQ27441_CONTROL_SET_HIBERNATE = 0x11
BQ27441_CONTROL_CLEAR_HIBERNATE = 0x12
BQ27441_CONTROL_SET_CFGUPDATE = 0x13
BQ27441_CONTROL_SHUTDOWN_ENABLE = 0x1B
BQ27441_CONTROL_SHUTDOWN = 0x1C
BQ27441_CONTROL_SEALED = 0x20
BQ27441_CONTROL_PULSE_SOC_INT = 0x23
BQ27441_CONTROL_RESET = 0x41
BQ27441_CONTROL_SOFT_RESET = 0x42
BQ27441_CONTROL_EXIT_CFGUPDATE = 0x43
BQ27441_CONTROL_EXIT_RESIM = 0x44

#
# Control Status Word - Bit Definitions 控制状态词
#
# Bit positions for the 16-bit data of CONTROL_STATUS.
# CONTROL_STATUS instructs the fuel gauge to return status information to
# Control(self) addresses 0x00 and 0x01. The read-only status word contains status
# bits that are set or cleared either automatically as conditions warrant or
# through using specified subcommands.

BQ27441_STATUS_SHUTDOWNEN = 1 << 15
BQ27441_STATUS_WDRESET = 1 << 14
BQ27441_STATUS_SS = 1 << 13
BQ27441_STATUS_CALMODE = 1 << 12
BQ27441_STATUS_CCA = 1 << 11
BQ27441_STATUS_BCA = 1 << 10
BQ27441_STATUS_QMAX_UP = 1 << 9
BQ27441_STATUS_RES_UP = 1 << 8
BQ27441_STATUS_INITCOMP = 1 << 7
BQ27441_STATUS_HIBERNATE = 1 << 6
BQ27441_STATUS_SLEEP = 1 << 4
BQ27441_STATUS_LDMD = 1 << 3
BQ27441_STATUS_RUP_DIS = 1 << 2
BQ27441_STATUS_VOK = 1 << 1

#
# Flag Command - Bit Definitions 标志命令
#
# Bit positions for the 16-bit data of Flags(self)
# This read-word function returns the contents of the fuel gauging status
# register, depicting the current operating status.
BQ27441_FLAG_OT = 1 << 15
BQ27441_FLAG_UT = 1 << 14
BQ27441_FLAG_FC = 1 << 9
BQ27441_FLAG_CHG = 1 << 8
BQ27441_FLAG_OCVTAKEN = 1 << 7
BQ27441_FLAG_ITPOR = 1 << 5
BQ27441_FLAG_CFGUPMODE = 1 << 4
BQ27441_FLAG_BAT_DET = 1 << 3
BQ27441_FLAG_SOC1 = 1 << 2
BQ27441_FLAG_SOCF = 1 << 1
BQ27441_FLAG_DSG = 1 << 0

dict_flag = {'OT': BQ27441_FLAG_OT, 'UT': BQ27441_FLAG_UT, 'FC': BQ27441_FLAG_FC, \
             'CHG': BQ27441_FLAG_CHG, 'OCVTAKEN': BQ27441_FLAG_OCVTAKEN, 'ITPOR': BQ27441_FLAG_ITPOR, \
             'CFGUPMODE': BQ27441_FLAG_CFGUPMODE, 'BAT_DET': BQ27441_FLAG_BAT_DET, \
             'SOC1': BQ27441_FLAG_SOC1, 'SOCF': BQ27441_FLAG_SOCF, 'DSG': BQ27441_FLAG_DSG}

#
# Extended Data Commands 控制数据命令
#
# Extended data commands offer additional functionality beyond the standard
# set of commands. They are used in the same manner; however, unlike standard
# commands, extended commands are not limited to 2-byte words.
BQ27441_EXTENDED_OPCONFIG = 0x3A  # OpConfig(self)
BQ27441_EXTENDED_CAPACITY = 0x3C  # DesignCapacity(self)
BQ27441_EXTENDED_DATACLASS = 0x3E  # DataClass(self)
BQ27441_EXTENDED_DATABLOCK = 0x3F  # DataBlock(self)
BQ27441_EXTENDED_BLOCKDATA = 0x40  # BlockData(self)
BQ27441_EXTENDED_CHECKSUM = 0x60  # BlockDataCheckSum(self)
BQ27441_EXTENDED_CONTROL = 0x61  # BlockDataControl(self)

#
# Configuration Class, Subclass ID's # 配置类，子类的ID
#
# To access a subclass of the extended data, set the DataClass(self) function
# with one of these values.
# Configuration Classes
BQ27441_ID_SAFETY = 2  # Safety
BQ27441_ID_CHG_TERMINATION = 36  # Charge Termination
BQ27441_ID_CONFIG_DATA = 48  # Data
BQ27441_ID_DISCHARGE = 49  # Discharge
BQ27441_ID_REGISTERS = 64  # Registers
BQ27441_ID_POWER = 68  # Power
# Gas Gauging Classes
BQ27441_ID_IT_CFG = 80  # IT Cfg
BQ27441_ID_CURRENT_THRESH = 81  # Current Thresholds
BQ27441_ID_STATE = 82  # State
# Ra Tables Classes
BQ27441_ID_R_A_RAM = 89  # R_a RAM
# Calibration Classes
BQ27441_ID_CALIB_DATA = 104  # Data
BQ27441_ID_CC_CAL = 105  # CC Cal
BQ27441_ID_CURRENT = 107  # Current
# Security Classes
BQ27441_ID_CODES = 112  # Codes

#
# OpConfig Register - Bit Definitions 选项寄存器
#
# Bit positions of the OpConfig Register
BQ27441_OPCONFIG_BIE = 1 << 13
BQ27441_OPCONFIG_BI_PU_EN = 1 << 12
BQ27441_OPCONFIG_GPIOPOL = 1 << 11
BQ27441_OPCONFIG_SLEEP = 1 << 5
BQ27441_OPCONFIG_RMFCC = 1 << 4
BQ27441_OPCONFIG_BATLOWEN = 1 << 2
BQ27441_OPCONFIG_TEMPS = 1 << 0
class UPS2Control():
    # this is the sample python 3 code that how to view the batery information
    # BQ27441 Python Library by ugeek (sp@geekworm.com)

    #
    # General Constants 常量
    #

    def __init__(self, battery_cap=2500):
    # Change 2500 to your battery capacity (mAh)
        self.MY_BATTERY_CAP = battery_cap
        self.bus=smbus.SMBus(1)
        time.sleep(1)
        self.writeCap(MY_BATTERY_CAP)

    def readControlWord(self, cmd):
        self.bus.write_word_data(BQ27441_I2C_ADDRESS, 0x00, cmd)
        # up is below
        # bus.write_byte_data(BQ27441_I2C_ADDRESS,0x00,cmd & 0x00FF)
        # bus.write_byte_data(BQ27441_I2C_ADDRESS,0x01,cmd >> 8)
        return self.bus.read_word_data(BQ27441_I2C_ADDRESS, 0x00)

    def executeControlWord(self, cmd):
        self.bus.write_word_data(BQ27441_I2C_ADDRESS, 0x00, cmd)

    # # subCommandMSB = (cmd >> 8);
    # # subCommandLSB = (cmd & 0x00FF)
    # # command = [subCommandLSB, subCommandMSB]
    # #print "Control Word [0x%04x]" % ((subCommandLSB << 8) | subCommandMSB)
    # new_cmd = (cmd >> 8) | ((cmd & 0x00FF) << 8)
    # print "Execute Control Word [0x%04x]" % cmd
    # bus.write_word_data(BQ27441_I2C_ADDRESS,0x00,0x0001)
    # bus.write_word_data(BQ27441_I2C_ADDRESS,0x00,new_cmd)
    # #bus.write_word_data(BQ27441_I2C_ADDRESS,0x00,(subCommandLSB << 8) | subCommandMSB)

    # #bus.write_byte_data(BQ27441_I2C_ADDRESS,0x00,subCommandLSB)
    # #bus.write_byte_data(BQ27441_I2C_ADDRESS,0x01,subCommandMSB)

    def writeExtendedCommand(self, addr, val):
        # print "Write Extend Command addr[0x%02x]" % addr ,", val[0x%02x]" % val
        self.bus.write_byte_data(BQ27441_I2C_ADDRESS, addr, val)

    # bus.write_byte_data(BQ27441_I2C_ADDRESS,0,addr)
    # bus.write_byte_data(BQ27441_I2C_ADDRESS,1,val)
    # bus.write_byte(BQ27441_I2C_ADDRESS,val)

    # bus.write_byte_data(BQ27441_I2C_ADDRESS, 0, addr)
    # bus.write_byte_data(BQ27441_I2C_ADDRESS, 1, val)

    def device_id(self):
        logger.debug('get device_id')
        return self.readControlWord(BQ27441_CONTROL_DEVICE_TYPE)

    def dm_id(self):
        logger.debug('get dm_id')
        return self.readControlWord(BQ27441_CONTROL_DM_CODE)

    def fw_version(self):
        logger.debug('get fw_version')
        return self.readControlWord(BQ27441_CONTROL_FW_VERSION)

    def chem_id(self):
        logger.debug('get chem_id')
        return self.readControlWord(BQ27441_CONTROL_CHEM_ID)

    def controlStatus(self):
        logger.debug('get controlStatus')
        status = self.readControlWord(BQ27441_CONTROL_STATUS)
        return status

    def softReset(self):
        logger.debug('perform soft_reset')
        self.executeControlWord(BQ27441_CONTROL_SOFT_RESET)

    def setConfigUpdate(self):
        logger.debug("Set Config Update...")
        self.executeControlWord(BQ27441_CONTROL_SET_CFGUPDATE)

    # bus.write_byte_data(BQ27441_I2C_ADDRESS,0x00,0x13)
    # bus.write_byte_data(BQ27441_I2C_ADDRESS,0x01,0x00)
    def exitConfigUpdate(self):
        logger.debug("Exit Config Update...")
        self.executeControlWord(BQ27441_CONTROL_EXIT_CFGUPDATE)

    def seal(self):
        logger.debug("seal")
        self.executeControlWord(BQ27441_CONTROL_SEALED)

    # executeControlWord(BQ27441_CONTROL_SEALED)

    def unseal(self):
        logger.debug("unseal")
        self.executeControlWord(BQ27441_UNSEAL_KEY)
        self.executeControlWord(BQ27441_UNSEAL_KEY)

    # bus.write_byte_data(BQ27441_I2C_ADDRESS,0x00,0x00)
    # bus.write_byte_data(BQ27441_I2C_ADDRESS,0x01,0x80)
    # bus.write_byte_data(BQ27441_I2C_ADDRESS,0x00,0x00)
    # bus.write_byte_data(BQ27441_I2C_ADDRESS,0x01,0x80)

    def writeCap(self, cap):
        self.unseal()
        self.setConfigUpdate()

        self.writeExtendedCommand(BQ27441_EXTENDED_CONTROL, 0x00)
        self.writeExtendedCommand(BQ27441_EXTENDED_DATACLASS, 0x52)
        self.writeExtendedCommand(BQ27441_EXTENDED_DATABLOCK, 0x00)

        block = self.bus.read_i2c_block_data(BQ27441_I2C_ADDRESS, BQ27441_EXTENDED_BLOCKDATA, 32)
        block = list(struct.unpack('32B', bytearray(block)[0:32]))
        block[0x0a] = cap >> 8
        block[0x0b] = cap & 0xFF
        new_checksum = ~sum(block) & 0xFF

        self.bus.write_byte_data(BQ27441_I2C_ADDRESS, 0x4a, cap >> 8)  # writing new capacity
        self.bus.write_byte_data(BQ27441_I2C_ADDRESS, 0x4b, cap & 0xFF)  # writing new capacity

        time.sleep(1)
        # write checksum
        self.bus.write_byte_data(BQ27441_I2C_ADDRESS, 0x60, new_checksum)  # trying to write on BlockDataChecksum(self)

        time.sleep(1)

        softReset()
        exitConfigUpdate()

        seal()

    def availCap(self):
        val = self.bus.read_i2c_block_data(BQ27441_I2C_ADDRESS, BQ27441_COMMAND_AVAIL_CAPACITY, 2)
        new_val = struct.unpack('H', bytearray(val)[0:2])
        return new_val[0]

    def desCap(self):
        val = self.bus.read_i2c_block_data(BQ27441_I2C_ADDRESS, BQ27441_EXTENDED_CAPACITY, 2)
        new_val = struct.unpack('H', bytearray(val)[0:2])
        return new_val[0]

    def voltage(self):
        val = self.bus.read_i2c_block_data(BQ27441_I2C_ADDRESS, BQ27441_COMMAND_VOLTAGE, 2)
        new_val = struct.unpack('H', bytearray(val)[0:2])
        return new_val[0]

    def soc(self):
        val = self.bus.read_i2c_block_data(BQ27441_I2C_ADDRESS, BQ27441_COMMAND_SOC, 2)
        new_val = struct.unpack('H', bytearray(val)[0:2])
        return new_val[0]

    def soh(self):
        val = self.get_status_u(BQ27441_COMMAND_SOH)
        return val & 0x00FF

    def get_status_u(self,reg):
        status = self.bus.read_i2c_block_data(BQ27441_I2C_ADDRESS, reg, 2)
        (new_status,) = struct.unpack('H', bytearray(status)[0:2])
        return new_status

    def get_status(self, reg):
        status = self.bus.read_i2c_block_data(BQ27441_I2C_ADDRESS, reg, 2)
        (new_status,) = struct.unpack('h', bytearray(status)[0:2])
        return new_status

    # def unseal():
    # executeControlWord(BQ27441_UNSEAL_KEY)
    # executeControlWord(BQ27441_UNSEAL_KEY)

    def get_all_info(self):
        info = {}
        info["status_control"] = self.get_status_u(BQ27441_COMMAND_CONTROL)
        info["status_temp"] = self.get_status_u(BQ27441_COMMAND_TEMP)
        info["status_voltage"] = self.get_status_u(BQ27441_COMMAND_VOLTAGE)
        info["status_nom_capacity"] = self.get_status_u(BQ27441_COMMAND_NOM_CAPACITY)
        info["status_avail_capacity"] = self.get_status_u(BQ27441_COMMAND_AVAIL_CAPACITY)
        info["status_rem_capacity"] = self.get_status_u(BQ27441_COMMAND_REM_CAPACITY)
        info["status_full_capacity"] = self.get_status_u(BQ27441_COMMAND_FULL_CAPACITY)
        info["status_avg_current"] = self.get_status(BQ27441_COMMAND_AVG_CURRENT)
        info["status_stdby_current"] = self.get_status(BQ27441_COMMAND_STDBY_CURRENT)
        info["status_max_current"] = self.get_status(BQ27441_COMMAND_MAX_CURRENT)
        info["status_avg_power"] = self.get_status(BQ27441_COMMAND_AVG_POWER)
        info["status_soc"] = self.get_status_u(BQ27441_COMMAND_SOC)
        info["status_int_temp"] = self.get_status_u(BQ27441_COMMAND_INT_TEMP)
        info["status_soh"] = self.soh()
        info["status_rem_cap_unfil"] = self.get_status_u(BQ27441_COMMAND_REM_CAP_UNFL)
        info["status_rem_cap_fil"] = self.get_status_u(BQ27441_COMMAND_REM_CAP_FIL)
        info["status_full_cap_unfil"] = self.get_status_u(BQ27441_COMMAND_FULL_CAP_UNFL)
        info["status_full_cap_fil"] = self.get_status_u(BQ27441_COMMAND_FULL_CAP_FIL)
        info["status_soc_unfl"] = self.get_status_u(BQ27441_COMMAND_SOC_UNFL)
        info["status_truerem_capacity"] = self.get_status_u(BQ27441_COMMAND_TRUEREM_CAPACITY)
        return info

    def get_basic_info(self):
        basic_info = {}
        basic_info["battery_soc"] = self.get_status_u(BQ27441_COMMAND_SOC)
        basic_info["battery_voltage"] = self.get_status_u(BQ27441_COMMAND_VOLTAGE)
        basic_info["battery_current"] = self.get_status(BQ27441_COMMAND_AVG_CURRENT)
        basic_info["battery_soh"] = self.soh()
        return basic_info

    def print_basic_info(self):
        # print "[Basic Info]"
        # print "1.Voltage:", float(battery_voltage) / 1000, "V"
        # print "2.SOC:    ", battery_soc , "%"
        # print "3.Current:", float(battery_current) / 1000 , "A"
        # print "4.SOH:    ", battery_soh, "%"
        basic_info = self.get_basic_info()
        battery_voltage = basic_info['battery_voltage']
        battery_current = basic_info['battery_current']
        battery_soc = basic_info['battery_soc']
        print("Voltage:{voltage:.3f}V - Current:{current:.3f}A - SOC:{soc:.2f}%" .format(
            voltage=float(battery_voltage) / 1000,
            current=float(battery_current) / 1000,
            soc=battery_soc))


if __name__ == "__main__":
    ups = UPS2Control(8000)
    ups.print_basic_info()
    all_infos = ups.get_all_info()
    for k,v in  all_infos.items():
        print(k, v)

