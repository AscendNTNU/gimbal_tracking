import serial
import struct
import time
from collections import namedtuple


### - Verify correct USB tty port - ###
USB_PORT = '/dev/ttyUSB0'


ControlData = namedtuple(
    'ControlData',
    'roll_mode pitch_mode yaw_mode roll_speed roll_angle pitch_speed pitch_angle yaw_speed yaw_angle')

Message = namedtuple(
    'Message',
    'start_character command_id payload_size header_checksum payload payload_checksum')

Empty = namedtuple(
    'Empty',
    'empty1 empty2')

ImuData = namedtuple(
    'ImuData',
    'roll roll_target roll_target_speed pitch pitch_target pitch_speed yaw yaw_target yaw_speed'
)


def pack_control_data(control_data):
    return struct.pack('<BBBhhhhhh', *control_data)


def create_message(command_id, payload=None): #-> Message
    payload_size = len(payload)
    return Message(start_character=ord('>'),
                   command_id=command_id,
                   payload_size=payload_size,
                   header_checksum=(command_id + payload_size) % 256,
                   payload=payload,
                   payload_checksum=sum(bytearray(payload)) % 256)


def pack_message(message): # -> bytes:
    message_format = '<BBBB{}sB'.format(message.payload_size)
    return bytes(struct.pack(message_format, *message))    


def unpack_message(data, payload_size): # -> Message:
    message_format = '<BBBB{}sB'.format(payload_size)
    return Message._make(struct.unpack(message_format, data))


def read_message(connection, payload_size):# -> Message:
    # 5 is the length of the header + payload checksum byte
    # 1 is the payload size
    response_data = connection.read(5 + payload_size)
    # print('received response', response_data)
    return unpack_message(response_data, payload_size)


def test_rotate_gimbal():
    CMD_CONTROL = 67
    control_data = ControlData(roll_mode=0, roll_speed=0, roll_angle=0,
                               pitch_mode=0, pitch_speed=40, pitch_angle=0,
                               yaw_mode=0, yaw_speed=60, yaw_angle=0)
    print('command to send:', control_data)
    packed_control_data = pack_control_data(control_data)
    print('packed command as payload:', packed_control_data)
    message = create_message(CMD_CONTROL, packed_control_data)
    print('created message:', message)
    packed_message = pack_message(message)
    print('packed message:', packed_message)

    connection = serial.Serial(USB_PORT, baudrate=115200, timeout=10)
    print('send packed message:', packed_message)
    connection.write(packed_message)
    message = read_message(connection, 1)
    print('received confirmation:', message)
    print('confirmed command with ID:', ord(message.payload))



def speed_control(yaw_speed,pitch_speed):
    CMD_CONTROL = 67
    control_data = ControlData(roll_mode=0, roll_speed=0, roll_angle=0,
                               pitch_mode=1, pitch_speed=pitch_speed, pitch_angle=0,
                               yaw_mode=1, yaw_speed=yaw_speed, yaw_angle=0)
    packed_control_data = pack_control_data(control_data)
    message = create_message(CMD_CONTROL, packed_control_data)
    packed_message = pack_message(message)
    connection = serial.Serial(USB_PORT, baudrate=115200, timeout=10)
    connection.write(packed_message)
    message = read_message(connection, 1)


def position_control(yaw,pitch):
    '''Input in degrees
    - zero pitch is horizon level
    - zero yaw is forward in frame
    '''
    # Conversion from degree to IMU target
    y_imu = get_IMU_angles().yaw
    y_enc = get_encoder_angle_yaw()
    dy_imu = (yaw - y_enc)*45.5111  # difference in imu
    y_target = y_imu - dy_imu

    p_target = pitch*-45.5111 - 2048

    CMD_CONTROL = 67
    control_data = ControlData(roll_mode=0, roll_speed=0, roll_angle=0,
                               pitch_mode=2, pitch_speed=40, pitch_angle=p_target,
                               yaw_mode=2, yaw_speed=100, yaw_angle=y_target)
    # packing and sending
    packed_control_data = pack_control_data(control_data)
    message = create_message(CMD_CONTROL, packed_control_data)
    packed_message = pack_message(message)
    connection = serial.Serial(USB_PORT, baudrate=115200, timeout=10)
    connection.write(packed_message)
    # reading confirmaton
    message = read_message(connection, 1)


def follow_yaw_mode():
    CMD_CONTROL = 67
    control_data = ControlData(roll_mode=0, roll_speed=0, roll_angle=0,
                               pitch_mode=0, pitch_speed=0, pitch_angle=0,
                               yaw_mode=0, yaw_speed=0, yaw_angle=0)
    packed_control_data = pack_control_data(control_data)
    message = create_message(CMD_CONTROL, packed_control_data)
    packed_message = pack_message(message)
    connection = serial.Serial(USB_PORT, baudrate=115200, timeout=10)
    connection.write(packed_message)
    message = read_message(connection, 1)


def get_IMU_angles():
    CMD_GET_ANGLES = 73
    empty_data = Empty(empty1=0, empty2=0)
    packed_control_data = struct.pack('<hh', *empty_data)    
    message = create_message(CMD_GET_ANGLES, packed_control_data)
    packed_message = pack_message(message)
    connection = serial.Serial(USB_PORT, baudrate=115200, timeout=10)
    connection.write(packed_message)
    message = read_message(connection, 18)
    imu_angles = ImuData(*struct.unpack('<hhhhhhhhh',message.payload))
    return imu_angles

# Outputs yaw from encoders
def get_encoder_angle_yaw():
    CMD_DEBUG_VARS_3 = 254
    empty_data = Empty(empty1=0, empty2=0)
    packed_control_data = struct.pack('<hh', *empty_data)
    message = create_message(CMD_DEBUG_VARS_3, packed_control_data)
    packed_message = pack_message(message)
    connection = serial.Serial(USB_PORT, baudrate=115200, timeout=10)
    connection.write(packed_message)
    message = read_message(connection, 23)
    a = struct.unpack('<bbbbbbiiibbbbb',message.payload)
    yaw = a[8]//262144*(1.0/4096*360)-74 # Conversion from enc to deg
    return yaw


if __name__=="__main__":
    # position_control(0,0)

    while 1:
        y_imu = get_IMU_angles().yaw
        y_enc = get_encoder_angle_yaw()
        p_imu = get_IMU_angles().pitch

        print('IMU yaw= %s, ENC yaw= %s IMU pitch= %s' % (y_imu,y_enc,p_imu))