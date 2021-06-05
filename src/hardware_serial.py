from collections import namedtuple

import serial
import struct

import time

# outgoing CMD_CONTROL - control gimbal movement
ControlData = namedtuple(
    'ControlData',
    'roll_mode pitch_mode yaw_mode roll_speed roll_angle pitch_speed pitch_angle yaw_speed yaw_angle')

Message = namedtuple(
    'Message',
    'start_character command_id payload_size header_checksum payload payload_checksum')


def pack_control_data(control_data):
    return struct.pack('<BBBhhhhhh', *control_data)


def create_message(command_id, payload): #-> Message
    payload_size = len(payload)
    print('payload_size: %s' % payload_size)
    a = sum(bytearray(payload)) % 256
    print('payload_checksum: %s' % a)
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
    print('received response', response_data)
    return unpack_message(response_data, payload_size)


def rotate_gimbal(yaw_speed):
    CMD_CONTROL = 67
    control_data = ControlData(roll_mode=0, roll_speed=0, roll_angle=0,
                               pitch_mode=0, pitch_speed=40, pitch_angle=0,
                               yaw_mode=2, yaw_speed=40, yaw_angle=-9000)
    print('command to send:', control_data)
    packed_control_data = pack_control_data(control_data)
    print('packed command as payload:', packed_control_data)
    message = create_message(CMD_CONTROL, packed_control_data)
    print('created message:', message)
    packed_message = pack_message(message)
    print('packed message:', packed_message)

    connection = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=10)
    print('send packed message:', packed_message)
    connection.write(packed_message)
    message = read_message(connection, 1)
    print('received confirmation:', message)
    print('confirmed command with ID:', ord(message.payload))

def test(yaw_speed):
    while 1:
        rotate_gimbal(-200)
        time.sleep(5)
        rotate_gimbal(200)
        time.sleep(5)


if __name__ == '__main__':
    rotate_gimbal(0)