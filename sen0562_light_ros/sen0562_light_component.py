# MIT License

# Copyright (c) 2024 Takumi Asada

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# https://wiki.dfrobot.com/SKU_SEN0562_Gravity_I2C_Waterproof_Ambient_Light_Sensor_1_65535lx

#--------------------------------------------------------------------------------#
# Include #
#--------------------------------------------------------------------------------#
import smbus
import time

from rclpy.node import Node
from std_msgs.msg import Float32

#--------------------------------------------------------------------------------#
# Class #
#--------------------------------------------------------------------------------#
class LightSensorComponent(Node):
    def __init__(self):
        super().__init__('sen0562_light_node')
        self.pub_illuminance = self.create_publisher(Float32, "/sen0562/illuminance", 10)

        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.address = 0x23  # I2C address
        self.register = 0x10  # register address
        self.size = 2  # datasize
        self.bus_number = 1  # bus number: default 1
        self.bus = smbus.SMBus(self.bus_number) # Init smbus object

    def __del__(self):
        self.bus.close()

    #----------------------------------------------------------------------------#
    # timer_callback #
    #----------------------------------------------------------------------------#
    def timer_callback(self):
        msg_illuminance = Float32()

        data = self.read_reg(self.register)
        if data is not None:
            msg_illuminance.data = self.convert_data(data)
        time.sleep(0.25)
        self.get_logger().info("LUX: {}lx".format(msg_illuminance.data))
        self.pub_illuminance.publish(msg_illuminance)


    def read_reg(self, reg):
        try:
            # レジスタアドレスを指定してデータを読み取る
            data = self.bus.read_i2c_block_data(self.address, reg, self.size)
            return data
        except Exception as e:
            print(f"Error reading register: {e}")
            return None

    def convert_data(self, data):
        # データを16ビットに結合
        result = (data[0] << 8) | data[1]
        # ルクス値に変換
        lux = float(result) / 1.2
        return lux
