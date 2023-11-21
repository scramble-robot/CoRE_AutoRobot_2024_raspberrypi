import rclpy
from std_msgs.msg import Float64, Int64

from threading import Thread
import queue
import can
import struct
import time

def c620_set_current(bus, currents):
    d = [0]*8
    raw_array = [0]*4
    for i in range(4):
        v = int(round(currents[i]/20 * 16384))
        v = min(v, 16384)
        v = max(v, -16384)
        raw_array[i] = v
    b = struct.pack('>4h', *raw_array)
    msg = can.Message(arbitration_id=0x200, is_extended_id=False, data=b)
    try:
        bus.send(msg)
    except can.exceptions.CanOperationError:
        pass

def gm6020_set_volt(bus, volts):
    d = [0]*8
    raw_array = [0]*4
    for i in range(4):
        v = volts[i]
        v = min(v, 30000)
        v = max(v, -30000)
        raw_array[i] = v
    b = struct.pack('>4h', *raw_array)
    msg = can.Message(arbitration_id=0x1ff, is_extended_id=False, data=b)
    try:
        bus.send(msg)
    except can.exceptions.CanOperationError:
        pass

def parse_c620_data(msg: can.Message):
    data = struct.unpack('>hhhbb', msg.data)
    degree = data[0]/ 8192 * 360
    rpm = data[1]
    current = data[2]/16384*20
    temperature = data[3]
    return degree, rpm, current, temperature

def parse_gm6020_data(msg: can.Message):
    data = struct.unpack('>hhhbb', msg.data)
    degree = data[0]/ 8192 * 360
    rpm = data[1]
    current = data[2]/16384*20
    temperature = data[3]
    return degree, rpm, current, temperature

exit_flag = False
c620_msg_count = [0, 0]
gm6020_msg_count = [0, 0]

def can_task(c620_queues, c620_rpm_publishes, gm6020_queues, gm6020_deg_publishes):
    global exit_flag
    bus = can.interface.Bus(bustype="socketcan", channel="can0", bitrate=1000000)
    
    def notifier(msg: can.Message) -> None:
        if msg.arbitration_id == 0x201 or msg.arbitration_id == 0x202:
            idx = msg.arbitration_id - 0x201
            c620_msg_count[idx] += 1
            if c620_msg_count[idx] >= 5:
                c620_msg_count[idx] = 0
                degree, rpm, current, temperature = parse_c620_data(msg)
                topic = Float64()
                topic.data = float(rpm)
                c620_rpm_publishes[idx].publish(topic)
        elif msg.arbitration_id == 0x205 or msg.arbitration_id == 0x206:
            idx = msg.arbitration_id - 0x205
            gm6020_msg_count[idx] += 1
            if gm6020_msg_count[idx] >= 5:
                gm6020_msg_count[idx] = 0
                degree, rpm, current, temperature = parse_gm6020_data(msg)
                topic = Float64()
                topic.data = float(degree)
                gm6020_deg_publishes[idx].publish(topic)
            
    notifier = can.Notifier(bus, [notifier])
    
    c620_target_current = [0]*4
    gm6020_target_voltage = [0]*4
    
    while not exit_flag:
        # C620
        for i, que in enumerate(c620_queues):
            if not que.empty():
                c620_target_current[i] = que.get(block=False)
        c620_set_current(bus, c620_target_current)
        
        #GM6020
        for i, que in enumerate(gm6020_queues):
            if not que.empty():
                gm6020_target_voltage[i] = que.get(block=False)
        gm6020_set_volt(bus, gm6020_target_voltage)
        
        time.sleep(0.001)
    
    notifier.stop()
    bus.shutdown()
    
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('can_node')
    
    c620_queues = []
    c620_rpm_publishes = []
    
    for i in range(2):
        c620_queues.append(queue.Queue())
        p = node.create_publisher(Float64, f'~/c620_{i}/rpm', 10)
        c620_rpm_publishes.append(p)

    c620_s0 = node.create_subscription(Float64, f'~/c620_0/target_current', lambda msg: c620_queues[0].put(msg.data), 10)
    c620_s1 = node.create_subscription(Float64, f'~/c620_1/target_current', lambda msg: c620_queues[1].put(msg.data), 10)
    
    gm6020_queues = []
    gm6020_deg_publishes = []

    for i in range(2):
        gm6020_queues.append(queue.Queue())
        p = node.create_publisher(Float64, f'~/gm6020_{i}/degree', 10)
        gm6020_deg_publishes.append(p)

    gm6020_s0 = node.create_subscription(Int64, f'~/gm6020_0/target_volt', lambda msg: gm6020_queues[0].put(msg.data), 10)
    gm6020_s1 = node.create_subscription(Int64, f'~/gm6020_1/target_volt', lambda msg: gm6020_queues[1].put(msg.data), 10)
    
    task = Thread(target=can_task, args=[c620_queues, c620_rpm_publishes, gm6020_queues, gm6020_deg_publishes])
    task.start()

    rclpy.spin(node)
    
    exit_flag = True
    task.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
