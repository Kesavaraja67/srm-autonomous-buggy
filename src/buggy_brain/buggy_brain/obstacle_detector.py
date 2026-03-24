#!/usr/bin/env python3
"""
obstacle_detector.py  —  Team Bravo (v3)
-----------------------------------------
Processes /scan (LaserScan) to detect:
1. Wide objects (Cylinders/Blocks) -> STOP (/obstacle_detected = True)
2. Narrow objects (Trees/Lights)     -> AVOID (/avoidance_steering offset)

Algorithm:
- Segment Lidar points in front arc (±30 degrees).
- Group consecutive hits into clusters.
- If a cluster is wider than WIDTH_THRESHOLD -> STOP.
- If a cluster is narrow -> ADJUST steering to nudge away.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

# Thresholds
FRONT_ARC_DEG = 25.0   # Narrowed arc to avoid seeing walls/pillars
STOP_DIST     = 2.2    # Metres - closer stop threshold
AVOID_DIST    = 4.0    # Metres - start nudging sooner
WIDTH_THR     = 0.4    # Metres - clusters wider than this are STOP objects
STEER_GAIN    = 0.8    # Rad/s steering offset per degree of avoidance

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self._stop_pub  = self.create_publisher(Bool, '/obstacle_detected', 10)
        self._steer_pub = self.create_publisher(Float32, '/avoidance_steering', 10)

        self.get_logger().info('ObstacleDetectorNode v3 started (Intelligent Avoidance).')

    def _scan_callback(self, msg: LaserScan):
        # 1. Extract front arc points
        # Assuming 0 is front, angles -PI to PI
        num_points = len(msg.ranges)
        angle_inc  = msg.angle_increment
        
        # Ranges in front arc
        front_points = []
        for i in range(num_points):
            angle = msg.angle_min + i * angle_inc
            # Normalise angle to [-PI, PI]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            if abs(math.degrees(angle)) <= FRONT_ARC_DEG:
                r = msg.ranges[i]
                if msg.range_min < r < AVOID_DIST:
                    front_points.append({'r': r, 'angle': angle})

        # 2. Simple clustering
        clusters = []
        if front_points:
            current_cluster = [front_points[0]]
            for j in range(1, len(front_points)):
                # If points are close in range AND angle, same cluster
                range_diff = abs(front_points[j]['r'] - front_points[j-1]['r'])
                angle_diff = abs(front_points[j]['angle'] - front_points[j-1]['angle'])
                
                if range_diff < 0.5 and angle_diff < 0.05:
                    current_cluster.append(front_points[j])
                else:
                    clusters.append(current_cluster)
                    current_cluster = [front_points[j]]
            clusters.append(current_cluster)

        # 3. Analyze clusters
        should_stop = False
        steer_offset = 0.0

        for cluster in clusters:
            # Estimate width
            if not cluster:
                continue
            
            # Simple width: r * dTheta
            avg_r = sum(p['r'] for p in cluster) / len(cluster)
            angle_span = abs(cluster[-1]['angle'] - cluster[0]['angle'])
            width = avg_r * angle_span

            # Stop logic
            if avg_r < STOP_DIST and width > WIDTH_THR:
                should_stop = True
                break
            
            # Avoid logic (if not stopping)
            if avg_r < AVOID_DIST and width <= WIDTH_THR:
                # Nudge away from the object
                avg_angle = sum(p['angle'] for p in cluster) / len(cluster)
                # If object is at -10 deg, steer +ve (RIGHT nudge)
                # If object is at +10 deg, steer -ve (LEFT nudge)
                steer_offset -= STEER_GAIN * avg_angle

        # 4. Publish
        stop_msg = Bool()
        stop_msg.data = should_stop
        self._stop_pub.publish(stop_msg)

        steer_msg = Float32()
        steer_msg.data = steer_offset
        self._steer_pub.publish(steer_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            # Shutdown errors are usually benign (e.g., already shut down)
            print(f'Shutdown warning (ObstacleDetectorNode): {e}')

if __name__ == '__main__':
    main()
