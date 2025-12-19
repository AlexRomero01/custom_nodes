import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from math import radians, cos, sin, sqrt, atan2

# ==========================
#  Global Config
# ==========================

PRINT_INFO = True

rows = {
    "section1 (solar opaque)": {
        "row1-2": {"N": {"lat": 41.28795045, "lon": 2.04391535}, "S": {"lat": 41.287786, "lon": 2.0439733}},
        "row2-3": {"N": {"lat": 41.2879391, "lon": 2.04390545}, "S": {"lat": 41.28777615, "lon": 2.04396275}},
        "row3-4": {"N": {"lat": 41.287934345, "lon": 2.04387395}, "S": {"lat": 41.28777085, "lon": 2.0439341}},
        "row4-5": {"N": {"lat": 41.287933395, "lon": 2.04384965}, "S": {"lat": 41.28776495, "lon": 2.0439096}},
        "row5-6": {"N": {"lat": 41.2879266, "lon": 2.04383785}, "S": {"lat": 41.2877586, "lon": 2.04389625}},
        "row6-7": {"N": {"lat": 41.2879228, "lon": 2.0438055}, "S": {"lat": 41.28775595, "lon": 2.0438658}},
        "row7-8": {"N": {"lat": 41.2879194, "lon": 2.04378085}, "S": {"lat": 41.287752, "lon": 2.0438422}},
        "row8-9": {"N": {"lat": 41.2879124, "lon": 2.04376925}, "S": {"lat": 41.28774585, "lon": 2.04382865}},
        "row9-10": {"N": {"lat": 41.28790795, "lon": 2.04374125}, "S": {"lat": 41.28774515, "lon": 2.0437953}},
        "row10-11": {"N": {"lat": 41.2879038, "lon": 2.0437127}, "S": {"lat": 41.2877464, "lon": 2.04376645}},
        "row11-12": {"N": {"lat": 41.28789725, "lon": 2.04369785}, "S": {"lat": 41.28774005, "lon": 2.04375415}},
    },
    "section2 (solar semi)" : {
        "row1-2": {"N": {"lat": 41.28799645, "lon": 2.04417645}, "S": {"lat": 41.2878341, "lon": 2.04423515}},
        "row2-3": {"N": {"lat": 41.2879938, "lon": 2.0441546}, "S": {"lat": 41.28783065, "lon": 2.0442118}},
        "row3-4": {"N": {"lat": 41.28799155, "lon": 2.04413305}, "S": {"lat": 41.28782815, "lon": 2.0441896}},
        "row4-5": {"N": {"lat": 41.2879811, "lon": 2.0441115}, "S": {"lat": 41.2878246, "lon": 2.0441662}},
        "row5-6": {"N": {"lat": 41.2879662, "lon": 2.04408135}, "S": {"lat": 41.2878162, "lon": 2.04414025}},
        "row6-7": {"N": {"lat": 41.28796375, "lon": 2.04404955}, "S": {"lat": 41.2878106, "lon": 2.0441147}},
        "row7-8": {"N": {"lat": 41.2879671, "lon": 2.04402915}, "S": {"lat": 41.28781125, "lon": 2.04408985}},
    },
    "open air": {
        "row1-2": {"N": {"lat": 41.288049995, "lon": 2.04442805}, "S": {"lat": 41.28788735, "lon": 2.0444869}},
        "row2-3": {"N": {"lat": 41.288044795, "lon": 2.0444113}, "S": {"lat": 41.28788565, "lon": 2.04446855}},
        "row3-4": {"N": {"lat": 41.288041595, "lon": 2.0443915}, "S": {"lat": 41.2878844, "lon": 2.04444585}},
        "row4-5": {"N": {"lat": 41.2880357, "lon": 2.04437205}, "S": {"lat": 41.2878797, "lon": 2.0444266}},
        "row5-6": {"N": {"lat": 41.288030145, "lon": 2.04435095}, "S": {"lat": 41.2878699, "lon": 2.04440825}},
        "row6-7": {"N": {"lat": 41.28802909, "lon": 2.044331}, "S": {"lat": 41.28786505, "lon": 2.04438705}},
        "row7-8": {"N": {"lat": 41.288024745, "lon": 2.04430955}, "S": {"lat": 41.28785875, "lon": 2.0443678}},
    }
}

# ==========================
# Helper functions
# ==========================

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c

def gps_to_local(lat_ref, lon_ref, lat, lon):
    x = haversine(lat_ref, lon_ref, lat_ref, lon) * (1 if lon > lon_ref else -1)
    y = haversine(lat_ref, lon_ref, lat, lon_ref) * (1 if lat > lat_ref else -1)
    return x, y

def project_point_on_row(lat, lon, N, S):
    Nx, Ny = 0.0, 0.0
    Sx, Sy = gps_to_local(N["lat"], N["lon"], S["lat"], S["lon"])
    Px, Py = gps_to_local(N["lat"], N["lon"], lat, lon)

    dx, dy = Sx - Nx, Sy - Ny
    if dx == 0 and dy == 0:
        return 0.0, sqrt(Px**2 + Py**2), 0.0

    t = ((Px - Nx) * dx + (Py - Ny) * dy) / (dx*dx + dy*dy)
    t_clamped = max(0, min(1, t))

    proj_x = Nx + t_clamped * dx
    proj_y = Ny + t_clamped * dy

    dist_from_N = sqrt(proj_x**2 + proj_y**2)
    dist_perp = sqrt((Px - proj_x)**2 + (Py - proj_y)**2)
    return dist_from_N, dist_perp, sqrt(dx*dx + dy*dy)

# ==========================
# ROS2 Node
# ==========================

class GPSRowDetector(Node):
    def __init__(self):
        super().__init__('gps_row_detector')
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', self.listener_callback, 10)
        self.pub_info = self.create_publisher(String, '/navigation/information', 10)
        self.last_dist_from_N = None
        self.last_row = None
        self.get_logger().info("GPSRowDetector initialized and listening...")

    def listener_callback(self, msg: NavSatFix):
        lat, lon = msg.latitude, msg.longitude

        # 1️⃣ Identificar la fila más cercana
        best_section = best_row = None
        best_dist_from_N = None
        min_perp_dist = float("inf")

        for section, section_rows in rows.items():
            for row_name, coords in section_rows.items():
                dist_from_N, dist_perp, _ = project_point_on_row(lat, lon, coords["N"], coords["S"])
                if dist_perp < min_perp_dist:
                    min_perp_dist = dist_perp
                    best_section, best_row = section, row_name
                    best_dist_from_N = dist_from_N

        # 2️⃣ Ajustar fila real según dirección
        direction = "unknown"
        real_row = best_row
        if self.last_row == (best_section, best_row) and self.last_dist_from_N is not None:
            if best_dist_from_N > self.last_dist_from_N:
                direction = "North → South"
                row_num = int(best_row.split('-')[0].replace('row',''))
                if row_num % 2 == 0:
                    real_row = f"row{row_num-1}-{row_num}"
            elif best_dist_from_N < self.last_dist_from_N:
                direction = "South → North"
                row_num = int(best_row.split('-')[0].replace('row',''))
                if row_num % 2 == 1:
                    real_row = f"row{row_num+1}-{row_num+2}"

        self.last_dist_from_N, self.last_row = best_dist_from_N, (best_section, best_row)

        # 3️⃣ Publicar información
        info_str = (
            f"section: {best_section}, "
            f"row: {real_row}, "
            f"position_from_N: {best_dist_from_N:.2f} m, "
            f"direction: {direction}"
        )
        if PRINT_INFO:
            self.get_logger().info(info_str)
        self.pub_info.publish(String(data=info_str))

# ==========================
# Main
# ==========================

def main(args=None):
    rclpy.init(args=args)
    node = GPSRowDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
