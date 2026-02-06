import pytest
import json
from unittest.mock import MagicMock
import sys

# Mock geometry_msgs
sys.modules["geometry_msgs"] = MagicMock()
sys.modules["geometry_msgs.msg"] = MagicMock()

from geometry_msgs.msg import Twist

# ==== JOYSTICK PARSING TESTS ====
def parse_bluetooth_json(data_str):
    """Extract joystick parsing logic from ros2_node.py for testing"""
    data = json.loads(data_str)
    msg_type = data.get("type")
    
    if msg_type == "joystick":
        twist = Twist()
        JOY_SCALE = 70.0
        MAX_LIN_VEL = 1.0
        MAX_ANG_VEL = 2.0

        l_x = float(data.get("l_x", 0.0))
        l_y = float(data.get("l_y", 0.0))
        r   = float(data.get("r", 0.0))

        twist.linear.x = -(l_y / JOY_SCALE) * MAX_LIN_VEL
        twist.linear.y = -(l_x / JOY_SCALE) * MAX_LIN_VEL
        twist.angular.z = -(r / JOY_SCALE) * MAX_ANG_VEL
        return twist
    return None

class TestBTConverters:
    def test_joystick_parsing_forward(self):
        json_str = '{"type": "joystick", "l_x": 0, "l_y": -70, "r": 0}'
        twist = parse_bluetooth_json(json_str)
        
        assert twist is not None
        assert twist.linear.x == pytest.approx(1.0, 0.01)
        assert twist.linear.y == 0
        assert twist.angular.z == 0

    def test_joystick_parsing_left(self):
        json_str = '{"type": "joystick", "l_x": -70, "l_y": 0, "r": 0}'
        twist = parse_bluetooth_json(json_str)
        
        assert twist.linear.y == pytest.approx(1.0, 0.01)

    def test_invalid_json(self):
        with pytest.raises(json.JSONDecodeError):
            parse_bluetooth_json("invalid")
    
    def test_joystick_partial_values(self):
        """Test with partial joystick input"""
        json_str = '{"type": "joystick", "l_x": 35, "l_y": -35, "r": 20}'
        twist = parse_bluetooth_json(json_str)
        
        assert twist.linear.x == pytest.approx(0.5, 0.01)  # -(-35/70)
        assert twist.linear.y == pytest.approx(-0.5, 0.01) # -(35/70)
        assert twist.angular.z == pytest.approx(-0.571, 0.01) # -(20/70)*2.0

    def test_joystick_max_angular(self):
        """Test maximum angular velocity"""
        json_str = '{"type": "joystick", "l_x": 0, "l_y": 0, "r": 70}'
        twist = parse_bluetooth_json(json_str)
        
        assert twist.angular.z == pytest.approx(-2.0, 0.01)
    
    def test_joystick_default_zero(self):
        """Test missing values default to zero"""
        json_str = '{"type": "joystick"}'
        twist = parse_bluetooth_json(json_str)
        
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0
        assert twist.angular.z == 0.0

# ==== VELOCITY SCALING TESTS ====
def scale_joystick_to_velocity(joy_value, joy_scale=70.0, max_vel=1.0):
    """Scale joystick input to velocity"""
    return -(joy_value / joy_scale) * max_vel

class TestVelocityScaling:
    def test_max_positive_scale(self):
        vel = scale_joystick_to_velocity(-70.0)
        assert vel == pytest.approx(1.0, 0.01)
    
    def test_max_negative_scale(self):
        vel = scale_joystick_to_velocity(70.0)
        assert vel == pytest.approx(-1.0, 0.01)
    
    def test_zero_scale(self):
        vel = scale_joystick_to_velocity(0.0)
        assert vel == 0.0
    
    def test_half_scale(self):
        vel = scale_joystick_to_velocity(-35.0)
        assert vel == pytest.approx(0.5, 0.01)

# ==== NAVIGATION COORDINATE CONVERSION TESTS ====
def convert_mm_to_meters(x_mm, y_mm):
    """Convert field coordinates from mm to meters for Nav2"""
    return x_mm / 1000.0, y_mm / 1000.0

class TestNavigationConversion:
    def test_origin_conversion(self):
        x_m, y_m = convert_mm_to_meters(0, 0)
        assert x_m == 0.0
        assert y_m == 0.0
    
    def test_max_field_conversion(self):
        x_m, y_m = convert_mm_to_meters(3500, 7000)
        assert x_m == 3.5
        assert y_m == 7.0
    
    def test_center_conversion(self):
        x_m, y_m = convert_mm_to_meters(1750, 3500)
        assert x_m == 1.75
        assert y_m == 3.5
    
    def test_arbitrary_conversion(self):
        x_m, y_m = convert_mm_to_meters(1234, 5678)
        assert x_m == pytest.approx(1.234, 0.001)
        assert y_m == pytest.approx(5.678, 0.001)
