import pytest
import numpy as np
from ros_speckle_bridge.converter import Converter
from bim_interfaces.msg import Property

class MockSpeckleObject:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

def test_transform_point_identity():
    """Test coordinate transformation without datum"""
    converter = Converter(datum=[0.0, 0.0, 0.0])
    
    # BIM point: [1, 2, 3] (x, y=up, z=forward)
    # Expected ROS: [1, 3, 2] (x, y=forward, z=up)
    # Wait, let's check the matrix in converter.py
    # [1, 0, 0]
    # [0, 0, 1]
    # [0, -1, 0]
    # [1, 2, 3] -> [1, 3, -2] ?
    
    # X_ros = 1*1 + 0*2 + 0*3 = 1
    # Y_ros = 0*1 + 0*2 + 1*3 = 3
    # Z_ros = 0*1 - 1*2 + 0*3 = -2
    
    bim_point = [1.0, 2.0, 3.0]
    ros_point = converter.transform_point(bim_point)
    
    assert ros_point == (1.0, 3.0, -2.0)

def test_transform_point_with_datum():
    """Test coordinate transformation with datum offset"""
    # Datum is subtracted BEFORE rotation
    converter = Converter(datum=[10.0, 20.0, 0.0])
    
    # BIM point: [11, 22, 3]
    # After datum: [1, 2, 3]
    # After rotation: [1, 3, -2]
    
    bim_point = [11.0, 22.0, 3.0]
    ros_point = converter.transform_point(bim_point)
    
    assert np.allclose(ros_point, (1.0, 3.0, -2.0))

def test_extract_properties_dict():
    """Test property extraction from dictionary"""
    converter = Converter()
    
    data = {
        "id": "ignore_me",
        "Height": 3.0,
        "Material": "Concrete",
        "isExternal": True
    }
    
    props = converter.extract_properties(data)
    
    assert len(props) == 3
    
    # Convert to dict for easier checking
    prop_dict = {p.key: (p.value, p.type) for p in props}
    
    assert prop_dict["Height"] == ("3.0", "Number")
    assert prop_dict["Material"] == ("Concrete", "Text")
    assert prop_dict["isExternal"] == ("True", "Boolean")

def test_extract_properties_object():
    """Test property extraction from object"""
    converter = Converter()
    
    obj = MockSpeckleObject(
        id="ignore_me",
        Width=0.5,
        Name="Wall-01"
    )
    
    props = converter.extract_properties(obj)
    
    assert len(props) == 2
    prop_dict = {p.key: p.value for p in props}
    
    assert prop_dict["Width"] == "0.5"
    assert prop_dict["Name"] == "Wall-01"
