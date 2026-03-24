import pytest
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'buggy_brain'))
from map_graph import find_shortest_path, get_node_coordinates, get_path_coordinates

def test_hub_to_ist():
    path = find_shortest_path('BUGGY_HUB', 'SRM_IST')
    assert path == ['BUGGY_HUB', 'SRM_IST']

def test_hub_to_hosp():
    path = find_shortest_path('BUGGY_HUB', 'SRM_HOSP')
    assert path == ['BUGGY_HUB', 'SRM_HOSP']

def test_hub_to_temple():
    path = find_shortest_path('BUGGY_HUB', 'SRM_TEMPLE')
    assert path == ['BUGGY_HUB', 'SRM_TEMPLE']

def test_ist_to_hosp_via_hub():
    path = find_shortest_path('SRM_IST', 'SRM_HOSP')
    assert path == ['SRM_IST', 'BUGGY_HUB', 'SRM_HOSP']

def test_temple_to_hosp_via_hub():
    path = find_shortest_path('SRM_TEMPLE', 'SRM_HOSP')
    assert path == ['SRM_TEMPLE', 'BUGGY_HUB', 'SRM_HOSP']

def test_same_start_and_goal():
    path = find_shortest_path('BUGGY_HUB', 'BUGGY_HUB')
    assert path == ['BUGGY_HUB']

def test_invalid_node_returns_empty():
    path = find_shortest_path('INVALID', 'SRM_IST')
    assert path == []

def test_all_nodes_reachable_from_hub():
    for node in ['SRM_IST', 'SRM_HOSP', 'SRM_TEMPLE']:
        path = find_shortest_path('BUGGY_HUB', node)
        assert len(path) > 0

def test_get_node_coordinates():
    assert get_node_coordinates('BUGGY_HUB')  == (0, 0)
    assert get_node_coordinates('SRM_IST')    == (0, 50)
    assert get_node_coordinates('SRM_HOSP')   == (50, 0)
    assert get_node_coordinates('SRM_TEMPLE') == (0, -50)

def test_get_node_coordinates_invalid():
    assert get_node_coordinates('NOWHERE') is None

def test_get_path_coordinates():
    path   = find_shortest_path('BUGGY_HUB', 'SRM_IST')
    coords = get_path_coordinates(path)
    assert coords == [(0, 0), (0, 50)]
