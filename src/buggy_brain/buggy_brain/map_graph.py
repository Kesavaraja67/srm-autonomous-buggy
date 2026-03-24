#!/usr/bin/env python3
import heapq

NODES = {
    'BUGGY_HUB':  (-13.0,   0.0),
    'SRM_IST':    (-12.0,  50.0),
    'SRM_HOSP':   ( 50.0,  12.0),
    'SRM_TEMPLE': (-12.0, -50.0),
    'RND_N':      (  0.0,   8.0),
    'RND_S':      (  0.0,  -8.0),
    'RND_E':      (  8.0,   0.0),
    'RND_W':      ( -8.0,   0.0),
}

EDGES = {
    'BUGGY_HUB':  [('RND_W', 5.0)],
    'SRM_IST':    [('RND_N', 42.0)],
    'SRM_HOSP':   [('RND_E', 42.0)],
    'SRM_TEMPLE': [('RND_S', 42.0)],
    'RND_N':      [('SRM_IST', 42.0), ('RND_W', 12.0), ('RND_E', 12.0)],
    'RND_S':      [('SRM_TEMPLE', 42.0), ('RND_W', 12.0), ('RND_E', 12.0)],
    'RND_E':      [('SRM_HOSP', 42.0), ('RND_N', 12.0), ('RND_S', 12.0)],
    'RND_W':      [('BUGGY_HUB', 5.0), ('RND_N', 12.0), ('RND_S', 12.0)],
}

VALID_DESTINATIONS = {
    'A': 'SRM_IST',
    'B': 'SRM_HOSP',
    'C': 'SRM_TEMPLE',
    'D': 'BUGGY_HUB',
}

DESTINATION_DISPLAY = {
    'SRM_IST':    'SRM Institute of Science & Technology (North)',
    'SRM_HOSP':   'SRM Hospital / Medical College (East)',
    'SRM_TEMPLE': 'SRM Campus Temple (South)',
    'BUGGY_HUB':  'Buggy Hub — Home Base',
}

def find_shortest_path(start, goal):
    if start not in NODES or goal not in NODES:
        return []
    if start == goal:
        return [start]
    queue    = [(0, start)]
    visited  = set()
    previous = {}
    distance = {node: float('inf') for node in NODES}
    distance[start] = 0
    while queue:
        cost, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            path = []
            while current in previous:
                path.append(current)
                current = previous[current]
            path.append(start)
            return list(reversed(path))
        for neighbour, weight in EDGES.get(current, []):
            new_cost = cost + weight
            if new_cost < distance[neighbour]:
                distance[neighbour] = new_cost
                previous[neighbour] = current
                heapq.heappush(queue, (new_cost, neighbour))
    return []

def get_node_coordinates(node_name):
    return NODES.get(node_name, None)

def get_path_coordinates(path):
    return [NODES[node] for node in path if node in NODES]

if __name__ == '__main__':
    tests = [
        ('BUGGY_HUB', 'SRM_IST'),
        ('BUGGY_HUB', 'SRM_HOSP'),
        ('BUGGY_HUB', 'SRM_TEMPLE'),
        ('SRM_IST',   'SRM_HOSP'),
        ('SRM_IST',   'SRM_TEMPLE'),
        ('SRM_TEMPLE','BUGGY_HUB'),
    ]
    for start, goal in tests:
        path = find_shortest_path(start, goal)
        coords = get_path_coordinates(path)
        print(f'{start} -> {goal}: {path}')
        print(f'  coords: {coords}')
