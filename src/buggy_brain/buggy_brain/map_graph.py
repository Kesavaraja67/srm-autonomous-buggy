import heapq

# Campus nodes with intermediate waypoints for smoother navigation
NODES = {
    'BUGGY_HUB':   (  0.0,   0.0),
    'SRM_IST':     (  0.0,  50.0),
    'SRM_HOSP':    ( 50.0,   0.0),
    'SRM_TEMPLE':  (  0.0, -50.0),
}

EDGES = {
    'BUGGY_HUB':  [('HUB_N', 13), ('HUB_E', 13), ('HUB_S', 13)],
    'HUB_N':      [('BUGGY_HUB', 13), ('SRM_IST', 38), ('HUB_E', 17), ('HUB_S', 24)],
    'HUB_E':      [('BUGGY_HUB', 13), ('SRM_HOSP', 38), ('HUB_N', 17), ('HUB_S', 17)],
    'HUB_S':      [('BUGGY_HUB', 13), ('SRM_TEMPLE', 38), ('HUB_N', 24), ('HUB_E', 17)],
    'SRM_IST':    [('HUB_N', 38)],
    'SRM_HOSP':   [('HUB_E', 38)],
    'SRM_TEMPLE': [('HUB_S', 38)],
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
    ]
    for start, goal in tests:
        path = find_shortest_path(start, goal)
        coords = get_path_coordinates(path)
        print(f'{start} -> {goal}: {path}')
        print(f'  coords: {coords}')

