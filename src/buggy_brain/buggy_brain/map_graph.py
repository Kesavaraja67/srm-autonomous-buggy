import heapq

# Campus nodes — coordinates match srm_campus.world v3.0 exactly
NODES = {
    'BUGGY_HUB':  (  0.0,   0.0),
    'SRM_IST':    (  0.0,  50.0),
    'SRM_HOSP':   ( 50.0,   0.0),
    'SRM_TEMPLE': (  0.0, -50.0),
}

# Bidirectional edges with distances in metres
EDGES = {
    'BUGGY_HUB':  [('SRM_IST', 50), ('SRM_HOSP', 50), ('SRM_TEMPLE', 50)],
    'SRM_IST':    [('BUGGY_HUB', 50)],
    'SRM_HOSP':   [('BUGGY_HUB', 50)],
    'SRM_TEMPLE': [('BUGGY_HUB', 50)],
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
        ('BUGGY_HUB',  'SRM_IST'),
        ('BUGGY_HUB',  'SRM_HOSP'),
        ('BUGGY_HUB',  'SRM_TEMPLE'),
        ('SRM_IST',    'SRM_HOSP'),
        ('SRM_TEMPLE', 'SRM_HOSP'),
        ('BUGGY_HUB',  'BUGGY_HUB'),
    ]
    for start, goal in tests:
        path = find_shortest_path(start, goal)
        coords = get_path_coordinates(path)
        print(f'{start} -> {goal}: {path}')
        print(f'  coords: {coords}')
