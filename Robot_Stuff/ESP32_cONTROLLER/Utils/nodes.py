
# This code defines a graph structure with nodes and their connections.
graph = {
    'A1': {'pos': (0, 0), 'neighbors': {'B': 5, 'C': 2}},
    'A2': {'pos': (0, 0), 'neighbors': {'B': 5, 'C': 2}},
    'A3': {'pos': (0, 0), 'neighbors': {'B': 5, 'C': 2}},
    'A4': {'pos': (0, 0), 'neighbors': {'B': 5, 'C': 2}},

    'B1': {'pos': (100, 0), 'neighbors': {'A': 5, 'D': 4}},
    'B2': {'pos': (100, 0), 'neighbors': {'A': 5, 'D': 4}},
    'B3': {'pos': (100, 0), 'neighbors': {'A': 5, 'D': 4}},
    'B4': {'pos': (100, 0), 'neighbors': {'A': 5, 'D': 4}},
    'B5': {'pos': (100, 0), 'neighbors': {'A': 5, 'D': 4}},
    'B9': {'pos': (100, 0), 'neighbors': {'A': 5, 'D': 4}},

    'C5': {'pos': (0, 100), 'neighbors': {'A': 2, 'F': 7}},
    'C9': {'pos': (0, 100), 'neighbors': {'A': 2, 'F': 7}},

    'D1': {'pos': (200, 0), 'neighbors': {'B': 4}},
    'D5': {'pos': (200, 0), 'neighbors': {'B': 4}},
    'D9': {'pos': (200, 0), 'neighbors': {'B': 4}},

    'E1': {'pos': (100, 100), 'neighbors': {'B': 2, 'F': 1}},
    'E5': {'pos': (100, 100), 'neighbors': {'B': 2, 'F': 1}},

    'F1': {'pos': (200, 100), 'neighbors': {'C': 7, 'E': 1}},
    'F5': {'pos': (200, 100), 'neighbors': {'C': 7, 'E': 1}},
    'F6': {'pos': (200, 100), 'neighbors': {'C': 7, 'E': 1}},
    'F7': {'pos': (200, 100), 'neighbors': {'C': 7, 'E': 1}},
    'F8': {'pos': (200, 100), 'neighbors': {'C': 7, 'E': 1}},
    'F9': {'pos': (200, 100), 'neighbors': {'C': 7, 'E': 1}},

    'G6': {'pos': (300, 0), 'neighbors': {'D': 5, 'H': 2}},
    'G7': {'pos': (300, 0), 'neighbors': {'D': 5, 'H': 2}},
    'G8': {'pos': (300, 0), 'neighbors': {'D': 5, 'H': 2}},
    'G9': {'pos': (300, 0), 'neighbors': {'D': 5, 'H': 2}},
}