
# This code defines a graph structure with nodes and their connections.
graph = {
    'A1': {'pos': (-5, 4), 'neighbors': {'B1': 1}},
    'A2': {'pos': (-4, 4), 'neighbors': {'B2': 1}},
    'A3': {'pos': (-3, 4), 'neighbors': {'B3': 1}},
    'A4': {'pos': (-2, 4), 'neighbors': {'B4': 1}},

    'B1': {'pos': (-5, 3), 'neighbors': {'A1': 1, 'B2': 1, 'D1': 2}},
    'B2': {'pos': (-4, 3), 'neighbors': {'A2': 1, 'B1': 1, 'B3': 1,}},
    'B3': {'pos': (-3, 3), 'neighbors': {'A3': 1, 'B2': 1, 'B4': 1,}},
    'B4': {'pos': (-2, 3), 'neighbors': {'A4': 1, 'B3': 4, 'B5': 2,}},
    'B5': {'pos': (0, 3), 'neighbors': {'B4': 2, 'B9': 5, 'D5': 3,}},
    'B9': {'pos': (5, 3), 'neighbors': {'B5': 5, 'C9': 2}},

    'C5': {'pos': (0, 1), 'neighbors': {'B5': 2, 'C9': 5, 'D5': 1}},
    'C9': {'pos': (5, 1), 'neighbors': {'C5': 5, 'B9': 2, 'D9':1}},

    'D1': {'pos': (-5, 0), 'neighbors': {'B1': 3, 'D5': 5, 'E1': 1}},
    'D5': {'pos': (0, 0), 'neighbors': {'D1': 5, 'E5': 1, 'D9': 5, 'C5': 1,}},
    'D9': {'pos': (5, 0), 'neighbors': {'D5': 5, 'C9': 1, 'F9': 3,}},

    'E1': {'pos': (-5, -1), 'neighbors': {'D1': 1, 'F1': 2, 'E5': 5}},
    'E5': {'pos': (0, -1), 'neighbors': {'E1': 5, 'D5': 1, 'F5': 2,}},

    'F1': {'pos': (-5, -3), 'neighbors': {'E1': 2, 'F5': 5}},
    'F5': {'pos': (0, -3), 'neighbors': {'F1': 5, 'E5': 2, 'F6': 2}},
    'F6': {'pos': (2, -3), 'neighbors': {'F5': 2, 'F7': 1, 'G6': 1}},
    'F7': {'pos': (3, -3), 'neighbors': {'F6': 1, 'F8': 1, 'G7': 1}},
    'F8': {'pos': (4, -3), 'neighbors': {'F7': 1, 'F9': 1, 'G8': 1}},
    'F9': {'pos': (5, -3), 'neighbors': {'F8': 1, 'G9': 1, 'D9': 3}},

    'G6': {'pos': (2,-4), 'neighbors': {'F6': 1}},
    'G7': {'pos': (3, -4),'neighbors': {'F7': 1}},
    'G8': {'pos': (4,-4), 'neighbors': {'F8': 1}},
    'G9': {'pos': (5, -4), 'neighbors': {'F9': 1}},
}