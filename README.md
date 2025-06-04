R & P DEMONSTRATION REPOSITORY OF MERRAN ROMP & SVEN ZUIDEMA

Dependencies
Python 3.11.9
Numpy 2.2.5
Thonny 3.1.1
ESP32 with micropython 1.25.0
Webots R2025a
(Matplotlib if you want the plot on the Webots)

How do you run the code:
1. Get you ESP32 connected to the pc and install micropython on it
2. Upload the ESP32 code on to the esp as Main.py
3. start up WeBots and open RaFLite.wbt to get the world
4. Make a new controller and upload the WeBots_controller.py in to the controller and save
5. Run the Main.py programme in Thonny and when it started close off Thonny
6. In WeBots you can press play after you did that press the button on the ESP32 board

To change the path go in the ESP32 code in line 61 and change the location of the nodes (do make sure the e-puck starts at the first node) and when u changed that you can restart from step 5
If you dont have a button on your ESP32 then connect a button to D34

The graph and node infromation 
graph = {
    'A1': [('A2', 1), ('C1', 2.5)],
    'A2': [('A1', 1), ('A3', 1)],
    'A3': [('A2', 1), ('A4', 1)],
    'A4': [('A3', 1), ('A5', 2)],
    'A5': [('A4', 2), ('B5', 1.5), ('A9', 5)],
    'A9': [('A5', 5), ('B9', 1.5)],

    'B5': [('A5', 1.5), ('B9', 5), ('C5', 1)],
    'B9': [('A9', 1.5), ('B5', 5), ('C9', 1)],

    'C1': [('D1', 1), ('C5', 5), ('A1', 2.5)],
    'C5': [('C1', 5), ('B5', 1), ('C9', 5), ('D5', 1)],
    'C9': [('B9', 1), ('C5', 5), ('E9', 2.5)],

    'D1': [('C1', 1), ('D5', 5), ('E1', 1.5)],
    'D5': [('D1', 5), ('C5', 1), ('E5', 1.5)],

    'E1': [('D1', 1.5), ('E5', 5)],
    'E5': [('E1', 5), ('D5', 1.5), ('E6', 2)],
    'E6': [('E5', 2), ('E7', 1)],
    'E7': [('E6', 1), ('E8', 1)],
    'E8': [('E7', 1), ('E9', 1)],
    'E9': [('E8', 1), ('C9', 2.5)],
}

node_coords = {
    'A1': (-0.50, 0.25),
    'A2': (-0.40, 0.25),
    'A3': (-0.30, 0.25),
    'A4': (-0.20, 0.25),
    'A5': (0.00, 0.25),
    'A9': (0.50, 0.25),
    'B5': (0.00, 0.10),
    'B9': (0.50, 0.10),
    'C1': (-0.50, 0.00),
    'C5': (0.00, 0.00),
    'C9': (0.50, 0.00),
    'D1': (-0.50, -0.10),
    'D5': (0.00, -0.10),
    'E1': (-0.50, -0.25),
    'E5': (0.00, -0.25),
    'E6': (0.20, -0.25),
    'E7': (0.30, -0.25),
    'E8': (0.40, -0.25),
    'E9': (0.50, -0.25)
}