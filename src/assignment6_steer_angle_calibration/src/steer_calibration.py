import matplotlib.pyplot as plt
import numpy as np
import math
import xml.etree.ElementTree as ET
from sklearn import linear_model

max_left = None
max_right = None
slope = None
intercept = None

def ransac_method(commands):
    """
    Get the linear model of lines
    :param commands: dictionary of commands and corresponding angels
    :return: parameters m and b of the linear model
    """

    ransac = linear_model.RANSACRegressor()
    
    # We need to fing the corresponding commands (y-axis) for function
    angles_x = []
    commands_y = []
    
    for cmd, angle in commands.items():
        commands_y.append([cmd])
        angles_x.append([angle])
    
    ransac.fit(angles_x, commands_y)
    b = ransac.estimator_.intercept_
    m = ransac.estimator_.coef_

    return m, b

def make_turn(angle):
    if angle < max_left:
        angle = max_left
    elif angle > max_right:
        angle = max_right
    
    # convert deg to rad
    angle = math.radians(angle)
    
    command = slope * angle + intercept
    return int(command)

#=========================
#Parse the XML file
def parse_xml(file_name):
    tree = ET.parse(file_name)
    root = tree.getroot()
    items = root.findall("./myPair/item")
    
    commands = {int(i.find('command').text) : float(i.find('steering').text) for i in items}
    print(commands)
    
    return commands

#==========================
def main():
    
    file_name = 'SteerAngleActuator_03Dec.xml'
    commands = parse_xml(file_name)
     
    plt.figure()
    plt.xlabel('Angles in grad')
    plt.ylabel('Commands')
    plt.plot(list(map(math.degrees, commands.values())), commands.keys(), 'rx')
    plt.show()
    
    global slope, intercept
    slope, intercept = ransac_method(commands)
    print("Equation line 1: y = %fx + %f" % (slope, intercept))

    # define borders
    global max_left, max_right
    max_left = math.degrees((179-intercept)/slope)
    max_right = math.degrees(-intercept/slope)
    print(max_left, max_right)
       
    
    print("Test:")
    angles = [-45, -30, -15, 0, 15, 30, 45]
    for a in angles:
        print('Command for %i grad: %i' % (a, make_turn(a)))
        
        
if __name__ == '__main__':
    main()


