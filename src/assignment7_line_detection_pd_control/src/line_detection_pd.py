'''
The Algorithm:
1. Subscribe to camera topic
2. Detect line and obtain line equation with Ransac (code from assignment 5)
3. Calculate an error using PD controller (new code created for assignment 7)
4. Create a command for an actuator (code from assignment 6)
5. Move a mobile robot
6. Detect the new line position and calculate a new error and a new command and perform the next movement
'''

