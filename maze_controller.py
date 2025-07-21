"""maze_controller controller."""

from controller import Robot

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # ENABLE MOTORS
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # ENABLE SENSORS
    prox_sensor = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        sensor = robot.getDistanceSensor(sensor_name)
        sensor.enable(timestep)
        prox_sensor.append(sensor)

    while robot.step(timestep) != -1:
        # Read sensor values
        for ind in range(8):
            print("ind: {}, val: {}".format(ind, prox_sensor[ind].getValue()))

        left_wall = prox_sensor[5].getValue() > 80
        left_corner = prox_sensor[6].getValue() > 80
        front_wall = prox_sensor[7].getValue() > 80

        left_speed = max_speed
        right_speed = max_speed

        if front_wall:
            print("Turn right in place")
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                print("Drive forward")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print("Turn left")
                left_speed = max_speed / 8
                right_speed = max_speed

            if left_corner:
                print("Too close to left wall, adjust right")
                left_speed = max_speed
                right_speed = max_speed / 8

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)