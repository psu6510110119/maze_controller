"""maze_controller controller."""

from controller import Robot
import math

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())  # [ms]
    delta_t = timestep / 1000.0               # [s]

    max_speed = 6.28                          # [rad/s]
    R = 0.0205                                # wheel radius [m] (example)
    D = 0.052                                 # distance between wheels [m] (example)

    # Initial pose
    x = 0.0
    y = 0.0
    phi = 0.0

    # ENABLE MOTORS
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # ENABLE ENCODERS
    left_encoder = left_motor.getPositionSensor()
    right_encoder = right_motor.getPositionSensor()
    left_encoder.enable(timestep)
    right_encoder.enable(timestep)

    # READ INITIAL ENCODER VALUE
    old_left_enc = left_encoder.getValue()
    old_right_enc = right_encoder.getValue()

    # ENABLE SENSORS (Optional)
    prox_sensor = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        sensor = robot.getDevice(sensor_name)
        sensor.enable(timestep)
        prox_sensor.append(sensor)

    while robot.step(timestep) != -1:
        # === ODOMETRY CALCULATION ===

        # Read current encoder values
        new_left_enc = left_encoder.getValue()
        new_right_enc = right_encoder.getValue()

        # Δtheta of wheels
        dtheta_l = new_left_enc - old_left_enc
        dtheta_r = new_right_enc - old_right_enc

        # Update old values
        old_left_enc = new_left_enc
        old_right_enc = new_right_enc

        # Compute wheel speeds (rad/s)
        wl = dtheta_l / delta_t
        wr = dtheta_r / delta_t

        # Compute robot linear (u) and angular (w) speed
        u = R * (wr + wl) / 2.0
        w = R * (wr - wl) / D

        # Update robot pose
        dx = u * math.cos(phi)
        dy = u * math.sin(phi)
        dphi = w

        x += dx * delta_t
        y += dy * delta_t
        phi += dphi * delta_t

        print("Pose: x = {:.3f}, y = {:.3f}, phi = {:.3f}".format(x, y, phi))

        # === MOVEMENT CONTROL ===

        front_wall = prox_sensor[7].getValue() > 80
        left_wall = prox_sensor[5].getValue() > 80
        left_corner = prox_sensor[6].getValue() > 80

        left_speed = max_speed
        right_speed = max_speed

        if front_wall:
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                left_speed = max_speed
                right_speed = max_speed
            else:
                left_speed = max_speed / 8
                right_speed = max_speed

            if left_corner:
                left_speed = max_speed
                right_speed = max_speed / 8

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
