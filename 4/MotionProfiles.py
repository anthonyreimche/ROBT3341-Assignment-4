"""
Motion Profile Functions for Trapezoidal Velocity Control
This module provides functions to calculate equations of motion for robotic systems.
"""

import math


def profile(displacement, start, time, ta):
    """
    Calculate the 3 equations of motion for a trapezoidal velocity profile.

    Args:
        displacement (float): The total change in position
        start (float): The initial position for a move
        time (list of floats): Contains each value of time to evaluate
        ta (float): The duration of the acceleration and deceleration

    Returns:
        tuple: Three lists containing the calculated equations of motion
            - displacement_list (list of floats): position calculated for each time step
            - velocity_list (list of floats): velocity calculated for each time step
            - acceleration_list (list of floats): acceleration calculated for each time step
    """
    displacement_list = []
    velocity_list = []
    acceleration_list = []

    # Calculate total time (last element in time list)
    t_total = time[-1] if time else 0

    # Calculate peak velocity
    # For trapezoidal profile: displacement = v_max * (t_total - ta)
    if t_total > ta:
        v_max = displacement / (t_total - ta)
    else:
        # If total time equals acceleration time, it's a triangular profile
        v_max = displacement / (t_total / 2)

    # Calculate acceleration
    if ta > 0:
        accel = v_max / ta
    else:
        accel = 0

    # Calculate motion for each time step
    for t in time:
        if t <= ta:
            # Acceleration phase
            pos = start + 0.5 * accel * t**2
            vel = accel * t
            acc = accel
        elif t <= (t_total - ta):
            # Constant velocity phase
            pos = start + 0.5 * accel * ta**2 + v_max * (t - ta)
            vel = v_max
            acc = 0
        elif t <= t_total:
            # Deceleration phase
            t_decel = t - (t_total - ta)
            pos = (
                start
                + 0.5 * accel * ta**2
                + v_max * (t_total - 2 * ta)
                + v_max * t_decel
                - 0.5 * accel * t_decel**2
            )
            vel = v_max - accel * t_decel
            acc = -accel
        else:
            # After motion complete
            pos = start + displacement
            vel = 0
            acc = 0

        displacement_list.append(pos)
        velocity_list.append(vel)
        acceleration_list.append(acc)

    return (displacement_list, velocity_list, acceleration_list)


def motion(displacement, interval, accel_limit, velo_limit):
    """
    Calculate the fastest time to perform a move given acceleration and velocity limits.

    Args:
        displacement (float): The total change in position
        interval (float): The time step interval
        accel_limit (float): The acceleration limit
        velo_limit (float): The velocity limit

    Returns:
        tuple: A list of time and ta to describe the shape of the motion
            - time (list of floats): contains the value of each time to calculate a motion profile
            - ta (float): the duration of the acceleration and deceleration
    """
    # Handle the case where displacement is negative
    abs_displacement = abs(displacement)

    # Calculate time to reach velocity limit
    t_accel_to_vmax = velo_limit / accel_limit

    # Calculate displacement during acceleration to vmax
    d_accel = 0.5 * accel_limit * t_accel_to_vmax**2

    # Check if we can reach velocity limit (trapezoidal profile)
    if 2 * d_accel <= abs_displacement:
        # Trapezoidal profile: we reach velocity limit
        ta = t_accel_to_vmax

        # Calculate constant velocity distance
        d_const = abs_displacement - 2 * d_accel

        # Calculate constant velocity time
        t_const = d_const / velo_limit

        # Total time
        t_total = 2 * ta + t_const
    else:
        # Triangular profile: we don't reach velocity limit
        # displacement = 0.5 * a * ta^2 + 0.5 * a * ta^2 = a * ta^2
        ta = math.sqrt(abs_displacement / accel_limit)
        t_total = 2 * ta

    # Generate time array
    num_steps = int(math.ceil(t_total / interval)) + 1
    time = [i * interval for i in range(num_steps)]

    # Ensure the last time step equals t_total
    if time[-1] < t_total:
        time.append(t_total)

    return (time, ta)


def joint_interpolation(
    displacement_a,
    start_a,
    displacement_b,
    start_b,
    interval,
    accel_limit_a,
    velo_limit_a,
    accel_limit_b,
    velo_limit_b,
):
    """
    Calculate the fastest time to coordinate a move between two joints.
    Both joints will complete their motion at the same time.

    Args:
        displacement_a (float): The total change in position for joint A
        start_a (float): The initial position for joint A
        displacement_b (float): The total change in position for joint B
        start_b (float): The initial position for joint B
        interval (float): The time step interval
        accel_limit_a (float): The acceleration limit for joint A
        velo_limit_a (float): The velocity limit for joint A
        accel_limit_b (float): The acceleration limit for joint B
        velo_limit_b (float): The velocity limit for joint B

    Returns:
        tuple: Two tuples and a list
            - eom_a (tuple of 3 lists of floats): the equations of motion (d, v, a) for joint A
            - eom_b (tuple of 3 lists of floats): the equations of motion (d, v, a) for joint B
            - time (list of floats): contains the value of each time to calculate a motion profile
    """
    # Calculate motion parameters for each joint independently
    time_a, ta_a = motion(displacement_a, interval, accel_limit_a, velo_limit_a)
    time_b, ta_b = motion(displacement_b, interval, accel_limit_b, velo_limit_b)

    # Find the longer time (this will be the coordinated time)
    t_total_a = time_a[-1]
    t_total_b = time_b[-1]

    if t_total_a >= t_total_b:
        # Joint A takes longer, so use its time
        t_total = t_total_a
        ta_coordinated_a = ta_a

        # Scale joint B to match the total time
        # Need to recalculate ta for joint B
        abs_displacement_b = abs(displacement_b)

        # Try to maintain trapezoidal profile for B
        # displacement = v_max * (t_total - ta)
        # v_max = a * ta
        # displacement = a * ta * (t_total - ta)
        # This is a quadratic equation: a * ta^2 - a * t_total * ta + displacement = 0

        # Check if we can use velocity limit
        t_accel_to_vmax_b = velo_limit_b / accel_limit_b
        d_accel_b = 0.5 * accel_limit_b * t_accel_to_vmax_b**2

        if 2 * d_accel_b <= abs_displacement_b and (t_total >= 2 * t_accel_to_vmax_b):
            # Can potentially reach velocity limit
            v_max_needed = abs_displacement_b / (t_total - t_accel_to_vmax_b)

            if v_max_needed <= velo_limit_b:
                # Use velocity limit
                ta_coordinated_b = t_accel_to_vmax_b
            else:
                # Need to slow down - use triangular profile
                ta_coordinated_b = t_total / 2
        else:
            # Use triangular profile
            ta_coordinated_b = t_total / 2
    else:
        # Joint B takes longer, so use its time
        t_total = t_total_b
        ta_coordinated_b = ta_b

        # Scale joint A to match the total time
        abs_displacement_a = abs(displacement_a)

        t_accel_to_vmax_a = velo_limit_a / accel_limit_a
        d_accel_a = 0.5 * accel_limit_a * t_accel_to_vmax_a**2

        if 2 * d_accel_a <= abs_displacement_a and (t_total >= 2 * t_accel_to_vmax_a):
            v_max_needed = abs_displacement_a / (t_total - t_accel_to_vmax_a)

            if v_max_needed <= velo_limit_a:
                ta_coordinated_a = t_accel_to_vmax_a
            else:
                ta_coordinated_a = t_total / 2
        else:
            ta_coordinated_a = t_total / 2

    # Generate coordinated time array
    num_steps = int(math.ceil(t_total / interval)) + 1
    time = [i * interval for i in range(num_steps)]

    if time[-1] < t_total:
        time.append(t_total)

    # Calculate motion profiles for both joints with coordinated timing
    eom_a = profile(displacement_a, start_a, time, ta_coordinated_a)
    eom_b = profile(displacement_b, start_b, time, ta_coordinated_b)

    return (eom_a, eom_b, time)


# Test functions
if __name__ == "__main__":
    # Test profile function
    print("Testing profile function:")
    test_time = [i * 0.1 for i in range(31)]  # 0 to 3 seconds
    disp, vel, acc = profile(displacement=10, start=0, time=test_time, ta=1.0)
    print(f"Final position: {disp[-1]:.2f}")
    print(f"Peak velocity: {max(vel):.2f}")
    print(f"Peak acceleration: {max(acc):.2f}")
    print()

    # Test motion function
    print("Testing motion function:")
    time_list, ta = motion(displacement=10, interval=0.1, accel_limit=5, velo_limit=3)
    print(f"Total time: {time_list[-1]:.2f} seconds")
    print(f"Acceleration time (ta): {ta:.2f} seconds")
    print()

    # Test joint_interpolation function
    print("Testing joint_interpolation function:")
    eom_a, eom_b, time_coord = joint_interpolation(
        displacement_a=10,
        start_a=0,
        displacement_b=5,
        start_b=0,
        interval=0.1,
        accel_limit_a=5,
        velo_limit_a=3,
        accel_limit_b=4,
        velo_limit_b=2,
    )
    print(f"Coordinated time: {time_coord[-1]:.2f} seconds")
    print(f"Joint A final position: {eom_a[0][-1]:.2f}")
    print(f"Joint B final position: {eom_b[0][-1]:.2f}")
