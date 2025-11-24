def profile(displacement, start, time, Ta):
    totalTime = time[-1]
    cruiseVelocity = displacement / (totalTime - Ta)
    accel = cruiseVelocity / Ta

    def calc(currentTime):
        if currentTime <= Ta:
            # Acceleration phase: d(t) = (a_m * t²) / 2
            return (start + 0.5 * accel * currentTime**2, accel * currentTime, accel)
        elif currentTime <= totalTime - Ta:
            # Constant velocity phase: d(t) = a_m * T_A * t - (a_m * T_A²) / 2
            return (
                start + accel * Ta * currentTime - 0.5 * accel * Ta**2,
                accel * Ta,  # This equals cruiseVelocity
                0,
            )
        else:
            # Deceleration phase: d(t) = -(a_m * (t - T)²) / 2 + a_m * T_A * (T - T_A)
            timeFromEnd = currentTime - totalTime
            return (
                start + (-0.5 * accel * timeFromEnd**2 + accel * Ta * (totalTime - Ta)),
                -accel * timeFromEnd,  # v(t) = -a_m(t - T)
                -accel,
            )

    results = [calc(ti) for ti in time]
    displacements = [x[0] for x in results]
    velocities = [x[1] for x in results]
    accelerations = [x[2] for x in results]

    return (displacements, velocities, accelerations)
