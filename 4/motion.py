def motion(displacement, interval, accelLimit, veloLimit):
    Ta = veloLimit / accelLimit

    if abs(displacement) <= veloLimit**2 / accelLimit:
        # Triangular profile
        Ta = (abs(displacement) / accelLimit) ** 0.5
        Tf = 2 * Ta
    else:
        # Trapezoidal profile
        Tf = abs(displacement) / veloLimit + Ta

    time = []
    currentTime = 0
    while currentTime <= Tf:
        time.append(currentTime)
        currentTime += interval
    if not time or time[-1] < Tf:
        time.append(Tf)

    return (time, Ta)
