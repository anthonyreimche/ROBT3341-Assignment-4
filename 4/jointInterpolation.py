from motion import motion
from trajectory import profile


def jointInterpolation(
    displacementA,
    startA,
    displacementB,
    startB,
    interval,
    accelLimitA,
    veloLimitA,
    accelLimitB,
    veloLimitB,
):
    timeA, taA = motion(displacementA, interval, accelLimitA, veloLimitA)
    timeB, taB = motion(displacementB, interval, accelLimitB, veloLimitB)

    finalTime = max(timeA[-1], timeB[-1])

    # Create synchronized time array
    time = []
    currentTime = 0
    while currentTime <= finalTime:
        time.append(currentTime)
        currentTime += interval
    if not time or time[-1] < finalTime:
        time.append(finalTime)

    taA_synced = taA
    if abs(displacementA) > 0:
        if timeA[-1] < finalTime:
            discriminant = finalTime**2 - 4 * abs(displacementA) / accelLimitA
            if discriminant >= 0:
                taA_synced = (finalTime - discriminant**0.5) / 2
            else:
                taA_synced = finalTime / 2  # Fallback to triangular profile

    taB_synced = taB
    if abs(displacementB) > 0:
        if timeB[-1] < finalTime:
            discriminant = finalTime**2 - 4 * abs(displacementB) / accelLimitB
            if discriminant >= 0:
                taB_synced = (finalTime - discriminant**0.5) / 2
            else:
                taB_synced = finalTime / 2  # Fallback to triangular profile

    eomA = profile(displacementA, startA, time, taA_synced)
    eomB = profile(displacementB, startB, time, taB_synced)

    return (eomA, eomB, time)
