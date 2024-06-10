from PyVecs import Vector3

def aim_boost_back(land_pos, target_pos, vel):
    land_dir = land_pos.normalize()

    target_correct_pos = target_pos - vel * Vector3(0, 1, 1) # drag correction factor
    target_dir = target_correct_pos.normalize()

    error = target_dir - land_dir

    target_dir = Vector3(0, error.y, error.z)

    return target_dir

def aim_coasting(land_pos, target_pos, vel):
    land_dir = land_pos.normalize()

    #target_correct_pos = target_pos + vel * Vector3(0, 1, 1) # drag correction factor
    target_correct_pos = target_pos + Vector3(0, 0.1*target_pos.y, 0.1*target_pos.z)
    target_dir = target_correct_pos.normalize()

    error = land_dir - target_dir

    return Vector3(0.1, error.y, error.z)

def aim_throttling(land_pos, target_pos, vel):
    land_dir = land_pos.normalize()

    #target_correct_pos = target_pos - Vector3(0, vel.y*5, vel.z*5) # drag correction factor
    if vel.x < -100:
        target_correct_pos = target_pos - Vector3(0, vel.y*5, vel.z*5) # drag correction factor
    else:
        k = max(0, vel.x / -100)
        target_correct_pos = target_pos - Vector3(0, vel.y*5*k, vel.z*5*k)

    target_dir = target_correct_pos.normalize()

    error = target_dir - land_dir

    return Vector3(2, error.y, error.z)

def aim_hovering(land_pos, target_pos, vel):
    land_dir = land_pos.normalize()

    target_dir = (target_pos - Vector3(0, vel.y*5, vel.z*5)).normalize()

    error = target_dir - land_dir

    return Vector3(5, error.y, error.z)

def aim_descent_in(land_pos, target_pos, vel):
    land_dir = land_pos.normalize()

    target_correct_pos = target_pos - Vector3(0, vel.y, vel.z) # drag correction factor
    target_dir = target_correct_pos.normalize()

    error = target_dir - land_dir

    return Vector3(2, error.y, error.z)

def aim_descent_out(vel):
    return Vector3(abs(vel.x)*5, -vel.y, -vel.z)