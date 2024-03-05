class MotorPower:
    def __init__(self):
        self.m1 = 0.0
        self.m2 = 0.0
        self.m3 = 0.0
        self.m4 = 0.0


class ControlCommands:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.altitude = 0.0


class DesiredState:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.altitude = 0.0
        self.vx = 0.0
        self.vy = 0.0


class ActualState:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.altitude = 0.0
        self.vx = 0.0
        self.vy = 0.0


class GainsPID:
    def __init__(self):
        self.kp_att_rp = 0.0
        self.kd_att_rp = 0.0
        self.kp_att_y = 0.0
        self.kd_att_y = 0.0
        self.kp_vel_xy = 0.0
        self.kd_vel_xy = 0.0
        self.kp_z = 0.0
        self.kd_z = 0.0
        self.ki_z = 0.0


def constrain(value, minVal, maxVal):
    return min(maxVal, max(value, minVal))


pastAltitudeError = pastPitchError = pastRollError = pastYawRateError = 0.0
pastVxError = pastVyError = 0.0
altitudeIntegrator = 0.0


def init_pid_attitude_fixed_height_controller():
    global pastAltitudeError, pastYawRateError, pastPitchError, pastRollError, pastVxError, pastVyError, altitudeIntegrator
    pastAltitudeError = pastYawRateError = pastPitchError = pastRollError = pastVxError = pastVyError = 0.0
    altitudeIntegrator = 0.0


def pid_attitude_fixed_height_controller(actual_state, desired_state, gains_pid, dt, motorCommands):
    control_commands = ControlCommands()
    pid_fixed_height_controller(
        actual_state, desired_state, gains_pid, dt, control_commands)
    pid_attitude_controller(actual_state, desired_state,
                            gains_pid, dt, control_commands)
    motor_mixing(control_commands, motorCommands)


def pid_velocity_fixed_height_controller(actual_state, desired_state, gains_pid, dt, motorCommands):
    control_commands = ControlCommands()
    pid_horizontal_velocity_controller(
        actual_state, desired_state, gains_pid, dt)
    pid_fixed_height_controller(
        actual_state, desired_state, gains_pid, dt, control_commands)
    pid_attitude_controller(actual_state, desired_state,
                            gains_pid, dt, control_commands)
    motor_mixing(control_commands, motorCommands)


def pid_fixed_height_controller(actual_state, desired_state, gains_pid, dt, control_commands):
    global pastAltitudeError, altitudeIntegrator
    altitudeError = desired_state.altitude - actual_state.altitude
    altitudeDerivativeError = (altitudeError - pastAltitudeError) / dt

    altitudeIntegrator += altitudeError * dt
    control_commands.altitude = (
        gains_pid.kp_z * constrain(altitudeError, -1, 1) + gains_pid.kd_z * altitudeDerivativeError +
        gains_pid.ki_z * altitudeIntegrator + 48
    )
    pastAltitudeError = altitudeError


def motor_mixing(control_commands, motorCommands):
    motorCommands.m1 = control_commands.altitude - control_commands.roll + \
        control_commands.pitch + control_commands.yaw
    motorCommands.m2 = control_commands.altitude - control_commands.roll - \
        control_commands.pitch - control_commands.yaw
    motorCommands.m3 = control_commands.altitude + control_commands.roll - \
        control_commands.pitch + control_commands.yaw
    motorCommands.m4 = control_commands.altitude + control_commands.roll + \
        control_commands.pitch - control_commands.yaw


def pid_attitude_controller(actual_state, desired_state, gains_pid, dt, control_commands):
    global pastPitchError, pastRollError, pastYawRateError
    pitchError = desired_state.pitch - actual_state.pitch
    pitchDerivativeError = (pitchError - pastPitchError) / dt
    rollError = desired_state.roll - actual_state.roll
    rollDerivativeError = (rollError - pastRollError) / dt
    yawRateError = desired_state.yaw_rate - actual_state.yaw_rate

    control_commands.roll = gains_pid.kp_att_rp * \
        constrain(rollError, -1, 1) + gains_pid.kd_att_rp * rollDerivativeError
    control_commands.pitch = -gains_pid.kp_att_rp * \
        constrain(pitchError, -1, 1) - \
        gains_pid.kd_att_rp * pitchDerivativeError
    control_commands.yaw = gains_pid.kp_att_y * constrain(yawRateError, -1, 1)

    pastPitchError = pitchError
    pastRollError = rollError
    pastYawRateError = yawRateError


def pid_horizontal_velocity_controller(actual_state, desired_state, gains_pid, dt):
    global pastVxError, pastVyError
    vxError = desired_state.vx - actual_state.vx
    vxDerivative = (vxError - pastVxError) / dt
    vyError = desired_state.vy - actual_state.vy
    vyDerivative = (vyError - pastVyError) / dt

    pitchCommand = gains_pid.kp_vel_xy * \
        constrain(vxError, -1, 1) + gains_pid.kd_vel_xy * vxDerivative
    rollCommand = -gains_pid.kp_vel_xy * \
        constrain(vyError, -1, 1) - gains_pid.kd_vel_xy * vyDerivative

    desired_state.pitch = pitchCommand
    desired_state.roll = rollCommand

    pastVxError = vxError
    pastVyError = vyError
