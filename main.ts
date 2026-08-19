//% color=#ff0011 icon="\uf1b9" block="Smart Motor" weight=100
//% groups='["Motor","Position","Robot","Readings","Gyroscope","Information"]'
namespace smartMotor {
    const I2C_ADDRESS = 0x66
    const I2C_QUERY_POLL_INTERVAL_MS = 2
    const I2C_QUERY_TIMEOUT_MS = 100
    const GYRO_RESET_CONFIRM_TIMEOUT_MS = 350
    const GYRO_RESET_CONFIRM_POLL_INTERVAL_MS = 10
    const GYRO_RESET_CONFIRM_TOLERANCE_X10 = 1
    const COMMAND_REGISTER_READ = 0x01
    const COMMAND_MOTOR_DATA_REFRESH = 0x02
    const COMMAND_VERSION = 0x10
    const COMMAND_GYRO_RESET = 0x11
    const COMMAND_SET_SPEED = 0x20
    const COMMAND_STOP = 0x21
    const COMMAND_MOVE = 0x22
    const COMMAND_MOVE_ABSOLUTE = 0x23
    const COMMAND_RESET_PHYSICAL = 0x24
    const COMMAND_ROBOT_SET_SPEED = 0x26
    const REGISTER_GYRO_ANGLE_START = 0x03
    const REGISTER_HOST_ACCELERATION_START = 0x0F
    const MOTOR_DATA_RECORD_LENGTH = 13
    const MOTOR_DATA_ANGLE_VALID = 0x01
    const MOTOR_DATA_SPEED_VALID = 0x02
    const MOTOR_DATA_RELATIVE_ANGLE_OFFSET = 1
    const MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET = 5
    const MOTOR_DATA_SPEED_OFFSET = 9
    const MOTOR_DATA_REFRESH_ANGLE = 0x01
    const MOTOR_DATA_REFRESH_SPEED = 0x02
    const ROBOT_INVALID_GYRO_ANGLE = 1000000000
    const ROBOT_DEFAULT_WHEEL_DIAMETER_MM = 62
    const MOTOR_WAIT_POLL_INTERVAL_MS = 20
    const MOTOR_WAIT_ANGLE_TOLERANCE_X10 = 20
    const MOTOR_WAIT_SPEED_TOLERANCE = 5
    const MOTOR_RESET_ESTIMATED_SPEED_PERCENT = 100
    const NEZHA_ESTIMATE_BUFFER_MS = 500
    const MOTION_TIMEOUT_GUARD_MS = 500
    const ROBOT_TURN_GYRO_UNCHANGED_TIMEOUT_MS = 2000

    /** Motor connector shown on the Power Smart Motor Hub. */
    export enum MotorPort {
        //% block="M5"
        M5 = 1,
        //% block="M6"
        M6 = 2,
        //% block="M7"
        M7 = 3,
        //% block="M8"
        M8 = 4
    }

    /** Motor rotation direction. */
    export enum MotorDirection {
        //% block="clockwise"
        Clockwise = 0,
        //% block="counterclockwise"
        Counterclockwise = 1
    }

    /** Unit used by the relative motor move block. */
    export enum MotorMoveUnit {
        //% block="circle"
        Circle = 1,
        //% block="degree"
        Degree = 2,
        //% block="second"
        Second = 3
    }

    /** Turn mode used by the absolute motor move block. */
    export enum MotorTurnMode {
        //% block="clockwise"
        Clockwise = 0,
        //% block="counterclockwise"
        Counterclockwise = 1,
        //% block="shortest path"
        ShortestPath = 2
    }

    /** Robot straight-drive direction. */
    export enum DriveDirection {
        //% block="forward"
        Forward = 0,
        //% block="backward"
        Backward = 1
    }

    /** Unit or mode used by the robot straight-drive block. */
    export enum DriveMode {
        //% block="millimeters"
        Millimeters = 0,
        //% block="seconds"
        Seconds = 1,
        //% block="degrees"
        Degrees = 2
    }

    /** Robot acceleration level. */
    export enum AccelLevel {
        //% block="slow"
        Slow = 0,
        //% block="medium"
        Medium = 1,
        //% block="fast"
        Fast = 2
    }

    /** Robot load level. */
    export enum RobotLoad {
        //% block="light"
        Light = 0,
        //% block="medium"
        Medium = 1,
        //% block="heavy"
        Heavy = 2
    }

    /** Whether a motor or robot command waits until the motion is complete. */
    export enum WaitMode {
        //% block="do not wait"
        NoWait = 0,
        //% block="wait until done"
        Wait = 1
    }

    /** Gyroscope attitude axis. */
    export enum GyroAxis {
        //% block="pitch"
        Pitch = 0,
        //% block="yaw"
        Yaw = 2,
        //% block="roll"
        Roll = 1
    }

    /** Whether the robot gyroscope angle is mirrored for steering control. */
    export enum GyroMirror {
        //% block="normal"
        Normal = 0,
        //% block="mirrored"
        Mirrored = 1
    }

    /** Host acceleration axis. */
    export enum HostAccelAxis {
        //% block="X"
        X = 0,
        //% block="Y"
        Y = 1,
        //% block="Z"
        Z = 2
    }

    let robotLeftMotor = MotorPort.M5
    let robotRightMotor = MotorPort.M6
    let robotWheelDiameterMm = ROBOT_DEFAULT_WHEEL_DIAMETER_MM
    let robotGyroAxis = GyroAxis.Yaw
    let robotGyroMirror = GyroMirror.Mirrored
    let robotLoad = RobotLoad.Medium
    let robotMotionId = 0
    let robotTurnActive = false
    let robotDriveActive = false
    let robotWorkerStarted = false
    let robotTurnMotionId = 0
    let robotTurnTargetYaw = 0
    let robotTurnCurrentSpeed = 0
    let robotTurnMaxSpeed = 0
    let robotTurnLastError = 0
    let robotTurnLastTime = 0
    let robotTurnAccel = AccelLevel.Medium
    let robotTurnLastYaw = ROBOT_INVALID_GYRO_ANGLE
    let robotTurnUnchangedMs = 0
    let robotDriveMotionId = 0
    let robotDriveMode = DriveMode.Seconds
    let robotDriveDirection = DriveDirection.Forward
    let robotDriveTargetValue = 0
    let robotDriveLastLocation = 0
    let robotDriveCurrentSpeed = 0
    let robotDriveMaxSpeed = 0
    let robotDriveLastError = 0
    let robotDriveIntegral = 0
    let robotDriveTargetYaw = 0
    let robotDriveLastTime = 0
    let robotDriveAccel = AccelLevel.Medium
    let robotDriveDeadlineMs = 0
    let lastQueryWasSuccessful = false
    let queryCacheKeys: string[] = []
    let queryCacheData: Buffer[] = []
    let motorMotionIds: number[] = [0, 0, 0, 0, 0]
    let gyroSpeedLastAngle: number[] = [0, 0, 0]
    let gyroSpeedLastTime: number[] = [0, 0, 0]
    let gyroSpeedHasSample: boolean[] = [false, false, false]

    function readI16Le(buffer: Buffer, offset: number): number {
        let value = buffer[offset] | (buffer[offset + 1] << 8)
        return value >= 0x8000 ? value - 0x10000 : value
    }

    function readI32Le(buffer: Buffer, offset: number): number {
        return (buffer[offset])
            | (buffer[offset + 1] << 8)
            | (buffer[offset + 2] << 16)
            | (buffer[offset + 3] << 24)
    }

    function clamp(value: number, minimum: number, maximum: number): number {
        return value < minimum ? minimum : value > maximum ? maximum : value
    }

    function copyBuffer(source: Buffer): Buffer {
        let result = pins.createBuffer(source.length)
        for (let index = 0; index < source.length; index++) {
            result[index] = source[index]
        }
        return result
    }

    function delayMs(ms: number): void {
        let endTime = input.runningTime() + ms
        while (endTime > input.runningTime()) {
        }
    }

    function queryCacheKey(command: number, commandData: number[], dataLength: number): string {
        let key = "" + command
        for (let index = 0; index < commandData.length; index++) {
            key += ":" + commandData[index]
        }
        return key + ":" + dataLength
    }

    function findQueryCache(key: string): number {
        for (let index = 0; index < queryCacheKeys.length; index++) {
            if (queryCacheKeys[index] == key) {
                return index
            }
        }
        return -1
    }

    function readQueryCache(key: string, dataLength: number): Buffer {
        let index = findQueryCache(key)
        return index >= 0 ? copyBuffer(queryCacheData[index]) : pins.createBuffer(dataLength)
    }

    function writeQueryCache(key: string, data: Buffer): void {
        let index = findQueryCache(key)
        if (index < 0) {
            queryCacheKeys.push(key)
            queryCacheData.push(copyBuffer(data))
        } else {
            queryCacheData[index] = copyBuffer(data)
        }
    }

    function i2cCommandSend(command: number, data: number[], delay: number = 1): void {
        let frame = pins.createBuffer(data.length + 4)
        frame[0] = 0xFF
        frame[1] = 0xF9
        frame[2] = command
        frame[3] = data.length
        for (let index = 0; index < data.length; index++) {
            frame[index + 4] = data[index]
        }
        pins.i2cWriteBuffer(I2C_ADDRESS, frame)
        delayMs(delay)
    }

    function i2cQueryRead(command: number, commandData: number[], dataLength: number,
        timeoutMs: number = 100): Buffer {
        let key = queryCacheKey(command, commandData, dataLength)
        let cachedData = readQueryCache(key, dataLength)
        lastQueryWasSuccessful = false
        let deadline = input.runningTime() + Math.max(0, timeoutMs)
        i2cCommandSend(command, commandData, 0)
        while (input.runningTime() < deadline) {
            let remainingMs = deadline - input.runningTime()
            delayMs(Math.min(I2C_QUERY_POLL_INTERVAL_MS, remainingMs))
            if (input.runningTime() >= deadline) {
                break
            }
            let reply = pins.i2cReadBuffer(I2C_ADDRESS, dataLength + 1)
            if (reply.length >= dataLength + 1 && reply[0] == 1) {
                let data = pins.createBuffer(dataLength)
                for (let index = 0; index < dataLength; index++) {
                    data[index] = reply[index + 1]
                }
                writeQueryCache(key, data)
                lastQueryWasSuccessful = true
                return data
            }
        }
        return cachedData
    }

    function readRegisters(startAddress: number, length: number,
        timeoutMs: number = 100): Buffer {
        let requestLength = clamp(Math.round(length), 1, 24)
        return i2cQueryRead(COMMAND_REGISTER_READ, [startAddress, requestLength], requestLength,
            timeoutMs)
    }

    function refreshMotorData(motor: MotorPort, dataMask: number): Buffer {
        return i2cQueryRead(COMMAND_MOTOR_DATA_REFRESH, [motor, dataMask], MOTOR_DATA_RECORD_LENGTH)
    }

    function readFreshGyroAngle(axis: GyroAxis): number {
        let data = readRegisters(REGISTER_GYRO_ANGLE_START + axis * 4, 4)
        return lastQueryWasSuccessful && data.length == 4
            ? readI32Le(data, 0) / 10
            : ROBOT_INVALID_GYRO_ANGLE
    }

    function gyroAngleIsValid(angle: number): boolean {
        return angle != ROBOT_INVALID_GYRO_ANGLE
    }

    function readRobotControlAngle(): number {
        let angle = readFreshGyroAngle(robotGyroAxis)
        if (!gyroAngleIsValid(angle)) {
            return ROBOT_INVALID_GYRO_ANGLE
        }
        return robotGyroMirror == GyroMirror.Mirrored ? -angle : angle
    }

    function refreshFreshMotorData(motor: MotorPort, dataMask: number): Buffer {
        let data = refreshMotorData(motor, dataMask)
        return lastQueryWasSuccessful ? data : pins.createBuffer(0)
    }

    function normalizeAngleX10(angleX10: number): number {
        let normalized = angleX10 % 3600
        return normalized < 0 ? normalized + 3600 : normalized
    }

    function circularAngleErrorX10(targetX10: number, currentX10: number): number {
        let error = normalizeAngleX10(targetX10) - normalizeAngleX10(currentX10)
        if (error > 1800) {
            error -= 3600
        } else if (error < -1800) {
            error += 3600
        }
        return error
    }

    function motorMask(motor: MotorPort): number {
        return (1 << (motor - 1)) & 0x0F
    }

    function signedSpeed(speed: number): number {
        return Math.round(clamp(speed, -100, 100))
    }

    function motorDirectionBit(direction: MotorDirection, speed: number): number {
        let reverse = signedSpeed(speed) < 0
        let counterclockwise = direction == MotorDirection.Counterclockwise
        return reverse != counterclockwise ? 1 : 0
    }

    function motorTurnModeByte(turnMode: MotorTurnMode, speed: number): number {
        if (turnMode == MotorTurnMode.ShortestPath) {
            return 2
        }
        let reverse = signedSpeed(speed) < 0
        let counterclockwise = turnMode == MotorTurnMode.Counterclockwise
        return reverse != counterclockwise ? 1 : 0
    }

    function nextMotorMotionId(motor: MotorPort): number {
        motorMotionIds[motor]++
        return motorMotionIds[motor]
    }

    function motorMotionIsCurrent(motor: MotorPort, motionId: number): boolean {
        return motorMotionIds[motor] == motionId
    }

    function robotMotorMask(): number {
        return motorMask(robotLeftMotor) | motorMask(robotRightMotor)
    }

    function cancelRobotMotion(stopActiveMotors: boolean = true): void {
        robotMotionId++
        let shouldStop = (robotTurnActive || robotDriveActive) && stopActiveMotors
        robotTurnActive = false
        robotDriveActive = false
        if (shouldStop) {
            i2cCommandSend(COMMAND_STOP, [robotMotorMask()])
        }
    }

    function sendRobotSpeed(leftSpeed: number, rightSpeed: number): void {
        let leftMotorSpeed = -Math.round(clamp(leftSpeed, -100, 100))
        let rightMotorSpeed = Math.round(clamp(rightSpeed, -100, 100))
        let direction = 0
        if (leftMotorSpeed < 0) {
            direction |= 0x01
        }
        if (rightMotorSpeed < 0) {
            direction |= 0x02
        }
        i2cCommandSend(COMMAND_ROBOT_SET_SPEED, [
            robotLeftMotor,
            robotRightMotor,
            Math.abs(leftMotorSpeed),
            Math.abs(rightMotorSpeed),
            direction
        ])
    }

    function mapRobotTurnMotorSpeed(speed: number): number {
        let deadZone = 8
        let magnitude = clamp(Math.abs(speed), 0, 100)
        let mapped = deadZone + magnitude * (100 - deadZone) / 100
        return speed < 0 ? -mapped : mapped
    }

    function truncateTowardZero(value: number): number {
        return value < 0 ? Math.ceil(value) : Math.floor(value)
    }

    function stopMotor(motor: MotorPort): void {
        i2cCommandSend(COMMAND_STOP, [motorMask(motor)])
    }

    function stopMotorIfCurrentMotion(motor: MotorPort, motionId: number): void {
        if (motorMotionIsCurrent(motor, motionId)) {
            stopMotor(motor)
        }
    }

    function motorRelativeValueX10(value: number, unit: MotorMoveUnit): number {
        let magnitude = Math.abs(value)
        if (unit == MotorMoveUnit.Circle) {
            return Math.round(magnitude * 3600)
        }
        if (unit == MotorMoveUnit.Degree) {
            magnitude = clamp(magnitude, 0, 360)
        }
        return Math.round(magnitude * 10)
    }

    function motorRelativeTargetDegrees(value: number, unit: MotorMoveUnit): number {
        if (unit == MotorMoveUnit.Circle) {
            return Math.abs(value) * 360
        }
        if (unit == MotorMoveUnit.Second) {
            return 0
        }
        return clamp(Math.abs(value), 0, 360)
    }

    function nezhaSpeedUnit(speed: number): number {
        return Math.max(Math.abs(speed) * 9, 1)
    }

    function nezhaEstimatedDegreeMs(degrees: number, speed: number): number {
        return Math.abs(degrees) * 1000 / nezhaSpeedUnit(speed) + NEZHA_ESTIMATE_BUFFER_MS
    }

    function nezhaEstimatedSecondMs(seconds: number): number {
        return Math.abs(seconds) * 1000
    }

    function motionTimeoutMs(estimatedMs: number): number {
        return Math.round(Math.max(estimatedMs, 0) + MOTION_TIMEOUT_GUARD_MS)
    }

    function accelerationGuardMs(speed: number, acceleration: number): number {
        return Math.round(Math.abs(speed) * 1000 / Math.max(acceleration, 1))
    }

    function motorRelativeTimeoutMs(value: number, speed: number, unit: MotorMoveUnit): number {
        if (unit == MotorMoveUnit.Second) {
            return motionTimeoutMs(nezhaEstimatedSecondMs(value))
        }
        let degrees = motorRelativeTargetDegrees(value, unit)
        return motionTimeoutMs(nezhaEstimatedDegreeMs(degrees, speed))
    }

    function motorResetTimeoutMs(motor: MotorPort): number {
        let degrees = 360
        let data = refreshFreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
        if (data.length == MOTOR_DATA_RECORD_LENGTH
            && (data[0] & MOTOR_DATA_ANGLE_VALID) != 0) {
            degrees = Math.abs(circularAngleErrorX10(0,
                readI32Le(data, MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET))) / 10
        }
        return motionTimeoutMs(nezhaEstimatedDegreeMs(degrees,
            MOTOR_RESET_ESTIMATED_SPEED_PERCENT))
    }

    function motorAbsoluteMoveDegrees(targetX10: number, currentX10: number,
        turnMode: MotorTurnMode, speed: number): number {
        if (turnMode == MotorTurnMode.ShortestPath) {
            return Math.abs(circularAngleErrorX10(targetX10, currentX10)) / 10
        }
        let current = normalizeAngleX10(currentX10)
        let target = normalizeAngleX10(targetX10)
        let effectiveCounterclockwise = motorTurnModeByte(turnMode, speed) == 1
        let deltaX10 = effectiveCounterclockwise
            ? normalizeAngleX10(current - target)
            : normalizeAngleX10(target - current)
        return deltaX10 == 0 ? 360 : deltaX10 / 10
    }

    function motorAbsoluteTimeoutMs(degrees: number, speed: number): number {
        return motionTimeoutMs(nezhaEstimatedDegreeMs(degrees, speed))
    }

    function robotDriveTimeoutMs(value: number, speed: number, mode: DriveMode,
        accel: AccelLevel): number {
        if (mode == DriveMode.Seconds) {
            return motionTimeoutMs(nezhaEstimatedSecondMs(value))
        }
        let wheelDegrees = value
        if (mode == DriveMode.Millimeters) {
            wheelDegrees = value * 360 / (robotWheelDiameterMm * Math.PI)
        }
        let effectiveSpeed = speed * 0.9
        return motionTimeoutMs(nezhaEstimatedDegreeMs(wheelDegrees, effectiveSpeed)
            + accelerationGuardMs(effectiveSpeed, driveAccelerationForLevel(accel)))
    }

    function sendMotorRelativeStep(motor: MotorPort, value: number, speed: number,
        direction: MotorDirection, unit: MotorMoveUnit): void {
        if (value <= 0 || speed == 0) {
            return
        }
        let valueX10 = motorRelativeValueX10(value, unit)
        let speedPercent = Math.abs(signedSpeed(speed))
        i2cCommandSend(COMMAND_MOVE, [
            motor,
            unit,
            (valueX10 >> 24) & 0xFF,
            (valueX10 >> 16) & 0xFF,
            (valueX10 >> 8) & 0xFF,
            valueX10 & 0xFF,
            speedPercent,
            motorDirectionBit(direction, speed)
        ])
    }

    function waitForMotorReset(motor: MotorPort, motionId: number, timeoutMs: number): void {
        let deadline = input.runningTime() + timeoutMs
        while (motorMotionIsCurrent(motor, motionId) && input.runningTime() < deadline) {
            let data = refreshFreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE | MOTOR_DATA_REFRESH_SPEED)
            if (data.length == MOTOR_DATA_RECORD_LENGTH
                && (data[0] & MOTOR_DATA_ANGLE_VALID) != 0) {
                let relativeX10 = readI32Le(data, MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
                let absoluteX10 = circularAngleErrorX10(0, readI32Le(data, MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET))
                let speedOk = (data[0] & MOTOR_DATA_SPEED_VALID) == 0
                    || Math.abs(readI16Le(data, MOTOR_DATA_SPEED_OFFSET)) <= MOTOR_WAIT_SPEED_TOLERANCE
                if (Math.abs(relativeX10) <= MOTOR_WAIT_ANGLE_TOLERANCE_X10
                    && Math.abs(absoluteX10) <= MOTOR_WAIT_ANGLE_TOLERANCE_X10
                    && speedOk) {
                    return
                }
            }
            basic.pause(MOTOR_WAIT_POLL_INTERVAL_MS)
        }
        stopMotorIfCurrentMotion(motor, motionId)
    }

    function waitForMotorRelativeMove(motor: MotorPort, startRelativeX10: number,
        hasStart: boolean, targetDegrees: number, directionSign: number,
        motionId: number, timeoutMs: number): void {
        let deadline = input.runningTime() + timeoutMs
        while (motorMotionIsCurrent(motor, motionId) && input.runningTime() < deadline) {
            let data = refreshFreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE | MOTOR_DATA_REFRESH_SPEED)
            if (data.length == MOTOR_DATA_RECORD_LENGTH
                && (data[0] & MOTOR_DATA_ANGLE_VALID) != 0) {
                let currentRelativeX10 = readI32Le(data, MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
                if (!hasStart) {
                    startRelativeX10 = currentRelativeX10
                    hasStart = true
                }
                let traveledX10 = (currentRelativeX10 - startRelativeX10) * directionSign
                let speedOk = (data[0] & MOTOR_DATA_SPEED_VALID) != 0
                    && Math.abs(readI16Le(data, MOTOR_DATA_SPEED_OFFSET)) <= MOTOR_WAIT_SPEED_TOLERANCE
                if (traveledX10 >= targetDegrees * 10 - MOTOR_WAIT_ANGLE_TOLERANCE_X10
                    && speedOk) {
                    return
                }
            }
            basic.pause(MOTOR_WAIT_POLL_INTERVAL_MS)
        }
        stopMotorIfCurrentMotion(motor, motionId)
    }

    function waitForMotorTimedMove(motor: MotorPort, runMs: number,
        motionId: number, timeoutMs: number): void {
        let start = input.runningTime()
        let deadline = start + timeoutMs
        while (motorMotionIsCurrent(motor, motionId) && input.runningTime() < deadline) {
            let elapsed = input.runningTime() - start
            if (elapsed >= runMs) {
                let data = refreshFreshMotorData(motor, MOTOR_DATA_REFRESH_SPEED)
                if (data.length == MOTOR_DATA_RECORD_LENGTH
                    && (data[0] & MOTOR_DATA_SPEED_VALID) != 0
                    && Math.abs(readI16Le(data, MOTOR_DATA_SPEED_OFFSET)) <= MOTOR_WAIT_SPEED_TOLERANCE) {
                    return
                }
            }
            basic.pause(MOTOR_WAIT_POLL_INTERVAL_MS)
        }
        stopMotorIfCurrentMotion(motor, motionId)
    }

    function waitForMotorAbsoluteMove(motor: MotorPort, targetX10: number,
        requireMovement: boolean, motionId: number, timeoutMs: number): void {
        let deadline = input.runningTime() + timeoutMs
        let movedAway = !requireMovement
        while (motorMotionIsCurrent(motor, motionId) && input.runningTime() < deadline) {
            let data = refreshFreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE | MOTOR_DATA_REFRESH_SPEED)
            if (data.length == MOTOR_DATA_RECORD_LENGTH
                && (data[0] & MOTOR_DATA_ANGLE_VALID) != 0) {
                let errorX10 = circularAngleErrorX10(targetX10,
                    readI32Le(data, MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET))
                if (Math.abs(errorX10) > MOTOR_WAIT_ANGLE_TOLERANCE_X10) {
                    movedAway = true
                }
                let speedOk = (data[0] & MOTOR_DATA_SPEED_VALID) == 0
                    || Math.abs(readI16Le(data, MOTOR_DATA_SPEED_OFFSET)) <= MOTOR_WAIT_SPEED_TOLERANCE
                if (movedAway && Math.abs(errorX10) <= MOTOR_WAIT_ANGLE_TOLERANCE_X10 && speedOk) {
                    return
                }
            }
            basic.pause(MOTOR_WAIT_POLL_INTERVAL_MS)
        }
        stopMotorIfCurrentMotion(motor, motionId)
    }

    function turnAccelerationForLevel(accel: AccelLevel): number {
        if (accel == AccelLevel.Slow) {
            return 25
        }
        if (accel == AccelLevel.Fast) {
            return 300
        }
        return 60
    }

    function driveAccelerationForLevel(accel: AccelLevel): number {
        if (accel == AccelLevel.Slow) {
            return 20
        }
        if (accel == AccelLevel.Fast) {
            return 80
        }
        return 50
    }

    function robotStopIfCurrentMotion(motionId: number): void {
        if (motionId == robotMotionId) {
            i2cCommandSend(COMMAND_STOP, [robotMotorMask()])
            robotTurnActive = false
            robotDriveActive = false
        }
    }

    function updateRobotTurn(): void {
        if (!robotTurnActive) {
            return
        }
        let motionId = robotTurnMotionId
        if (motionId != robotMotionId) {
            return
        }

        let now = input.runningTime()
        let elapsed = now - robotTurnLastTime
        if (elapsed <= 0) {
            return
        }
        robotTurnLastTime = now

        let currentYaw = readRobotControlAngle()
        if (!gyroAngleIsValid(currentYaw)) {
            robotStopIfCurrentMotion(motionId)
            return
        }
        if (currentYaw == robotTurnLastYaw) {
            robotTurnUnchangedMs += elapsed
            if (robotTurnUnchangedMs >= ROBOT_TURN_GYRO_UNCHANGED_TIMEOUT_MS) {
                robotStopIfCurrentMotion(motionId)
                return
            }
        } else {
            robotTurnLastYaw = currentYaw
            robotTurnUnchangedMs = 0
        }
        let error = robotTurnTargetYaw - currentYaw
        let crossedTarget = (robotTurnLastError > 0 && error <= 0)
            || (robotTurnLastError < 0 && error >= 0)
        let stopTolerance = 1 + Math.max(0, robotTurnMaxSpeed - 50) / 50
        if (Math.abs(error) <= stopTolerance || crossedTarget) {
            robotStopIfCurrentMotion(motionId)
            return
        }

        let dt = elapsed / 1000
        let derivative = dt <= 0.2 ? (error - robotTurnLastError) / dt : 0
        robotTurnLastError = error

        let brakingAngle = 30
        let allowedSpeed = robotTurnMaxSpeed
        if (Math.abs(error) < brakingAngle) {
            allowedSpeed = robotTurnMaxSpeed * Math.abs(error) / brakingAngle
        }

        let acceleration = turnAccelerationForLevel(robotTurnAccel)
        if (robotTurnCurrentSpeed < allowedSpeed) {
            robotTurnCurrentSpeed = Math.min(allowedSpeed,
                robotTurnCurrentSpeed + acceleration * dt)
        } else if (robotTurnCurrentSpeed > allowedSpeed) {
            robotTurnCurrentSpeed = Math.max(allowedSpeed,
                robotTurnCurrentSpeed - acceleration * 2 * dt)
        }

        let output = clamp(0.35 * error + 0.015 * derivative,
            -robotTurnCurrentSpeed, robotTurnCurrentSpeed)
        if (output * error <= 0) {
            output = error > 0 ? 0.01 : -0.01
        }
        output = mapRobotTurnMotorSpeed(output)
        output = truncateTowardZero(output)
        if (motionId != robotMotionId || !robotTurnActive) {
            return
        }
        sendRobotSpeed(output, -output)
    }

    function updateRobotDriveStraight(): void {
        if (!robotDriveActive) {
            return
        }
        let motionId = robotDriveMotionId
        if (motionId != robotMotionId) {
            return
        }

        let now = input.runningTime()
        if (robotDriveDeadlineMs > 0 && now >= robotDriveDeadlineMs) {
            robotStopIfCurrentMotion(motionId)
            return
        }
        let elapsed = now - robotDriveLastTime
        if (elapsed <= 0) {
            return
        }
        robotDriveLastTime = now

        let allowedSpeed = robotDriveMaxSpeed
        let brakingRatio = Math.max(1, robotDriveMaxSpeed / 45)
        if (robotDriveMode == DriveMode.Seconds) {
            robotDriveTargetValue -= elapsed
            if (robotDriveTargetValue <= 0) {
                robotStopIfCurrentMotion(motionId)
                return
            }
            let brakingTime = 1000 * brakingRatio
            if (robotDriveTargetValue < brakingTime) {
                allowedSpeed *= robotDriveTargetValue / brakingTime
            }
        } else {
            let rightData = refreshFreshMotorData(robotRightMotor, MOTOR_DATA_REFRESH_ANGLE)
            if (rightData.length != MOTOR_DATA_RECORD_LENGTH
                || (rightData[0] & MOTOR_DATA_ANGLE_VALID) == 0) {
                robotStopIfCurrentMotion(motionId)
                return
            }
            let location = readI32Le(rightData, MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
            let traveled = location - robotDriveLastLocation
            if ((robotDriveTargetValue > 0 && traveled >= robotDriveTargetValue)
                || (robotDriveTargetValue < 0 && traveled <= robotDriveTargetValue)) {
                robotStopIfCurrentMotion(motionId)
                return
            }
            robotDriveLastLocation = location
            robotDriveTargetValue -= traveled

            let brakingDistanceMm = 100 * brakingRatio * brakingRatio
            let brakingDistance = brakingDistanceMm * 3600
                / (robotWheelDiameterMm * Math.PI)
            if (Math.abs(robotDriveTargetValue) < brakingDistance) {
                allowedSpeed *= Math.abs(robotDriveTargetValue) / brakingDistance
            }
        }

        let dt = elapsed / 1000
        if (robotDriveCurrentSpeed < allowedSpeed) {
            robotDriveCurrentSpeed = Math.min(allowedSpeed,
                robotDriveCurrentSpeed + driveAccelerationForLevel(robotDriveAccel) * dt)
        } else if (robotDriveCurrentSpeed > allowedSpeed) {
            robotDriveCurrentSpeed = Math.max(allowedSpeed,
                robotDriveCurrentSpeed - 60 * dt)
        }
        if (Math.abs(robotDriveCurrentSpeed) < 8) {
            robotDriveCurrentSpeed = 8
        }

        let currentYaw = readRobotControlAngle()
        if (!gyroAngleIsValid(currentYaw)) {
            robotStopIfCurrentMotion(motionId)
            return
        }
        let error = robotDriveTargetYaw - currentYaw
        robotDriveIntegral = clamp(robotDriveIntegral + error * dt, -40, 40)
        let derivative = (error - robotDriveLastError) / dt
        robotDriveLastError = error
        let output = clamp(3 * error + 0.2 * robotDriveIntegral + 0.1 * derivative,
            -10, 10)
        let baseSpeed = robotDriveDirection == DriveDirection.Backward
            ? -robotDriveCurrentSpeed : robotDriveCurrentSpeed
        if (motionId != robotMotionId || !robotDriveActive) {
            return
        }
        sendRobotSpeed(baseSpeed + output, baseSpeed - output)
    }

    function startRobotWorker(): void {
        if (robotWorkerStarted) {
            return
        }
        robotWorkerStarted = true
        control.inBackground(function () {
            while (true) {
                updateRobotTurn()
                updateRobotDriveStraight()
                basic.pause(9)
            }
        })
    }

    //% group="Motor"
    //% blockId=smartmotor_motor_start block="motor $motor direction $direction speed $speed start"
    //% motor.defl=smartMotor.MotorPort.M5
    //% speed.min=-100 speed.max=100 speed.defl=50
    //% direction.defl=smartMotor.MotorDirection.Clockwise
    //% weight=100
    /**
     * Start a motor with the selected direction and signed speed.
     * @param motor motor port M5-M8
     * @param direction clockwise or counterclockwise direction
     * @param speed speed from -100 to 100
     */
    export function motorStart(motor: MotorPort, direction: MotorDirection, speed: number): void {
        cancelRobotMotion()
        nextMotorMotionId(motor)
        let speedPercent = Math.abs(signedSpeed(speed))
        i2cCommandSend(COMMAND_SET_SPEED, [motor, speedPercent, motorDirectionBit(direction, speed)])
    }

    //% group="Motor"
    //% blockId=smartmotor_motor_stop block="motor $motor stop"
    //% motor.defl=smartMotor.MotorPort.M5
    //% weight=99
    /**
     * Stop one motor.
     * @param motor motor port M5-M8
     */
    export function motorStop(motor: MotorPort): void {
        cancelRobotMotion()
        nextMotorMotionId(motor)
        i2cCommandSend(COMMAND_STOP, [motorMask(motor)])
    }

    //% group="Position"
    //% blockId=smartmotor_motor_reset block="motor $motor reset position || $waitMode"
    //% motor.defl=smartMotor.MotorPort.M5
    //% waitMode.defl=smartMotor.WaitMode.Wait
    //% expandableArgumentMode="toggle"
    //% weight=90
    /**
     * Reset the current motor position to zero.
     * @param motor motor port M5-M8
     * @param waitMode wait for completion or return after starting the motion
     */
    export function motorReset(motor: MotorPort, waitMode: WaitMode = WaitMode.Wait): void {
        cancelRobotMotion()
        let motionId = nextMotorMotionId(motor)
        let timeoutMs = waitMode == WaitMode.Wait ? motorResetTimeoutMs(motor) : 0
        i2cCommandSend(COMMAND_RESET_PHYSICAL, [motorMask(motor)])
        if (waitMode == WaitMode.Wait) {
            waitForMotorReset(motor, motionId, timeoutMs)
        }
    }

    //% group="Position"
    //% blockId=smartmotor_motor_move_absolute block="motor $motor speed $speed $turnMode rotate to absolute angle $angle || $waitMode"
    //% motor.defl=smartMotor.MotorPort.M5
    //% angle.min=0 angle.max=360 angle.defl=90
    //% speed.min=-100 speed.max=100 speed.defl=50
    //% turnMode.defl=smartMotor.MotorTurnMode.ShortestPath
    //% waitMode.defl=smartMotor.WaitMode.Wait
    //% inlineInputMode=inline
    //% expandableArgumentMode="toggle"
    //% weight=89
    /**
     * Rotate a motor to an absolute angle.
     * @param motor motor port M5-M8
     * @param angle target angle in degrees, 0 to 360
     * @param speed speed from -100 to 100
     * @param turnMode clockwise, counterclockwise, or shortest path
     * @param waitMode wait for completion or return after starting the motion
     */
    export function motorMoveAbsolute(motor: MotorPort, angle: number, speed: number,
        turnMode: MotorTurnMode = MotorTurnMode.ShortestPath,
        waitMode: WaitMode = WaitMode.Wait): void {
        if (signedSpeed(speed) == 0) {
            return
        }
        cancelRobotMotion()
        let motionId = nextMotorMotionId(motor)
        let normalized = normalizeAngleX10(Math.round(angle * 10))
        let timeoutDegrees = 360
        if (waitMode == WaitMode.Wait) {
            let data = refreshFreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
            if (data.length == MOTOR_DATA_RECORD_LENGTH
                && (data[0] & MOTOR_DATA_ANGLE_VALID) != 0) {
                timeoutDegrees = motorAbsoluteMoveDegrees(normalized,
                    readI32Le(data, MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET), turnMode, speed)
            }
        }
        let speedPercent = Math.abs(signedSpeed(speed))
        i2cCommandSend(COMMAND_MOVE_ABSOLUTE, [
            motor,
            (normalized >> 8) & 0xFF,
            normalized & 0xFF,
            speedPercent,
            motorTurnModeByte(turnMode, speed)
        ])
        if (waitMode == WaitMode.Wait) {
            waitForMotorAbsoluteMove(motor, normalized,
                turnMode != MotorTurnMode.ShortestPath, motionId,
                motorAbsoluteTimeoutMs(timeoutDegrees, speed))
        }
    }

    //% group="Position"
    //% blockId=smartmotor_motor_move_relative block="motor $motor speed $speed direction $direction run $angle $unit || $waitMode"
    //% motor.defl=smartMotor.MotorPort.M5
    //% angle.min=0 angle.max=10000 angle.defl=90
    //% speed.min=-100 speed.max=100 speed.defl=50
    //% direction.defl=smartMotor.MotorDirection.Clockwise
    //% unit.defl=smartMotor.MotorMoveUnit.Degree
    //% waitMode.defl=smartMotor.WaitMode.Wait
    //% inlineInputMode=inline
    //% expandableArgumentMode="toggle"
    //% weight=88
    /**
     * Rotate a motor by a relative angle.
     * @param motor motor port M5-M8
     * @param angle relative value, interpreted by the selected unit
     * @param speed speed from -100 to 100
     * @param direction clockwise or counterclockwise direction
     * @param unit circle, degree, or second
     * @param waitMode wait for completion or return after starting the motion
     */
    export function motorMoveRelative(motor: MotorPort, angle: number, speed: number,
        direction: MotorDirection = MotorDirection.Clockwise,
        unit: MotorMoveUnit = MotorMoveUnit.Degree,
        waitMode: WaitMode = WaitMode.Wait): void {
        let moveValue = angle
        if (moveValue <= 0 || signedSpeed(speed) == 0) {
            return
        }
        cancelRobotMotion()
        let motionId = nextMotorMotionId(motor)
        let startRelativeX10 = 0
        let hasStart = false
        if (waitMode == WaitMode.Wait && unit != MotorMoveUnit.Second) {
            let data = refreshFreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
            if (data.length == MOTOR_DATA_RECORD_LENGTH
                && (data[0] & MOTOR_DATA_ANGLE_VALID) != 0) {
                startRelativeX10 = readI32Le(data, MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
                hasStart = true
            }
        }
        sendMotorRelativeStep(motor, moveValue, speed, direction, unit)
        if (waitMode == WaitMode.Wait) {
            let timeoutMs = motorRelativeTimeoutMs(moveValue, speed, unit)
            if (unit == MotorMoveUnit.Second) {
                waitForMotorTimedMove(motor, Math.round(moveValue * 1000), motionId, timeoutMs)
            } else {
                let directionSign = motorDirectionBit(direction, speed) == 0 ? 1 : -1
                waitForMotorRelativeMove(motor, startRelativeX10, hasStart,
                    motorRelativeTargetDegrees(moveValue, unit), directionSign, motionId, timeoutMs)
            }
        }
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_set_wheel_diameter block="robot wheel diameter $diameter mm"
    //% diameter.min=0 diameter.max=10000 diameter.defl=62
    //% weight=80
    /**
     * Set the robot wheel diameter.
     * @param diameter wheel diameter in millimeters, 0 to 10000
     */
    export function robotSetWheelDiameter(diameter: number): void {
        robotWheelDiameterMm = Math.round(clamp(diameter, 0, 10000))
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_set_motors block="robot set left wheel $leftMotor and set right wheel $rightMotor"
    //% leftMotor.defl=smartMotor.MotorPort.M5
    //% rightMotor.defl=smartMotor.MotorPort.M6
    //% weight=79
    /**
     * Select the motors used as the robot left and right wheels.
     * @param leftMotor left wheel motor port M5-M8
     * @param rightMotor right wheel motor port M5-M8
     */
    export function robotSetMotors(leftMotor: MotorPort, rightMotor: MotorPort): void {
        if (leftMotor != rightMotor) {
            cancelRobotMotion()
            robotLeftMotor = leftMotor
            robotRightMotor = rightMotor
        }
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_set_load block="robot set load $load"
    //% load.defl=smartMotor.RobotLoad.Medium
    //% weight=78
    /**
     * Set the robot load level.
     * @param load light, medium, or heavy load
     */
    export function robotSetLoad(load: RobotLoad): void {
        robotLoad = load
    }

    /**
     * Select the gyroscope axis and direction used by robot turn and straight-drive correction.
     * @param axis pitch, yaw, or roll axis
     * @param mirror normal or mirrored direction
     */
    export function robotSetGyro(axis: GyroAxis, mirror: GyroMirror): void {
        cancelRobotMotion()
        robotGyroAxis = axis
        robotGyroMirror = mirror
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_run block="robot run left wheel $leftSpeed % speed right wheel $rightSpeed % speed"
    //% leftSpeed.min=-100 leftSpeed.max=100 leftSpeed.defl=50
    //% rightSpeed.min=-100 rightSpeed.max=100 rightSpeed.defl=50
    //% inlineInputMode=inline
    //% weight=76
    /**
     * Run the selected robot left and right wheels directly without PID correction.
     * @param leftSpeed left wheel speed from -100 to 100
     * @param rightSpeed right wheel speed from -100 to 100
     */
    export function robotRun(leftSpeed: number, rightSpeed: number): void {
        cancelRobotMotion()
        sendRobotSpeed(clamp(leftSpeed, -100, 100), clamp(rightSpeed, -100, 100))
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_turn block="robot turn $angle degrees speed $speed acceleration $accel || $waitMode"
    //% angle.min=-360 angle.max=360 angle.defl=90
    //% speed.min=0 speed.max=100 speed.defl=50
    //% accel.defl=smartMotor.AccelLevel.Medium
    //% waitMode.defl=smartMotor.WaitMode.Wait
    //% inlineInputMode=inline
    //% expandableArgumentMode="toggle"
    //% weight=75
    /**
     * Turn the robot in place using gyroscope feedback.
     * @param angle turn angle in degrees, -360 to 360
     * @param speed speed from 0 to 100
     * @param accel acceleration level used by the turn speed ramp
     * @param waitMode wait for completion or return after starting the background motion
     */
    export function robotTurn(angle: number, speed: number, accel: AccelLevel,
        waitMode: WaitMode = WaitMode.Wait): void {
        cancelRobotMotion()
        let turnAngle = clamp(angle, -360, 360)
        let turnSpeed = clamp(speed, 0, 100)
        if (turnAngle == 0 || turnSpeed <= 0) {
            return
        }

        let motionId = robotMotionId
        let currentYaw = readRobotControlAngle()
        if (!gyroAngleIsValid(currentYaw)) {
            return
        }
        robotTurnMotionId = motionId
        robotTurnTargetYaw = currentYaw + turnAngle
        robotTurnCurrentSpeed = 8
        robotTurnMaxSpeed = Math.abs(turnSpeed)
        robotTurnLastError = turnAngle
        robotTurnLastTime = input.runningTime()
        robotTurnAccel = accel
        robotTurnLastYaw = currentYaw
        robotTurnUnchangedMs = 0
        robotTurnActive = true
        startRobotWorker()

        if (waitMode == WaitMode.Wait) {
            while (robotTurnActive && motionId == robotMotionId) {
                basic.pause(10)
            }
        }
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_drive_straight block="robot drive $direction $value $mode speed $speed acceleration $accel || $waitMode"
    //% direction.defl=smartMotor.DriveDirection.Forward
    //% value.min=0 value.max=10000 value.defl=100
    //% mode.defl=smartMotor.DriveMode.Millimeters
    //% speed.min=0 speed.max=100 speed.defl=50
    //% accel.defl=smartMotor.AccelLevel.Medium
    //% waitMode.defl=smartMotor.WaitMode.Wait
    //% inlineInputMode=inline
    //% expandableArgumentMode="toggle"
    //% weight=74
    /**
     * Drive the robot straight using a distance, time, or wheel-angle value.
     * @param direction forward or backward
     * @param value distance in millimeters, time in seconds, or wheel-angle value in degrees
     * @param mode millimeters, seconds, or wheel degrees
     * @param speed maximum straight-drive speed from 0 to 100
     * @param accel acceleration level used by the straight-drive speed ramp
     * @param waitMode wait for completion or return after starting the background motion
     */
    export function robotDriveStraight(direction: DriveDirection, value: number, mode: DriveMode,
        speed: number, accel: AccelLevel, waitMode: WaitMode = WaitMode.Wait): void {
        cancelRobotMotion()
        if (mode == DriveMode.Millimeters && robotWheelDiameterMm <= 0) {
            return
        }
        let driveValue = Math.round(clamp(value, 0, 10000))
        let driveSpeed = clamp(speed, 0, 100)
        if (driveValue <= 0 || driveSpeed <= 0) {
            return
        }

        let motionId = robotMotionId
        robotDriveMotionId = motionId
        robotDriveMode = mode
        robotDriveDirection = direction
        robotDriveTargetValue = driveValue * 10
        if (mode == DriveMode.Seconds) {
            robotDriveTargetValue = driveValue * 1000
        } else {
            let rightData = refreshFreshMotorData(robotRightMotor, MOTOR_DATA_REFRESH_ANGLE)
            if (rightData.length != MOTOR_DATA_RECORD_LENGTH
                || (rightData[0] & MOTOR_DATA_ANGLE_VALID) == 0) {
                return
            }
            robotDriveLastLocation = readI32Le(rightData, MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
            if (mode == DriveMode.Millimeters) {
                robotDriveTargetValue = Math.round(driveValue * 3600
                    / (robotWheelDiameterMm * Math.PI))
            }
            if (direction == DriveDirection.Backward) {
                robotDriveTargetValue = -robotDriveTargetValue
            }
        }

        robotDriveCurrentSpeed = 8
        robotDriveMaxSpeed = driveSpeed * 0.9
        robotDriveLastError = 0
        robotDriveIntegral = 0
        let currentYaw = readRobotControlAngle()
        if (!gyroAngleIsValid(currentYaw)) {
            return
        }
        robotDriveTargetYaw = currentYaw
        robotDriveLastTime = input.runningTime()
        robotDriveAccel = accel
        robotDriveDeadlineMs = robotDriveLastTime + robotDriveTimeoutMs(driveValue, driveSpeed, mode, accel)
        robotDriveActive = true
        startRobotWorker()

        if (waitMode == WaitMode.Wait) {
            while (robotDriveActive && motionId == robotMotionId) {
                basic.pause(10)
            }
        }
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_stop block="robot stop"
    //% weight=74
    /**
     * Stop robot motion.
     */
    export function robotStop(): void {
        cancelRobotMotion(false)
        i2cCommandSend(COMMAND_STOP, [robotMotorMask()])
    }

    /**
     * Check whether the robot has no active turn or straight-drive motion.
     */
    export function robotIsIdle(): boolean {
        return !robotTurnActive && !robotDriveActive
    }

    //% group="Readings"
    //% blockId=smartmotor_motor_speed block="motor $motor current speed (degrees/s)"
    //% motor.defl=smartMotor.MotorPort.M5
    //% weight=70
    /**
     * Read the current motor speed.
     * @param motor motor port M5-M8
     */
    export function motorGetSpeed(motor: MotorPort): number {
        let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_SPEED)
        if (motorData.length != MOTOR_DATA_RECORD_LENGTH
            || (motorData[0] & MOTOR_DATA_SPEED_VALID) == 0) {
            return 0
        }
        return readI16Le(motorData, MOTOR_DATA_SPEED_OFFSET)
    }

    //% group="Readings"
    //% blockId=smartmotor_motor_absolute_angle block="motor $motor absolute angle"
    //% motor.defl=smartMotor.MotorPort.M5
    //% weight=69
    /**
     * Read the current motor absolute angle in degrees.
     * @param motor motor port M5-M8
     */
    export function motorGetAbsoluteAngle(motor: MotorPort): number {
        let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
        if (motorData.length != MOTOR_DATA_RECORD_LENGTH
            || (motorData[0] & MOTOR_DATA_ANGLE_VALID) == 0) {
            return 0
        }
        return normalizeAngleX10(readI32Le(motorData, MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET)) / 10
    }

    //% group="Readings"
    //% blockId=smartmotor_motor_relative_angle block="motor $motor relative angle"
    //% motor.defl=smartMotor.MotorPort.M5
    //% weight=68
    /**
     * Read the current motor relative angle in degrees.
     * @param motor motor port M5-M8
     */
    export function motorGetRelativeAngle(motor: MotorPort): number {
        let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
        if (motorData.length != MOTOR_DATA_RECORD_LENGTH
            || (motorData[0] & MOTOR_DATA_ANGLE_VALID) == 0) {
            return 0
        }
        return readI32Le(motorData, MOTOR_DATA_RELATIVE_ANGLE_OFFSET) / 10
    }

    //% group="Gyroscope"
    //% blockId=smartmotor_gyro_reset block="host position reset"
    //% weight=60
    /**
     * Reset the gyroscope attitude angles.
     */
    export function resetGyroAngle(): void {
        i2cCommandSend(COMMAND_GYRO_RESET, [])
        gyroSpeedHasSample = [false, false, false]
        let deadline = input.runningTime() + GYRO_RESET_CONFIRM_TIMEOUT_MS
        while (input.runningTime() < deadline) {
            let data = readRegisters(REGISTER_GYRO_ANGLE_START, 12,
                deadline - input.runningTime())
            if (lastQueryWasSuccessful
                && Math.abs(readI32Le(data, 0)) <= GYRO_RESET_CONFIRM_TOLERANCE_X10
                && Math.abs(readI32Le(data, 4)) <= GYRO_RESET_CONFIRM_TOLERANCE_X10
                && Math.abs(readI32Le(data, 8)) <= GYRO_RESET_CONFIRM_TOLERANCE_X10) {
                return
            }
            let remainingMs = deadline - input.runningTime()
            if (remainingMs > 0) {
                delayMs(Math.min(GYRO_RESET_CONFIRM_POLL_INTERVAL_MS, remainingMs))
            }
        }
    }

    /**
     * Read the gyroscope angular speed in degrees per second.
     * @param axis pitch, yaw, or roll axis
     */
    export function readGyroAngularSpeed(axis: GyroAxis): number {
        let now = input.runningTime()
        let angle = readGyroAngle(axis)
        let index = axis
        if (!gyroSpeedHasSample[index]) {
            gyroSpeedLastAngle[index] = angle
            gyroSpeedLastTime[index] = now
            gyroSpeedHasSample[index] = true
            return 0
        }
        let elapsedMs = now - gyroSpeedLastTime[index]
        if (elapsedMs <= 0) {
            return 0
        }
        let angularSpeed = (angle - gyroSpeedLastAngle[index]) * 1000 / elapsedMs
        gyroSpeedLastAngle[index] = angle
        gyroSpeedLastTime[index] = now
        return angularSpeed
    }

    //% group="Gyroscope"
    //% blockId=smartmotor_host_acceleration block="host $axis acceleration (mg)"
    //% axis.defl=smartMotor.HostAccelAxis.X
    //% weight=59
    /**
     * Read the host acceleration on the selected X, Y, or Z axis in mg.
     * @param axis X, Y, or Z axis
     */
    export function readHostAcceleration(axis: HostAccelAxis): number {
        let data = readRegisters(REGISTER_HOST_ACCELERATION_START + axis * 2, 2)
        return data.length == 2 ? readI16Le(data, 0) : 0
    }

    //% group="Gyroscope"
    //% blockId=smartmotor_gyro_angle block="host $axis angle (degrees)"
    //% axis.defl=smartMotor.GyroAxis.Pitch
    //% weight=58
    /**
     * Read the gyroscope attitude angle in degrees.
     * @param axis pitch, yaw, or roll axis
     */
    export function readGyroAngle(axis: GyroAxis): number {
        let data = readRegisters(REGISTER_GYRO_ANGLE_START + axis * 4, 4)
        return data.length == 4 ? readI32Le(data, 0) / 10 : 0
    }

    //% group="Information"
    //% blockId=smartmotor_firmware_version block="firmware version"
    //% weight=50
    /**
     * Read the controller firmware version.
     */
    export function readVersion(): string {
        let data = i2cQueryRead(COMMAND_VERSION, [], 3)
        return "V " + data[0] + "." + data[1] + "." + data[2]
    }
}
