/*
 *
 * robotControlInterface.c - the interface routines for robot control, which simplify
 * interfacing to the simulator
 *
 * This program creates a shared memory segment with the simulator via a memory mapped file
 *
 * Platform: Any POSIX compliant platform
 * Intended for and tested on: Cygwin 64 bit
 *
 */

#include "robotControl.h"

Robot *pRobot;
int fd;
struct termios old_term;
pthread_t key_thread;
char keyPressed;

/*
 Function: setTerminalSettings
 -----------------------------
 Written by Jason Brown
 Date: 3/02/2020
 Version 1.0
 Purpose:
 sets the terminal settings to disable
 character echoing and line buffering
 Argument(s): none
 Return Value:
 a termios struct representing the original
 terminal settings for future restoration
 Usage: struct termios old_term = setTerminalSettings();
 */
struct termios setTerminalSettings() {

    struct termios old_term, new_term;

     /* Get old terminal settings for future restoration */
    tcgetattr(0, &old_term);

    /* Copy the settings to the new value */
    new_term = old_term;

    /* Disable echo of the character and line buffering */
    new_term.c_lflag &= (~ICANON & ~ECHO);

    /* Set new settings to the terminal */
    tcsetattr(0, TCSANOW, &new_term);

    return old_term;
}

/*
 Function: resetTerminalSettings
 -------------------------------
 Written by Jason Brown
 Date: 3/02/2020
 Version 1.0
 Purpose: resets the terminal settings
 Argument(s):
 struct termios old_term - terminal settings to restore
 Return Value: none
 Usage: resetTerminalSettings(old_term);
 */
void resetTerminalSettings(struct termios old_term) {

    /* Restore old settings */
    tcsetattr(0, TCSANOW, &old_term);

}

/*
 Function: getKeyPress
 ---------------------
 Written by Jason Brown
 Date: 3/02/2020
 Version 1.0
 Purpose:
 thread function to handle keyboard input, called as part
 of creation of a new thread
 Argument(s):  None
 Return Value: none
 Usage: not called directly but via pthread_create()
 */
void *getKeyPress(void *arguments)
{
    do {

        keyPressed = getchar();

    } while ((keyPressed != 'q') && (keyPressed != 'Q'));

    pRobot -> quit = ON;
    return NULL;
}

/*
 Function: robotOpen
 -------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose: sets the terminal settings, creates a separate thread to handle
 keyboard input, initializes and memory maps a file so that a shared memory
 segment is created with the simulator
 Argument(s): none
 Return Value: none
 Usage: robotOpen();
 */
void robotOpen()
{
    /* disable character echoing and line buffering */
    old_term = setTerminalSettings();

    /* create separate thread to handle keyboard input */
    int res = pthread_create(&key_thread, NULL, getKeyPress, NULL);
    if (res != 0)
    {
        perror("Problem creating thread to handle user input");
        exit(1);
    }

    /* initialize file */
    fd = open(MEMORY_MAPPED_FILE, (O_CREAT | O_RDWR), 0666);
    if (fd < 0)
    {
        perror("creation/opening of file failed");
        exit(1);
    }
    ftruncate(fd, sizeof(Robot));

    /* map the file to memory */
    pRobot = (Robot *)mmap(0, sizeof(Robot), (PROT_READ | PROT_WRITE),  MAP_SHARED, fd, (off_t)0);
    if (pRobot == MAP_FAILED)
    {
        perror("memory mapping of file failed");
        close(fd);
        exit(2);
    }
    //resetRobot(pRobot);
}

/*
 Function: robotClose
 --------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose: indicates to the simulator that the controller is quitting,
 unmaps the memory mapped file, closes the associated file descriptor
 and resets the terminal settings
 Argument(s): none
 Return Value: none
 Usage: robotClose();
 */
void robotClose()
{
    pRobot -> quit = ON;
    munmap(pRobot, sizeof(Robot));
    close(fd);

    /* reset terminal settings to original values */
    resetTerminalSettings(old_term);
}

/*
 Function: initializeMapContents
 -------------------------------
 Written by Jason Brown
 Date: 3/02/2020
 Version 1.0
 Purpose:
 initializes all (valid) locations in the controller map to UNKNOWN_WHETHER_OCCUPIED_OR_NOT_OCCUPIED (-1)
 except (0,0) which is always UNOCCUPIED (0)
 Argument(s):
 The following arguments are passed by reference and so are available to the calling function:
 int mapContents[][MAXIMUM_MAP_SIZE] - a pointer to an integer 2D array representing the map
 Return Value:
 none
 Usage: initializeMapContents(mapContents);
 */
void initializeMapContents(int mapContents[][MAXIMUM_MAP_SIZE])
{

    for (int i = 0; i < pRobot -> number_of_rows; i++)
    {
        for (int j = 0; j < pRobot -> number_of_columns; j++)
        {
                mapContents[i][j] = UNKNOWN_WHETHER_OCCUPIED_OR_NOT_OCCUPIED;
        }
        mapContents[0][0] = NOT_OCCUPIED;
    }
}

/*
 Function: setMapContents
 ------------------------
 Written by Jason Brown
 Date: 3/02/2020
 Version 1.0
 Purpose:
 sets the contents of one valid location in the controller map to OCCUPIED (1), NOT_OCCUPIED (0) or
 UNKNOWN_WHETHER_OCCUPIED_OR_NOT_OCCUPIED (-1)
 Argument(s):
 int i - the row of the location which is to be set in the controller map
 int j - the column of the location which is to be set in the controller map
 int value = the value to be set
 Return Value:
 one of:
 SET_MAP_CONTENTS_SUCCESS (0)
 SET_MAP_CONTENTS_INVALID_LOCATION (-1)
 SET_MAP_CONTENTS_INVALID_VALUE (-2)
 Usage: int result = setMapContents(mapContents, 3, 4, OCCUPIED);
 */
int setMapContents(int mapContents[][MAXIMUM_MAP_SIZE], int i, int j, int value)
{

    if (i < 0 || i >= pRobot -> number_of_rows || j < 0 || j >= pRobot -> number_of_columns) return SET_MAP_CONTENTS_INVALID_LOCATION;
    if (value != OCCUPIED && value != NOT_OCCUPIED && value != UNKNOWN_WHETHER_OCCUPIED_OR_NOT_OCCUPIED) return SET_MAP_CONTENTS_INVALID_VALUE;
    mapContents[i][j] = value;
    return SET_MAP_CONTENTS_SUCCESS;
}

/*
 Function: printMapContents
 --------------------------
 Written by Jason Brown
 Date: 3/02/2020
 Version 1.0
 Purpose:
 prints the controller map as currently known using '1' for OCCUPIED, '0' for NOT_OCCUPIED
 and 'X' for UNKNOWN_WHETHER_OCCUPIED_OR_NOT_OCCUPIED, also prints the current robot location
 Argument(s):
 The following arguments are passed:
 int mapContents[][MAXIMUM_MAP_SIZE] - a pointer to an integer 2D array representing the map
 int robot_i - current i co-ordinate of robot
 int robot_j - current j co-ordinate of robot
 Return Value:
 none
 Usage: printMapContents(mapContents);
 */
void printMapContents(int mapContents[][MAXIMUM_MAP_SIZE], int robot_i, int robot_j)
{
    printf("\nKnown map contents, map is %i x %i:\n", pRobot -> number_of_rows, pRobot -> number_of_columns);
    printf("0: UNOCCUPIED\n");
    printf("1: OCCUPIED\n");
    printf("X: UNKNOWN\n");
    printf("R: ROBOT\n\n");

    for (int i = 0; i < pRobot -> number_of_rows; i++)
    {
        for (int j = 0; j < pRobot -> number_of_columns; j++)
        {
            if ((robot_i == i) && (robot_j == j)) printf("R ");
            else switch(mapContents[i][j])
            {
                case OCCUPIED:
                    printf("1 ");
                    break;
                case NOT_OCCUPIED:
                    printf("0 ");
                    break;
                case UNKNOWN_WHETHER_OCCUPIED_OR_NOT_OCCUPIED:
                    printf("X ");
                    break;
            }
        }
        printf("\n");
    }
    printf("\n");
}

/*
 Function: printScreenLine
 -------------------------
 Written by Jason Brown
 Date: 4/02/2020
 Version 1.0
 Purpose:
 prints a line of status information to the screen
 Argument(s):
 The following arguments are passed:
 double simTime - current simulation time
 int robot_i - current i co-ordinate of robot
 int robot_j - current j co-ordinate of robot
 int robotHeading - one of NORTH (0), EAST (1), SOUTH (2) or WEST (3)
 int tn - current touch sensor value for NORTH, one of NOT_OCCUPIED (0) or OCCUPIED (1)
 int te - current touch sensor value for EAST, one of NOT_OCCUPIED (0) or OCCUPIED (1
 int ts - current touch sensor value for SOUTH, one of NOT_OCCUPIED (0) or OCCUPIED (1)
 int tw - current touch sensor value for WEST, one of NOT_OCCUPIED (0) or OCCUPIED (1)
 int distanceSensorHeading - one of NORTH (0), EAST (1), SOUTH (2) or WEST (3) for the distance sensor
 int distanceSensorValue - 0..RANGE_OF_DISTANCE_SENSOR otherwise NO_OBJECT_IN_RANGE (-1)
 char* stateName - name of the current state
 Return Value:
 none
 Usage: printScreenLine(getSimTime(), i, j, robotHeading, getTouchSensorValueNorth(), getTouchSensorValueEast(), getTouchSensorValueSouth(), getTouchSensorValueWest(), distanceSensorHeading, getDistanceSensorValue(), statename[state]);
 */
void printScreenLine(double simTime, int i, int j, int robotHeading, int tn, int te, int ts, int tw, int distanceSensorHeading, int distanceSensorValue, char* stateName)
{
    printf("time:%7.2f  ", simTime);
    printf("i:%i  ", i);
    printf("j:%i  ", j);

    if      (robotHeading == NORTH) printf("rbt hdg:N  ");
    else if (robotHeading == EAST)  printf("rbt hdg:E  ");
    else if (robotHeading == SOUTH) printf("rbt hdg:S  ");
    else if (robotHeading == WEST)  printf("rbt hdg:W  ");

    printf("touch N:%i  ", tn);
    printf("E:%i  ", te);
    printf("S:%i  ", ts);
    printf("W:%i  ", tw);

    if      (distanceSensorHeading == NORTH) printf("snsr hdg:N  ");
    else if (distanceSensorHeading == EAST)  printf("snsr hdg:E  ");
    else if (distanceSensorHeading == SOUTH) printf("snsr hdg:S  ");
    else if (distanceSensorHeading == WEST)  printf("snsr hdg:W  ");

    printf("dis:%i  ", distanceSensorValue);
    printf("state: %s  ", stateName);
    printf("\n");
}

/*
 Function: setRobotTrackMotors
 -----------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 sets the robot track motors for moving (1 unit) and turning (90 degrees) on a mutually exclusive basis, if turning the
 distance sensor also turns in sympathy
 Argument(s):
 int motorLeftTrack - the activation and direction of the motor for the left track of the robot (viewed from above)
 int motorRightTrack - the activation and direction of the motor for the right track of the robot (viewed from above)
 Each motor setting must be one of MOTOR_OFF (0), FORWARD (1), BACKWARD (-1)
 int robotHeading - the current heading of the robot (viewed from above), must be one of NORTH (0), EAST (1), SOUTH (2) or WEST (3)
 Return Value:
 one of:
 SET_ROBOT_TRACK_MOTORS_SUCCESS (0)
 SET_ROBOT_TRACK_MOTORS_ILLEGAL_MOTOR_CONFIG (-1): illegal motor configuration request
 SET_ROBOT_TRACK_MOTORS_NOT_READY_TO_ACCEPT_INSTRUCTION (-2): simulator still executing previous instruction
 SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION (-3): attempting to move forward or backward
 when an obstacle prevents such movement or when such movement would take the robot outside the limits of the map
 Usage: int result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading); // turn the robot clockwise by 90 degrees as viewed from above
 */
int setRobotTrackMotors(int motorLeftTrack, int motorRightTrack, int robotHeading)
{
    if (!isSimulatorReadyForNextInstruction()) return SET_ROBOT_TRACK_MOTORS_NOT_READY_TO_ACCEPT_INSTRUCTION;
    /* controller wishes to move the robot forward */
    if (motorLeftTrack == FORWARD && motorRightTrack == FORWARD)
    {
        switch (robotHeading)
        {
            case NORTH:
                if (pRobot -> touch_sensor_value_north == OCCUPIED) return SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION;
                break;
            case EAST:
                if (pRobot -> touch_sensor_value_east == OCCUPIED)  return SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION;
                break;
            case SOUTH:
                if (pRobot -> touch_sensor_value_south == OCCUPIED) return SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION;
                break;
            case WEST:
                if (pRobot -> touch_sensor_value_west == OCCUPIED)  return SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION;
                break;
        }
        pRobot -> motor_left_track  = motorLeftTrack;
        pRobot -> motor_right_track = motorRightTrack;
        return SET_ROBOT_TRACK_MOTORS_SUCCESS;
    }
    /* controller wishes to move the robot backward */
    else if (motorLeftTrack == BACKWARD && motorRightTrack == BACKWARD)
    {
        switch (robotHeading)
        {
            case NORTH:
                if (pRobot -> touch_sensor_value_south == OCCUPIED) return SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION;
                break;
            case EAST:
                if (pRobot -> touch_sensor_value_west == OCCUPIED)  return SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION;
                break;
            case SOUTH:
                if (pRobot -> touch_sensor_value_north == OCCUPIED) return SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION;
                break;
            case WEST:
                if (pRobot -> touch_sensor_value_east == OCCUPIED)  return SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION;
                break;
        }
        pRobot -> motor_left_track  = motorLeftTrack;
        pRobot -> motor_right_track = motorRightTrack;
        return SET_ROBOT_TRACK_MOTORS_SUCCESS;
    }
    /* controller wishes to turn the robot clockwise or anticlockwise */
    else if ((motorLeftTrack == FORWARD && motorRightTrack == BACKWARD) || (motorLeftTrack == BACKWARD && motorRightTrack == FORWARD))
    {
        pRobot -> motor_left_track  = motorLeftTrack;
        pRobot -> motor_right_track = motorRightTrack;
        return SET_ROBOT_TRACK_MOTORS_SUCCESS;
    }

    /* any other motor configuration is illegal */
    return SET_ROBOT_TRACK_MOTORS_ILLEGAL_MOTOR_CONFIG;
}

/*
 Function: setRobotDistanceSensorMotor
 -------------------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 sets the robot distance sensor motor for turning the distance sensor clockwise or anticlockwise by 90 degrees
 Argument(s):
 int motorDistanceSensor - the activation and direction of the motor for the distance sensor of the robot (viewed from above)
 The motor setting must be one of MOTOR_OFF (0), CLOCKWISE (1), ANTICLOCKWISE (-1)
 Return Value:
 one of:
 SET_ROBOT_DISTANCE_SENSOR_MOTOR_SUCCESS (0)
 SET_ROBOT_DISTANCE_SENSOR_MOTOR_ILLEGAL_MOTOR_CONFIG (-1): illegal motor configuration request
 SET_ROBOT_DISTANCE_SENSOR_MOTOR_NOT_READY_TO_ACCEPT_INSTRUCTION (-2): simulator still executing previous instruction
 Usage: int result = setRobotDistanceSensorMotor(CLOCKWISE); // turn the distance sensor clockwise by 90 degrees as viewed from above
 */
int setRobotDistanceSensorMotor(int motorDistanceSensor)
{
    if (!isSimulatorReadyForNextInstruction()) return SET_ROBOT_DISTANCE_SENSOR_MOTOR_NOT_READY_TO_ACCEPT_INSTRUCTION;
    switch(motorDistanceSensor)
    {
        case OFF:
        case CLOCKWISE:
        case ANTICLOCKWISE:
            pRobot -> motor_distance_sensor = motorDistanceSensor;
            return SET_ROBOT_DISTANCE_SENSOR_MOTOR_SUCCESS;
        default:
            /* any other motor configuration is illegal */
            return SET_ROBOT_DISTANCE_SENSOR_MOTOR_ILLEGAL_MOTOR_CONFIG;

    }
}

/*
 Function: getSimTime
 --------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 gets the current simulation time from the simulator in seconds,
 this is not necessarily real time
 Argument(s):
 none
 Return Value:
 a double representing current simulation time
 Usage:
 double simTime = getSimTime();
 */
double getSimTime()
{
    return pRobot -> sim_time;
}

/*
 Function: getMapNumberOfRows
 ----------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 gets the number of rows in the map
 Argument(s):
 none
 Return Value:
 an int representing the number of rows in the map
 Usage:
 int numberOfRows = getMapNumberOfRows();
 */
int getMapNumberOfRows()
{
    return pRobot -> number_of_rows;
}

/*
 Function: getMapNumberOfColumns
 -------------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 gets the number of columns in the map
 Argument(s):
 none
 Return Value:
 an int representing the number of columns in the map
 Usage:
 int numberOfColumns = getMapNumberOfColumns();
 */
int getMapNumberOfColumns()
{
    return pRobot -> number_of_columns;
}

/*
 Function: getOperationMode
 --------------------------
 Written by Jason Brown
 Date: 6/02/2020
 Version 1.0
 Purpose:
 gets the operation mode (MANUAL or AUTO) for control
 Argument(s):
 none
 Return Value:
 an int representing the operation mode
 Usage:
 int operationMode = getOperationMode();
 */
int getOperationMode()
{
    return pRobot -> mode;
}

/*
 Function: getTouchSensorValueNorth
 ----------------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 gets the value of the touch sensor currently oriented towards north
 Argument(s):
 none
 Return Value:
 an int representing the value of the touch sensor currently oriented towards north
 Usage:
 int touchSensorValueNorth = getTouchSensorValueNorth();
 */
int getTouchSensorValueNorth()
{
    return pRobot -> touch_sensor_value_north;
}

/*
 Function: getTouchSensorValueEast
 ---------------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 gets the value of the touch sensor currently oriented towards east
 Argument(s):
 none
 Return Value:
 an int representing the value of the touch sensor currently oriented towards east
 Usage:
 int touchSensorValueEast = getTouchSensorValueEast();
 */
int getTouchSensorValueEast()
{
    return pRobot -> touch_sensor_value_east;
}

/*
 Function: getTouchSensorValueSouth
 ----------------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 gets the value of the touch sensor currently oriented towards south
 Argument(s):
 none
 Return Value:
 an int representing the value of the touch sensor currently oriented towards south
 Usage:
 int touchSensorValueSouth = getTouchSensorValueSouth();
 */
int getTouchSensorValueSouth()
{
    return pRobot -> touch_sensor_value_south;
}

/*
 Function: getTouchSensorValueWest
 ---------------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 gets the value of the touch sensor currently oriented towards west
 Argument(s):
 none
 Return Value:
 an int representing the value of the touch sensor currently oriented towards west
 Usage:
 int touchSensorValueWest = getTouchSensorValueWest();
 */
int getTouchSensorValueWest()
{
    return pRobot -> touch_sensor_value_west;
}

/*
 Function: getDistanceSensorValue
 --------------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 gets the value of the distance sensor for its current heading
 Argument(s):
 none
 Return Value:
 an int representing the value of the distance sensor for its current heading
 Usage:
 int distanceSensorValue = getDistanceSensorValue();
 */
int getDistanceSensorValue()
{
    return pRobot -> distance_sensor_value;
}

/*
 Function: isSimulatorReadyForNextInstruction
 --------------------------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 provides information on whether the simulator has finished executing the previous instruction
 Argument(s):
 none
 Return Value:
 an int representing whether the simulator has finished executing the previous instruction (1) or not (0)
 Usage:
 int simulatorIsReadyForNextInstruction = isSimulatorReadyForNextInstruction();
 */
int isSimulatorReadyForNextInstruction()
{
    return pRobot -> ready_for_next_instruction;
}

/*
 Function: getKey
 -------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 gets the most recent key press by the user which has not already been handled, then resets
 the key to NO_KEY to prevent handling the same key press more than once
 Argument(s):
 none
 Return Value:
 the most recent key press by the user which has not already been handled as a char,
 otherwise NO_KEY (0)
 Usage:
 char c = getKey();
 */
char getKey()
{
    char c;
    c = keyPressed;
    keyPressed = NO_KEY;
    return c;
}

/*
 Function: isRobotSimulationQuitFlagOn
 -------------------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose:
 determines whether the simulator is quitting after receiving a 'q' key press
 Argument(s):
 none
 Return Value:
 one of:
 OFF (0) - quit flag off
 ON (1) - quit flag on
 Usage:
 int quiteFlagOn = isRobotSimulationQuitFlagOn();
 */
int isRobotSimulationQuitFlagOn()
{
    return pRobot -> quit;
}

/*
 Function: sleepMilliseconds
 ---------------------------
 Written by Jason Brown
 Date: 2/02/2020
 Version 1.0
 Purpose: put the calling thread to sleep
 for a certain number of ms
 Argument(s):
 long ms - the number of ms to sleep
 Return Value: none
 Usage: sleepMilliseconds(20);
 */
void sleepMilliseconds(long ms)
{
    struct timespec ts;

    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

