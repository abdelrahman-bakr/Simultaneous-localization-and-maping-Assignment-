/*
 *
 * robotControl.h - declarations for robot controller
 *
 * Platform: Any POSIX compliant platform
 * Intended for and tested on: Cygwin 64 bit
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

#define MEMORY_MAPPED_FILE "robot_shared_file"

#define MAXIMUM_MAP_SIZE 16

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

#define START_ROBOT_HEADING EAST
#define START_DISTANCE_SENSOR_HEADING EAST;

#define POLL_LOOP_RATE 40           // poll loops per second - should be less than the simulator poll loop rate for reliable operation
#define SCREEN_PRINT_RATE 2         // screen prints per second

#define ON 1
#define OFF 0

#define FORWARD 1
#define BACKWARD -1

#define CLOCKWISE 1
#define ANTICLOCKWISE -1

#define OCCUPIED 1
#define NOT_OCCUPIED 0
#define UNKNOWN_WHETHER_OCCUPIED_OR_NOT_OCCUPIED -1

#define RANGE_OF_DISTANCE_SENSOR 2
#define NO_OBJECT_IN_RANGE -1

#define SET_ROBOT_TRACK_MOTORS_SUCCESS 0
#define SET_ROBOT_TRACK_MOTORS_ILLEGAL_MOTOR_CONFIG -1
#define SET_ROBOT_TRACK_MOTORS_NOT_READY_TO_ACCEPT_INSTRUCTION -2
#define SET_ROBOT_TRACK_MOTORS_UNABLE_TO_MOVE_IN_SPECIFIED_DIRECTION -3
#define SET_ROBOT_DISTANCE_SENSOR_MOTOR_SUCCESS 0
#define SET_ROBOT_DISTANCE_SENSOR_MOTOR_ILLEGAL_MOTOR_CONFIG -1
#define SET_ROBOT_DISTANCE_SENSOR_MOTOR_NOT_READY_TO_ACCEPT_INSTRUCTION -2

#define START_I 0
#define START_J 0

#define MANUAL_CONTROL 1
#define AUTONOMOUS_CONTROL 2

#define NO_KEY 0

#define SET_MAP_CONTENTS_SUCCESS 0
#define SET_MAP_CONTENTS_INVALID_LOCATION -1
#define SET_MAP_CONTENTS_INVALID_VALUE -2

typedef struct
{
    double sim_time;
    int number_of_rows;
    int number_of_columns;
    int motor_left_track;
    int motor_right_track;
    int motor_distance_sensor;
    int touch_sensor_value_north;
    int touch_sensor_value_east;
    int touch_sensor_value_south;
    int touch_sensor_value_west;
    int distance_sensor_value;
    int ready_for_next_instruction;
    int mode;
    int quit;

} Robot;

struct termios setTerminalSettings(void);

void resetTerminalSettings(struct termios old_term);

void *getKeyPress(void *arguments);

void robotOpen();

void robotClose();

void initializeMapContents(int mapContents[][MAXIMUM_MAP_SIZE]);

int setMapContents(int mapContents[][MAXIMUM_MAP_SIZE], int, int, int);

void printMapContents(int mapContents[][MAXIMUM_MAP_SIZE], int, int);

void printScreenLine(double, int, int, int, int, int, int, int, int, int, char* stateName);

int setRobotTrackMotors(int, int, int);

int setRobotDistanceSensorMotor(int);

double getSimTime();

int getMapNumberOfRows();

int getMapNumberOfColumns();

int getOperationMode();

int getTouchSensorValueNorth();

int getTouchSensorValueEast();

int getTouchSensorValueSouth();

int getTouchSensorValueWest();

int getDistanceSensorValue();

int isSimulatorReadyForNextInstruction();

char getKey();

int isRobotSimulationQuitFlagOn();

void sleepMilliseconds(long ms);


