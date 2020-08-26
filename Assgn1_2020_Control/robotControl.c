/*
 *
 * robotControl.c - the controller for the robot in manual and autonomous mode
 *
 * Platform: Any POSIX compliant platform
 * Intended for and tested on: Cygwin 64 bit
 *
 */

#include "robotControl.h"

/****************************
private functions declaration
***************************/
static void prv_MapUpdate_TouchSensors(void);
static void prv_MapUpdate_DistanceSensor(void);
static void prv_VisitedMapUpdate(void);
static void prv_VisibleMapUpdate(void);
static int  whole_Map_Visible();


static void prv_TurnRobotEast();
static void prv_TurnRobotSouth();
static void prv_TurnRobotWest();
static void prv_TurnRobotNorth();

/******************************
global variables
******************************/

 int i = 0, j = 0, robotHeading = EAST, distanceSensorHeading = EAST;   //     these variables are now declared global...
 int mapContents[MAXIMUM_MAP_SIZE][MAXIMUM_MAP_SIZE];                   //           ... to be used by functions other than main function
 int mapIsVisted[MAXIMUM_MAP_SIZE][MAXIMUM_MAP_SIZE] ;
 int mapIsVisible[MAXIMUM_MAP_SIZE][MAXIMUM_MAP_SIZE] ;

// state names and numbers
#define ROBOT_STOPPED                           0
#define ROBOT_MOVING_FORWARDS                   1
#define ROBOT_MOVING_BACKWARDS                  2
#define ROBOT_TURNING_CLOCKWISE                 3
#define ROBOT_TURNING_ANTICLOCKWISE             4
#define DISTANCE_SENSOR_TURNING_CLOCKWISE       5
#define DISTANCE_SENSOR_TURNING_ANTICLOCKWISE   6


// states for self drive mode
#define ROBOT_DECIDING         7

/* statenames of up to 13 characters (the 14th character is a null terminator), only required for display purposes */
char statename[8][14] = {"STOPPED      ", "FORWARD      ", "BACKWARD     ", "RBT CLCKWSE  ", "RBT ACLCKWSE ", "SNSR CLCKWSE ", "SNSR ACLCKWSE","DECIDING      "};

int main()
{
    /* Initialise screenPrintCounter to be the number of poll loops which occur
       between consecutive printouts to the screen
       The frequency of screen printing must be limited so that a viewer can
       meaningfully observe the output
    */
    int screenPrintCounter = (int) POLL_LOOP_RATE / SCREEN_PRINT_RATE;

    robotOpen();


    /* mapContents is the 2D array that you populate as you move around */

    initializeMapContents(mapContents);




    if (getOperationMode() == MANUAL_CONTROL) {

        /*
         * some code for manual mode is provided, expand to meet the assignment requirements
         */

    	/* initial state */
        int state = ROBOT_STOPPED;

        printf("Operating in manual control mode\n");

        while(!isRobotSimulationQuitFlagOn())
        {

            int result;
            char c;

            sleepMilliseconds((long) 1000 / POLL_LOOP_RATE);

            c = getKey();

            switch (state)
            {
                case ROBOT_STOPPED:

                    if ((c == 'f') | (c == 'F'))
                    {
                        result = setRobotTrackMotors(FORWARD, FORWARD, robotHeading);
                        if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_MOVING_FORWARDS;
                        else printf("Instruction returned error: %i\n", result);
                    }
                    else if  ((c == 'b') | (c == 'B'))
                        {
                            result = setRobotTrackMotors(BACKWARD, BACKWARD, robotHeading);
                        if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_MOVING_BACKWARDS;
                        else printf("Instruction returned error: %i\n", result);
                        }
                        else if  ((c == 'c') | (c == 'C'))
                        {
                            result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading);
                        if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_CLOCKWISE;
                        else printf("Instruction returned error: %i\n", result);
                        }
                        else if  ((c == 'a') | (c == 'A'))
                        {
                            result = setRobotTrackMotors(BACKWARD, FORWARD, robotHeading);
                        if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_ANTICLOCKWISE;
                        else printf("Instruction returned error: %i\n", result);
                        }
                        else if  (c == '1')
                        {
                            result = setRobotDistanceSensorMotor(CLOCKWISE);
                        if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = DISTANCE_SENSOR_TURNING_CLOCKWISE;
                        else printf("Instruction returned error: %i\n", result);
                        }
                        else if  (c == '2')
                        {
                            result = setRobotDistanceSensorMotor(ANTICLOCKWISE);
                        if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = DISTANCE_SENSOR_TURNING_ANTICLOCKWISE;
                        else printf("Instruction returned error: %i\n", result);
                        }
                        else if  ((c == 'm') | (c == 'M'))
                        {
                            printMapContents(mapContents, i, j);
                        }
                        break;


                case ROBOT_MOVING_FORWARDS:

                    if (isSimulatorReadyForNextInstruction())
                    {
                        switch(robotHeading)
                        {
                            case NORTH: i--; break;
                            case EAST:  j++; break;
                            case SOUTH: i++; break;
                            case WEST:  j--; break;
                        }

                       // Update the map with my current location is NOT_OCCUPIED
                        setMapContents(mapContents,i,j,NOT_OCCUPIED);
                        // Update the map with 4 touch sensor values for my surrounding cells
                        prv_MapUpdate_TouchSensors();
                        // update the map with the reading of distance sensor in my direction
                        prv_MapUpdate_DistanceSensor();

                        state = ROBOT_STOPPED;
                    }
                    break;
                     case ROBOT_MOVING_BACKWARDS:

                    if (isSimulatorReadyForNextInstruction())
                    {
                        switch(robotHeading)
                        {
                            case NORTH: i++; break;
                            case EAST:  j--; break;
                            case SOUTH: i--; break;
                            case WEST:  j++; break;
                        }
                        // Update the map with my current location is NOT_OCCUPIED
                        setMapContents(mapContents,i,j,NOT_OCCUPIED);
                        // Update the map with 4 touch sensor values for my surrounding cells
                        prv_MapUpdate_TouchSensors();
                        // update the map with the reading of distance sensor in my direction
                        prv_MapUpdate_DistanceSensor();
                        // set state to stopped once again
                        state = ROBOT_STOPPED;
                    }
                    break;
                    case ROBOT_TURNING_CLOCKWISE:

                    if (isSimulatorReadyForNextInstruction())
                    {
                        // change the robot heading accordingly
                        switch(robotHeading)
                        {
                            case NORTH: robotHeading =EAST ; break;
                            case EAST:  robotHeading =SOUTH; break;
                            case SOUTH: robotHeading =WEST; break;
                            case WEST:  robotHeading =NORTH; break;
                        }
                         // change the distance sensor heading  accordingly
                        switch(distanceSensorHeading)
                        {
                            case NORTH: distanceSensorHeading =EAST ; break;
                            case EAST:  distanceSensorHeading =SOUTH; break;
                            case SOUTH: distanceSensorHeading =WEST; break;
                            case WEST:  distanceSensorHeading =NORTH; break;
                        }
                        // update the map with Distance sensor reading
                        prv_MapUpdate_DistanceSensor();

                        state = ROBOT_STOPPED;
                    }
                    break;
                     case ROBOT_TURNING_ANTICLOCKWISE:

                    if (isSimulatorReadyForNextInstruction())
                    {
                        // change the robot heading accordingly
                        switch(robotHeading)
                        {
                            case NORTH: robotHeading =WEST ; break;
                            case EAST:  robotHeading =NORTH; break;
                            case SOUTH: robotHeading =EAST; break;
                            case WEST:  robotHeading =SOUTH; break;
                        }
                         // change the distance sensor heading  accordingly
                        switch(distanceSensorHeading)
                        {
                            case NORTH: distanceSensorHeading =WEST ; break;
                            case EAST:  distanceSensorHeading =NORTH; break;
                            case SOUTH: distanceSensorHeading =EAST; break;
                            case WEST:  distanceSensorHeading =SOUTH; break;
                        }
                           // update the map with Distance sensor reading
                        prv_MapUpdate_DistanceSensor();
                        state = ROBOT_STOPPED;
                    }
                    break;
                 case DISTANCE_SENSOR_TURNING_CLOCKWISE:

                    if (isSimulatorReadyForNextInstruction())
                    {
                         // change the distance sensor heading  accordingly
                       switch(distanceSensorHeading)
                        {
                            case NORTH: distanceSensorHeading =EAST ; break;
                            case EAST:  distanceSensorHeading =SOUTH; break;
                            case SOUTH: distanceSensorHeading =WEST; break;
                            case WEST:  distanceSensorHeading =NORTH; break;
                        }
                           // update the map with Distance sensor reading
                        prv_MapUpdate_DistanceSensor();
                        state = ROBOT_STOPPED;
                    }
                    break;
                     case DISTANCE_SENSOR_TURNING_ANTICLOCKWISE:

                    if (isSimulatorReadyForNextInstruction())
                    {
                         // change the distance sensor heading  accordingly
                       switch(distanceSensorHeading)
                        {
                            case NORTH: distanceSensorHeading =WEST ; break;
                            case EAST:  distanceSensorHeading =NORTH; break;
                            case SOUTH: distanceSensorHeading =EAST; break;
                            case WEST:  distanceSensorHeading =SOUTH; break;
                        }
                           // update the map with Distance sensor reading
                        prv_MapUpdate_DistanceSensor();
                        state = ROBOT_STOPPED;
                    }
                    break;
             }

            screenPrintCounter--;

            /* only print to the screen when screenPrintCounter reaches zero */
            if (screenPrintCounter <= 0)
            {
                /* re-initialize screenPrintCounter */
                screenPrintCounter = (int) POLL_LOOP_RATE / SCREEN_PRINT_RATE;
                printScreenLine(getSimTime(), i, j, robotHeading, getTouchSensorValueNorth(), getTouchSensorValueEast(), getTouchSensorValueSouth(), getTouchSensorValueWest(), distanceSensorHeading, getDistanceSensorValue(), statename[state]);
            }
        }
    }
    /* mode is AUTONOMOUS_CONTROL */
    else
    {
        /*
         * Your code for autonomous mode goes here
         */

        printf("Operating in autonomous control mode\n");

        int state = ROBOT_STOPPED;

                /* initilaize is visted map contents with zeros */
                    for(int ii = 0 ; ii <getMapNumberOfRows()-1 ; ii++){
                    for(int  jj= 0 ; jj <getMapNumberOfColumns() -1; jj++){
                            mapIsVisted[ii][jj] = 0 ;
                         }
                    }

             /* initilaize is visible  map contents with zeros */
                    for(int ii = 0 ; ii <getMapNumberOfRows()-1 ; ii++){
                    for(int  jj= 0 ; jj <getMapNumberOfColumns()-1 ; jj++){
                            mapIsVisible[ii][jj] = 0 ;
                         }
                    }


                /// update visible map to avoid corners
                                /// some maps we can`t see corners
                                mapIsVisible[0][0]  =1 ;
                                mapIsVisible[0][getMapNumberOfColumns()-1]  =1 ;
                                mapIsVisible[getMapNumberOfRows()-1][getMapNumberOfColumns()-1]  =1 ;
                                mapIsVisible[getMapNumberOfRows()-1][0]  =1 ;

                /// update map contents and  visible map before starting to move
                                prv_MapUpdate_DistanceSensor();
                                prv_MapUpdate_TouchSensors();
                                prv_VisibleMapUpdate();

       while(!isRobotSimulationQuitFlagOn())
           {
                                int result;

                sleepMilliseconds((long) 1000 / POLL_LOOP_RATE);

            /****************************************
                    autonomous mode
            *****************************************/
            switch(state){

          case ROBOT_STOPPED:
               if( whole_Map_Visible() != 1 ){
                    /// the whole map is now visible on the screen
                        state = ROBOT_DECIDING ; // still deciding
                }
                else{
                        state = ROBOT_STOPPED ; // i am done
                                        }

                    break;
            case ROBOT_DECIDING:


                        if((getTouchSensorValueEast() == 0) && (j<getMapNumberOfColumns()-1) && (mapIsVisted[i][j+1] ==0 )  ){

                            switch(robotHeading){

                                    case NORTH:
                                           result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading);
                                           if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_CLOCKWISE;
                                           else printf("Instruction returned error: %i\n", result);
                                    break;
                                    case EAST:
                                            result = setRobotTrackMotors(FORWARD, FORWARD, robotHeading);
                                            if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_MOVING_FORWARDS;
                                            else printf("Instruction returned error: %i\n", result);
                                       break;
                                    case SOUTH:
                                            result = setRobotTrackMotors(BACKWARD, FORWARD, robotHeading);
                                            if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_ANTICLOCKWISE;
                                            else printf("Instruction returned error: %i\n", result);
                                    break;
                                    case WEST:
                                           result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading);
                                            if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_CLOCKWISE;
                                            else printf("Instruction returned error: %i\n", result);
                                     break;
                                            }
                                }

                                else{
                                       if((getTouchSensorValueSouth() == 0 ) && (i<getMapNumberOfRows()-1) && (mapIsVisted[i+1][j]  ==0 ) ){

                                       switch(robotHeading){
                                            case NORTH:
                                                   result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading);
                                                   if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_CLOCKWISE;
                                                   else printf("Instruction returned error: %i\n", result);
                                            break;
                                            case EAST:
                                                  result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading);
                                                   if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_CLOCKWISE;
                                                   else printf("Instruction returned error: %i\n", result);
                                            break;
                                            case SOUTH:
                                                    result = setRobotTrackMotors(FORWARD, FORWARD, robotHeading);
                                                    if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_MOVING_FORWARDS;
                                                    else printf("Instruction returned error: %i\n", result);
                                            break;
                                            case WEST:
                                                  result = setRobotTrackMotors(BACKWARD, FORWARD, robotHeading);
                                                    if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_ANTICLOCKWISE;
                                                    else printf("Instruction returned error: %i\n", result);
                                             break;
                                                    }
                                        }

                                        else{
                                                    if((getTouchSensorValueWest() == 0)&& (j>0) && (mapIsVisted[i][j-1]  ==0)){

                                                    switch(robotHeading){
                                                    case NORTH:
                                                            result = setRobotTrackMotors(BACKWARD, FORWARD, robotHeading);
                                                            if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_ANTICLOCKWISE;
                                                            else printf("Instruction returned error: %i\n", result);
                                                    break;
                                                    case EAST:
                                                          result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading);
                                                           if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_CLOCKWISE;
                                                           else printf("Instruction returned error: %i\n", result);
                                                    break;
                                                    case SOUTH:
                                                           result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading);
                                                           if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_CLOCKWISE;
                                                           else printf("Instruction returned error: %i\n", result);
                                                    break;
                                                    case WEST:
                                                         result = setRobotTrackMotors(FORWARD, FORWARD, robotHeading);
                                                            if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_MOVING_FORWARDS;
                                                            else printf("Instruction returned error: %i\n", result);
                                                     break;
                                                            }

                                                    }
                                                else{
                                                            if((getTouchSensorValueNorth() == 0)   &&(i>0) && (mapIsVisted[i-1][j]==0)  ){


                                                    switch(robotHeading){
                                                    case NORTH:
                                                          result = setRobotTrackMotors(FORWARD, FORWARD, robotHeading);
                                                            if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_MOVING_FORWARDS;
                                                            else printf("Instruction returned error: %i\n", result);
                                                    break;
                                                    case EAST:
                                                          result = setRobotTrackMotors(BACKWARD, FORWARD, robotHeading);
                                                            if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_ANTICLOCKWISE;
                                                            else printf("Instruction returned error: %i\n", result);
                                                    break;
                                                    case SOUTH:
                                                           result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading);
                                                           if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_CLOCKWISE;
                                                           else printf("Instruction returned error: %i\n", result);
                                                    break;
                                                    case WEST:
                                                        result = setRobotTrackMotors(FORWARD, BACKWARD, robotHeading);
                                                           if (result == SET_ROBOT_TRACK_MOTORS_SUCCESS) state = ROBOT_TURNING_CLOCKWISE;
                                                           else printf("Instruction returned error: %i\n", result);
                                                     break;
                                                            }
                                                }

                                                else{
                                                                    // robot stucked --> allow it to go back on the last visited cell
                                                                     mapIsVisted[i][j+1] = 0 ;
                                                                     mapIsVisted[i+1][j] = 0 ;
                                                                     mapIsVisted[i][j-1] = 0 ;
                                                                     mapIsVisted[i-1][j] = 0 ;


                                                    }

                                                }


                                        }

                                }





                break;


                case ROBOT_MOVING_FORWARDS:
                     if (isSimulatorReadyForNextInstruction())
                    {
                        switch(robotHeading)
                        {
                            case NORTH: i--; break;
                            case EAST:  j++; break;
                            case SOUTH: i++; break;
                            case WEST:  j--; break;
                        }

                       // Update the map with my current location is NOT_OCCUPIED
                        setMapContents(mapContents,i,j,NOT_OCCUPIED);
                        // Update the map with 4 touch sensor values for my surrounding cells
                        prv_MapUpdate_TouchSensors();
                        // update the map with the reading of distance sensor in my direction
                        prv_MapUpdate_DistanceSensor();
                        // update map is visited array
                        prv_VisitedMapUpdate();
                        // update is visible map
                        prv_VisibleMapUpdate();


                        state = ROBOT_STOPPED;
                    }
                break ;
                 case ROBOT_MOVING_BACKWARDS:

                if (isSimulatorReadyForNextInstruction())
                    {
                        switch(robotHeading)
                        {
                            case NORTH: i++; break;
                            case EAST:  j--; break;
                            case SOUTH: i--; break;
                            case WEST:  j++; break;
                        }
                        // Update the map with my current location is NOT_OCCUPIED
                        setMapContents(mapContents,i,j,NOT_OCCUPIED);
                        // Update the map with 4 touch sensor values for my surrounding cells
                        prv_MapUpdate_TouchSensors();
                        // update the map with the reading of distance sensor in my direction
                        prv_MapUpdate_DistanceSensor();
                          // update map is visited array
                        prv_VisitedMapUpdate();
                         // update is visible map
                        prv_VisibleMapUpdate();

                        // set state to deciding once again
                        state = ROBOT_STOPPED;
                    }
                break;
            case ROBOT_TURNING_CLOCKWISE:
                if (isSimulatorReadyForNextInstruction())
                    {
                        // change the robot heading accordingly
                        switch(robotHeading)
                        {
                            case NORTH: robotHeading =EAST ; break;
                            case EAST:  robotHeading =SOUTH; break;
                            case SOUTH: robotHeading =WEST; break;
                            case WEST:  robotHeading =NORTH; break;
                        }
                         // change the distance sensor heading  accordingly
                        switch(distanceSensorHeading)
                        {
                            case NORTH: distanceSensorHeading =EAST ; break;
                            case EAST:  distanceSensorHeading =SOUTH; break;
                            case SOUTH: distanceSensorHeading =WEST; break;
                            case WEST:  distanceSensorHeading =NORTH; break;
                        }
                        // update the map with Distance sensor reading
                        prv_MapUpdate_DistanceSensor();

                         // update is visible map
                         prv_VisibleMapUpdate();

                        state = ROBOT_STOPPED;
                    }
                break;
            case ROBOT_TURNING_ANTICLOCKWISE:
                 if (isSimulatorReadyForNextInstruction())
                    {
                        // change the robot heading accordingly
                        switch(robotHeading)
                        {
                            case NORTH: robotHeading =WEST ; break;
                            case EAST:  robotHeading =NORTH; break;
                            case SOUTH: robotHeading =EAST; break;
                            case WEST:  robotHeading =SOUTH; break;
                        }
                         // change the distance sensor heading  accordingly
                        switch(distanceSensorHeading)
                        {
                            case NORTH: distanceSensorHeading =WEST ; break;
                            case EAST:  distanceSensorHeading =NORTH; break;
                            case SOUTH: distanceSensorHeading =EAST; break;
                            case WEST:  distanceSensorHeading =SOUTH; break;
                        }
                           // update the map with Distance sensor reading
                        prv_MapUpdate_DistanceSensor();
                         // update is visible map
                        prv_VisibleMapUpdate();

                        state = ROBOT_STOPPED;
                    }
                break;
                case DISTANCE_SENSOR_TURNING_CLOCKWISE:

                    if (isSimulatorReadyForNextInstruction())
                    {
                         // change the distance sensor heading  accordingly
                       switch(distanceSensorHeading)
                        {
                            case NORTH: distanceSensorHeading =EAST ; break;
                            case EAST:  distanceSensorHeading =SOUTH; break;
                            case SOUTH: distanceSensorHeading =WEST; break;
                            case WEST:  distanceSensorHeading =NORTH; break;
                        }
                           // update the map with Distance sensor reading
                        prv_MapUpdate_DistanceSensor();
                        state = ROBOT_STOPPED;
                    }
                    break;
                case DISTANCE_SENSOR_TURNING_ANTICLOCKWISE:

                    if (isSimulatorReadyForNextInstruction())
                    {
                         // change the distance sensor heading  accordingly
                       switch(distanceSensorHeading)
                        {
                            case NORTH: distanceSensorHeading =WEST ; break;
                            case EAST:  distanceSensorHeading =NORTH; break;
                            case SOUTH: distanceSensorHeading =EAST; break;
                            case WEST:  distanceSensorHeading =SOUTH; break;
                        }
                           // update the map with Distance sensor reading
                        prv_MapUpdate_DistanceSensor();
                        state = ROBOT_STOPPED;
                    }
                    break;


            }

            screenPrintCounter--;

            /* only print to the screen when screenPrintCounter reaches zero */
            if (screenPrintCounter <= 0)
            {
                /* re-initialize screenPrintCounter */
                screenPrintCounter = (int) POLL_LOOP_RATE / SCREEN_PRINT_RATE;
                printScreenLine(getSimTime(), i, j, robotHeading, getTouchSensorValueNorth(), getTouchSensorValueEast(), getTouchSensorValueSouth(), getTouchSensorValueWest(), distanceSensorHeading, getDistanceSensorValue(),statename[state]);
                printMapContents(mapContents,i,j);
               // printMapContents(mapIsVisible,i,j);  //  this print line is used for debugging
                            }
        }
    }

    robotClose();
    return 0;
}

static void prv_MapUpdate_TouchSensors(void){

               if(i>0) setMapContents(mapContents,i-1,j,getTouchSensorValueNorth());
               if(j<7) setMapContents(mapContents,i,j+1,getTouchSensorValueEast());
               if(i<7) setMapContents(mapContents,i+1,j,getTouchSensorValueSouth());
               if(j>0) setMapContents(mapContents,i,j-1,getTouchSensorValueWest());


}

static void prv_MapUpdate_DistanceSensor(void){

            int Current_Distance_Sensor_Value = getDistanceSensorValue();
                        switch (Current_Distance_Sensor_Value){

                            case 0: // the facing cell is OCCUPIED
                            break;
                            case 1: // 1 cell only is not occupied
                                    //   switch to check where the distance sensor is looking
                                         switch(distanceSensorHeading)
                                            {
                                                case NORTH:  setMapContents(mapContents,i-1,j,NOT_OCCUPIED); break;
                                                case EAST:   setMapContents(mapContents,i,j+1,NOT_OCCUPIED); break;
                                                case SOUTH:  setMapContents(mapContents,i+1,j,NOT_OCCUPIED); break;
                                                case WEST:   setMapContents(mapContents,i,j-1,NOT_OCCUPIED); break;
                                            }
                            break;
                            case 2: // 2 cells are not occupied
                                   //   switch to check where the distance sensor is looking
                                             switch(distanceSensorHeading)
                                                {
                                                    case NORTH:  setMapContents(mapContents,i-1,j,NOT_OCCUPIED);
                                                                 setMapContents(mapContents,i-2,j,NOT_OCCUPIED);
                                                    break;
                                                    case EAST:   setMapContents(mapContents,i,j+1,NOT_OCCUPIED);
                                                                 setMapContents(mapContents,i,j+2,NOT_OCCUPIED);
                                                    break;
                                                    case SOUTH:  setMapContents(mapContents,i+1,j,NOT_OCCUPIED);
                                                                 setMapContents(mapContents,i+2,j,NOT_OCCUPIED);
                                                    break;
                                                    case WEST:  setMapContents(mapContents,i,j-1,NOT_OCCUPIED);
                                                                setMapContents(mapContents,i,j-2,NOT_OCCUPIED);
                                                    break;
                                                }
                            break;
                            case NO_OBJECT_IN_RANGE: // 3 grid are not occupied
                                    //   switch to check where the distance sensor is looking
                                             switch(distanceSensorHeading)
                                                {
                                                    case NORTH:  setMapContents(mapContents,i-1,j,NOT_OCCUPIED);
                                                                 setMapContents(mapContents,i-2,j,NOT_OCCUPIED);
                                                                 setMapContents(mapContents,i-3,j,NOT_OCCUPIED);
                                                    break;
                                                    case EAST:   setMapContents(mapContents,i,j+1,NOT_OCCUPIED);
                                                                 setMapContents(mapContents,i,j+2,NOT_OCCUPIED);
                                                                 setMapContents(mapContents,i,j+3,NOT_OCCUPIED);
                                                    break;
                                                    case SOUTH:  setMapContents(mapContents,i+1,j,NOT_OCCUPIED);
                                                                 setMapContents(mapContents,i+2,j,NOT_OCCUPIED);
                                                                 setMapContents(mapContents,i+3,j,NOT_OCCUPIED);
                                                    break;
                                                    case WEST:  setMapContents(mapContents,i,j-1,NOT_OCCUPIED);
                                                                setMapContents(mapContents,i,j-2,NOT_OCCUPIED);
                                                                setMapContents(mapContents,i,j-3,NOT_OCCUPIED);
                                                    break;
                                                }
                            break;
                        }


}
static void prv_VisibleMapUpdate(void){
                      mapIsVisible[i][j]  =1 ;
                      if(getTouchSensorValueEast() == 1 ){
                         mapIsVisible[i][j+1] =1 ;
                      }
                      if(getTouchSensorValueWest() == 1 ){
                         mapIsVisible[i][j-1]=1 ;
                      }
                      if(getTouchSensorValueSouth() == 1 ){
                         mapIsVisible[i+1][j] =1;
                      }
                       if(getTouchSensorValueNorth() == 1 ){
                         mapIsVisible[i-1][j] =1 ;
                      }
                      if(getDistanceSensorValue()==1){
                        switch(distanceSensorHeading)
                        {
                            case NORTH: mapIsVisible[i-1][j] =1 ; break;
                            case EAST:  mapIsVisible[i][j+1] =1 ; break;
                            case SOUTH: mapIsVisible[i+1][j] =1 ; break;
                            case WEST:  mapIsVisible[i][j-1] =1 ; break;
                        }
                      }
                      if(getDistanceSensorValue()==2){
                        switch(distanceSensorHeading)
                        {
                            case NORTH: mapIsVisible[i-1][j] =1 ;
                                        mapIsVisible[i-2][j] =1 ;
                             break;
                            case EAST:  mapIsVisible[i][j+1] =1 ;
                                        mapIsVisible[i][j+2] =1 ;
                             break;
                            case SOUTH: mapIsVisible[i+1][j] =1 ;
                                        mapIsVisible[i+2][j] =1 ;
                             break;
                            case WEST:  mapIsVisible[i][j-1] =1 ;
                                        mapIsVisible[i][j-2] =1 ;
                             break;
                        }
                      }
                      if(getDistanceSensorValue()==NO_OBJECT_IN_RANGE){
                        switch(distanceSensorHeading)
                        {
                            case NORTH: mapIsVisible[i-1][j] =1 ;
                                        mapIsVisible[i-2][j] =1 ;
                                        mapIsVisible[i-3][j] =1 ;
                             break;
                            case EAST:  mapIsVisible[i][j+1] =1 ;
                                        mapIsVisible[i][j+2] =1 ;
                                        mapIsVisible[i][j+3] =1 ;
                             break;
                            case SOUTH: mapIsVisible[i+1][j] =1 ;
                                        mapIsVisible[i+2][j] =1 ;
                                        mapIsVisible[i+3][j] =1 ;
                             break;
                            case WEST:  mapIsVisible[i][j-1] =1 ;
                                        mapIsVisible[i][j-2] =1 ;
                                        mapIsVisible[i][j-3] =1 ;
                             break;
                        }
                      }
}
static void prv_VisitedMapUpdate(void){

                      mapIsVisted[i][j]  = 1 ;
                      if(getTouchSensorValueEast() == 1 ){
                         mapIsVisted[i][j+1] =1 ;
                      }
                      if(getTouchSensorValueWest() == 1 ){
                         mapIsVisted[i][j-1]=1 ;
                      }
                      if(getTouchSensorValueSouth() == 1 ){
                         mapIsVisted[i+1][j] ++ ;
                      }
                       if(getTouchSensorValueNorth() == 1 ){
                         mapIsVisted[i-1][j] =1 ;
                      }
}


static void prv_TurnRobotEast(){

         switch(robotHeading){

                        case EAST:
                            break ;
                        case SOUTH: setRobotTrackMotors(BACKWARD,FORWARD,robotHeading); // move anti clock wise to be east
                            break ;
                        case WEST: setRobotTrackMotors(BACKWARD,FORWARD,robotHeading);  // move anti clock wise twice
                                   setRobotTrackMotors(BACKWARD,FORWARD,robotHeading);
                            break ;
                        case NORTH: setRobotTrackMotors(FORWARD,BACKWARD,robotHeading); // move  clock wise to be east
                            break ;
                       }
                       robotHeading = EAST;
                       distanceSensorHeading= EAST ;
}
static void prv_TurnRobotSouth(){

                switch(robotHeading){

                        case EAST: setRobotTrackMotors(FORWARD,BACKWARD,robotHeading); // move  clock wise to be south
                            break ;
                        case SOUTH:
                            break ;
                        case WEST: setRobotTrackMotors(BACKWARD,FORWARD,robotHeading);  // move anti clock wise to be south
                            break ;
                        case NORTH: setRobotTrackMotors(FORWARD,BACKWARD,robotHeading); // move  clock wise to twice
                                    setRobotTrackMotors(FORWARD,BACKWARD,robotHeading);
                            break ;
                       }
                        robotHeading = SOUTH;
                        distanceSensorHeading = SOUTH;
}

static void prv_TurnRobotWest(){
      switch(robotHeading){

                        case EAST: setRobotTrackMotors(BACKWARD,FORWARD,robotHeading);  // move anti clock wise twice
                                   setRobotTrackMotors(BACKWARD,FORWARD,robotHeading);
                            break ;
                        case SOUTH: setRobotTrackMotors(FORWARD,BACKWARD,robotHeading); // move  clock wise to be east
                            break ;
                        case WEST:
                            break ;
                        case NORTH: setRobotTrackMotors(BACKWARD,FORWARD,robotHeading); // move  anti clock wise to be east
                            break ;
                       }
                        robotHeading = WEST;
                    distanceSensorHeading = WEST ;
}
static void prv_TurnRobotNorth(){
          switch(robotHeading){

                        case EAST: setRobotTrackMotors(BACKWARD,FORWARD,robotHeading); // move anti   clock wise to be south
                            break ;
                        case SOUTH:setRobotTrackMotors(FORWARD,BACKWARD,robotHeading); // move  clock wise to twice
                                    setRobotTrackMotors(FORWARD,BACKWARD,robotHeading);
                            break ;
                        case WEST: setRobotTrackMotors(FORWARD,BACKWARD,robotHeading);  // move  clock wise to be south
                            break ;
                        case NORTH:
                            break ;
                       }
                        robotHeading = NORTH;
                        distanceSensorHeading = NORTH ;
}
static int whole_Map_Visible(){

                int all_ones  = 1 ;

                for(int ii = 0 ; ii < getMapNumberOfRows()-1 ; ii++){
                    for(int  jj= 0 ; jj < getMapNumberOfColumns() -1; jj++){

                           if( mapIsVisible[ii][jj] != 1   ) {
                                all_ones = 0 ;
                           }

                         }
                    }

                    return all_ones;
}
