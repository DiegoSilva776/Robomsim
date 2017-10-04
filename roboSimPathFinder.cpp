/**
 * actionExample.cpp
 *
 * Authors: Diego M. Silva, diegomsilva.com
 * Licese: MIT
 */
 #include "Aria.h"
 #include <ArAnalogGyro.h>
 #include <ArRecurrentTask.h>
 #include <ArKeyHandler.h>
 #include <thread>
 #include <string>
 #include <fstream>
 #include <iostream>
 #include <sstream>
 #include <algorithm>
 #include <iterator>
 #include <stack>
 #include <vector>
 #include <queue>
 
 using namespace std;
 
 /**
  * CLASSES
  */
 class MapBlock {

    public:
       double x1;
       double x2;
       double y1;
       double y2;
       
       bool isCoordWithinBlock(double x, double y) {
          return x1 < x && x < x2 && y1 < y && y < y2; 
       }

       void toString() {
          printf ("\n...........................................");
          printf ("\nMapBlock: ");
          printf ("\n(x[%.2f,%.2f], y[%.2f,%.2f])", x1, x2, y1, y2);
          printf ("\n...........................................");
       }

 };

 class State {

    public:
       int stepNumber;
       MapBlock mapBlock;
       bool triedActionForward;
       bool triedActionLeft;
       bool triedActionRight;
       bool triedActionBackwards;
       bool isForbiddenState;

       vector<State> children;

       void toString() {
          printf ("\nState");
          printf ("\nStep number: %d", stepNumber);
          mapBlock.toString();
          printf ("\nTried forward: %d", triedActionForward);
          printf ("\nTried left: %d", triedActionLeft);
          printf ("\nTried right: %d", triedActionRight);
          printf ("\nTried backward: %d", triedActionBackwards);
          printf ("\nIs a forbidden state: %d", isForbiddenState);
          printf ("\n-------------------------------------------");
       }

 };

 /**
  * DATA MANIPULATION VARIABLES & CONSTANTS
  */
 float MIN_COLISION_RANGE_MM = 400;
 float ROBOT_X_SIZE = 700;

 // Initialization
 int numArgs = 0;
 const string ARG_DROBOT_COORD = "-Drobot.coord";
 std::map<string, string> args;
 std::map<int, double> robotConfigArgs;
 std::map<int, double> goalConfigArgs;

 // Data manipudation
 const int ACTION_BACK = -1;
 const int ACTION_FRONT = 1;
 const int ACTION_LEFT = 2;
 const int ACTION_RIGHT = 3;

 std::stack<State> Q;
 std::stack<State> forbiddenStates;
 string path = "";

 bool mHasAchievedGoal = false;
 
 float mLatestX = 0.0;
 float mLatestY = 0.0;
 float mLatestRot = 0.0;
 float mLatestSpeed = 0.0;

 float mOriginX = 0.0;
 float mOriginY = 0.0;
 float mGoalX = 0.0;
 float mGoalY = 0.0;
 double mDistanceFromGoal = 0.0;

 double mLeftDistance = 0.0;
 double mFrontDistance = 0.0;
 double mRightDistance = 0.0;
 
 /**
  * FUNCTIONS' DECLARATION
  */ 
 /**
  * Input
  */
 void readConfigArgs(int argc, char **argv);
 void readDrobotArg(char **argv);

 /**
  * Setters & Getters
  */
 void setLatestX(float x);
 float getLatestX();
 void setLatestY(float y);
 float getLatestY();
 void setLatestRotation(float rot);
 float getLatestRotation();
 void setLatestSpeed(float speed);
 float getLatestSpeed();

 void setInitialX(float x);
 float getInitialX();
 void setInitialY(float y);
 float getInitialY();
 void setGoalX(float x);
 float getGoalX();
 void setGoalY(float y);
 float getGoalY();
 void updateDistanceFromGoal();
 double getDistanceFromGoal();

 void setLatestLeftDistance(double distance);
 float getLatestLeftDistance();
 void setLatestFrontDistance(double distance);
 float getLatestFrontDistance();
 void setLatestRightDistance(double distance);
 float getLatestRightDistance();
 
 void setHasAchievedGoal(bool hasAchievedIt);
 bool hasAchievedGoal();
 
 /**
  * Motion Commands
  */
 void normalizeRobotRotation(ArRobot &robot);
 void goForward(ArRobot &robot, int distanceMM);
 void goBackward(ArRobot &robot, int numBlocks);
 void turnLeft(ArRobot &robot);
 void turnRight(ArRobot &robot);
 void correctRotationAngle(ArRobot &robot);
 void turnBackwards(ArRobot &robot);
 void stop(ArRobot &robot);
 int mustHeadUp(ArRobot &robot);
 int mustHeadRight(ArRobot &robot);
 bool isHeadingUp(ArRobot &robot);
 bool isHeadingDown(ArRobot &robot);
 bool isHeadingRight(ArRobot &robot);
 bool isHeadingLeft(ArRobot &robot);
 
 /**
  * Monitoring Commands
  */
 State updateAgentState(ArRobot &robot, int action, int stepsCount);
 void updateRobotCoordinatesRotationSpeed(ArRobot &robot);
 void updateSonarReadings(ArRobot *robot);
 
 /**
  * DFS - Online
  */
 // TODO: Review the methods below once the implementation of the DFS algorithm is complete
 State getGoalState(ArRobot &robot);
 void deepFirstSearch(ArRobot &robot, State &rootState, State &goalState);
 void backtrackUntilLatestValidPosition(ArRobot &robot, stack<State> &path, State &state);
 void addStateToPath(stack<State> &path, State &state);
 bool wasPreviouslyVisited(stack<State> &path, double x, double y);
 void addToListForbiddenState(State &state);
 bool isForbiddenItem(State &state);
 int evaluateCandidatesTakeDecision(ArRobot &robot, State &state);
 double getDistanceAB(float aX, float aY, float bX, float bY);
 void verifyAchievedGoal(ArRobot &robot);
 void printPath(stack<State> &path);
 //./TODO

 /**
  * Log
  */
 void logRobotStatus(int stepsCount, ArRobot &robot);
 void logRobotStatusAndGoal(ArRobot &robot);
 void logSimpleMsg(ArRobot &robot, const char *msg);

 /**
  * MAIN
  */
 int main(int argc, char **argv) {
    // Init the robot and try to connect to it
    Aria::init();
 
    // Get the initialization arguments
    readConfigArgs(argc, argv);

    // Start map
    ArRobot robot;
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobotConnector robotConnector(&parser, &robot);
    
    if (!robotConnector.connectRobot()) {
       ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
       
       if (parser.checkHelpAndWarnUnparsed()) {
          Aria::logOptions();
          Aria::exit(1);
       }
    }
 
    if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
       Aria::logOptions();
       Aria::exit(1);
    }
   
    ArLog::log(ArLog::Normal, "simpleConnect: Connected to robot.");
   
    // Add Devices
    // Sonar
    ArSonarDevice sonar;
    robot.addRangeDevice(&sonar);

    // Turn the robot on
    robot.enableMotors();
    robot.runAsync(true);

    // Initialize the position of the robot, by rotating the robot to 0 degress
    normalizeRobotRotation(robot);
    
    // Update the status of the sensors and the speed of the robot 
    State rootState = updateAgentState(robot, 0, 0);
    State goalState = getGoalState(robot);
    deepFirstSearch(robot, rootState, goalState);

    // Print the path from the origin to the goal
    printPath(Q);

    robot.stopRunning();
    robot.waitForRunExit();
   
    // Make it possible to exit
    Aria::exit(0);
    return 0;
 }
 
 /**
  * FUNCTIONS' IMPLEMENTATION
  */
 /**
  * Input processing
  */
 void readConfigArgs(int argc, char **argv) {
    numArgs = argc - 1;
    readDrobotArg(argv);
 }

 void readDrobotArg(char **argv) {
    // Get the Drobot arg value
    std::stringstream ss;
    ss.str(argv[1]);
    std::string itemDrobotArg;

    while (getline(ss, itemDrobotArg, '=')) {
       args[ARG_DROBOT_COORD] = itemDrobotArg;
    }

    // Read the lines of the Drobot configuration file
    fstream drobotCoordsFile(args[ARG_DROBOT_COORD], fstream::in);
    std::map<int,string> drobotConfigFileLines;
    int drobotConfigFileLinesCount = 0;
    std::string drobotFileLine;
    
    while (getline(drobotCoordsFile, drobotFileLine)) {
      drobotConfigFileLines[drobotConfigFileLinesCount] = drobotFileLine;
      drobotConfigFileLinesCount++;
    }

    // Get the configuration arguments of the robot out of the Drobot file
    std::stringstream ssRobotConfig;
    ssRobotConfig.str(drobotConfigFileLines[0]);
    std::string itemRobotConfig;
    int robotConfigArgsCount = 0;

    while (getline(ssRobotConfig, itemRobotConfig, ',')) {
       robotConfigArgs[robotConfigArgsCount] = stod(itemRobotConfig);
       robotConfigArgsCount++;
    }

    // Get the configuration arguments of the goal out of the Drobot file
    std::stringstream ssGoalConfig;
    ssGoalConfig.str(drobotConfigFileLines[1]);
    std::string itemGoalConfig;
    int goalConfigArgsCount = 0;

    while (getline(ssGoalConfig, itemGoalConfig, ',')) {
       goalConfigArgs[goalConfigArgsCount] = stod(itemGoalConfig);
       goalConfigArgsCount++;
    }

    // Set up the initial position of the robot and the goal
    setInitialX(robotConfigArgs[0]);
    setInitialY(robotConfigArgs[1]);
    setLatestX(robotConfigArgs[0]);
    setLatestY(robotConfigArgs[1]);
    setLatestRotation(robotConfigArgs[2]);
    setGoalX(goalConfigArgs[0]);
    setGoalY(goalConfigArgs[1]);

    printf("\n........................");
    printf("\nCONFIGURATION ARGUMENTS:");
    printf("\n\nDrobot.coord filename:");
    printf("\n%s", args[ARG_DROBOT_COORD].c_str());
    printf("\n\nRobot config args: ");
    printf("\n%.2f", robotConfigArgs[0]);
    printf("\n%.2f", robotConfigArgs[1]);
    printf("\n%.2f", robotConfigArgs[2]);
    printf("\n\nGoal config args: ");
    printf("\n%.2f", goalConfigArgs[0]);
    printf("\n%.2f", goalConfigArgs[1]);
    printf("\n........................");

    drobotCoordsFile.close();
 }

 /**
  * Setters & Getters
  */
 void setInitialX(float x) {
    mOriginX = x;
 }

 float getInitialX() {
    return mOriginX;
 }

 void setInitialY(float y) {
    mOriginY = y;
 }

 float getInitialY() {
    return mOriginY;
 }

 void setLatestX(float x) {
    mLatestX = x;
 }
 
 float getLatestX() {
    return mLatestX;
 }
 
 void setLatestY(float y) {
    mLatestY = y;
 }
 
 float getLatestY() {
    return mLatestY;
 }
 
 void setGoalX(float x) {
    mGoalX = x;
 }

 float getGoalX() {
    return mGoalX;
 }

 void setGoalY(float y) {
    mGoalY = y;
 }

 float getGoalY() {
    return mGoalY;
 }
 
 void updateDistanceFromGoal() {
    mDistanceFromGoal = getDistanceAB(getLatestX(), getLatestY(), getGoalX(), getGoalY());
 }

 double getDistanceFromGoal() {
    return mDistanceFromGoal;
 }

 void setLatestRotation(float rot) {
    mLatestRot = rot;
 }
 
 float getLatestRotation() {
    return mLatestRot;
 }
 
 void setLatestSpeed(float speed) {
    mLatestSpeed = speed;
 }
 
 float getLatestSpeed() {
    return mLatestSpeed;
 }
 
 void setHasAchievedGoal(bool hasAchievedIt) {
    mHasAchievedGoal = hasAchievedIt;
 }
 
 void setLatestLeftDistance(double distance) {
    mLeftDistance  = distance;
 }

 float getLatestLeftDistance() {
    return mLeftDistance;
 }

 void setLatestFrontDistance(double distance) {
    mFrontDistance = distance; 
 }

 float getLatestFrontDistance() {
    return mFrontDistance;
 }

 void setLatestRightDistance(double distance) {
    mRightDistance = distance; 
 }

 float getLatestRightDistance() {
    return mRightDistance;
 }

 bool hasAchievedGoal() {
    return mHasAchievedGoal;
 }
 
 /**
  * Motion Commands
  */
 void normalizeRobotRotation(ArRobot &robot) {
    // Set the initial position of the robot logically
    robot.moveTo(ArPose(getLatestX(), getLatestY(), getLatestRotation()));
    ArUtil::sleep(1000);

    // Set the initial position of the robot phisically, step 1
    robot.lock();
    int numRoundsNormalization = 0;
    
    if (getLatestRotation() > 0) {
       robot.setRotVel(-getLatestRotation());
       ArLog::log(ArLog::Normal, "\nNormalizing the intial rotation of the robot %.2f", -getLatestRotation());
    } else {
       robot.setRotVel(getLatestRotation());
       ArLog::log(ArLog::Normal, "\nNormalizing the intial rotation of the robot %.2f", getLatestRotation());
    }
        
    robot.unlock();
    ArUtil::sleep(1000);

    // Set the initial position of the robot, step 2
    correctRotationAngle(robot);
 }

 void goForward(ArRobot &robot, int numBlocks) {
    const int BASE_BLOCK_VEL_MM_SEC = 300;
    const int BASE_BLOCK_TIME_MOTION_SEC = 2400;
   
    correctRotationAngle(robot);

    for (int i = 0; i < numBlocks; i++) {
       robot.lock();
       ArLog::log(ArLog::Normal, "\nMoving forward, block : (%.d/%.d)", i + 1, numBlocks);
       robot.setRotVel(0);
       robot.setVel(BASE_BLOCK_VEL_MM_SEC);
       robot.unlock();
 
       if (getDistanceFromGoal() < ROBOT_X_SIZE) {
          ArUtil::sleep(BASE_BLOCK_TIME_MOTION_SEC / 3);
       } else {
          ArUtil::sleep(BASE_BLOCK_TIME_MOTION_SEC);  
       }
       
       stop(robot);
    }
 }
 
 void goBackward(ArRobot &robot, int numBlocks) {
   const int BASE_BLOCK_VEL_MM_SEC = -300;
   const int BASE_BLOCK_TIME_MOTION_SEC = 2350;
 
   correctRotationAngle(robot);

   for (int i = 0; i < numBlocks; i++) {
      robot.lock();
      ArLog::log(ArLog::Normal, "\nMoving backward, block : (%.d/%.d)", i + 1, numBlocks);
      robot.setRotVel(0);
      robot.setVel(BASE_BLOCK_VEL_MM_SEC);
      robot.unlock();

      ArUtil::sleep(BASE_BLOCK_TIME_MOTION_SEC);
      stop(robot);
   }
 }
 
 void correctRotationAngle(ArRobot &robot) {
    robot.lock();
    float desiredCurrentAngle = round(robot.getTh() / 90) * 90;
    float correctionAngle = desiredCurrentAngle - robot.getTh();
    ArLog::log(ArLog::Normal, "\ncorrectionAngle: %.2f", correctionAngle);
    robot.setRotVel(correctionAngle);
    robot.unlock();

    ArUtil::sleep(1000); 
 }

 void turnLeft(ArRobot &robot) {
    robot.lock();
    ArLog::log(ArLog::Normal, "\nTurning left ...");
    robot.setRotVel(90);
    robot.unlock();
    
    ArUtil::sleep(1020);
    stop(robot);
 }
 
 void turnRight(ArRobot &robot) {
    robot.lock();
    ArLog::log(ArLog::Normal, "\nTurning right ...");
    robot.setRotVel(-90);
    robot.unlock();
 
    ArUtil::sleep(1030);
    stop(robot);
 }
 
 void turnBackwards(ArRobot &robot) {
    robot.lock();
    ArLog::log(ArLog::Normal, "\nTurning backwards ...");
    robot.unlock();
 
    turnLeft(robot);
    turnLeft(robot);
 }
 
 void stop(ArRobot &robot) {
    robot.lock();
    robot.stop();
    robot.unlock();

    ArUtil::sleep(1500);
 }

 int mustHeadUp(ArRobot &robot) {
    float dy = getGoalY() - robot.getY();

    if (dy > 0) {
       return 1;

    } else if (dy < 0) {
       return -1;

    } else {
       return 0;
    }
 }

 int mustHeadRight(ArRobot &robot) {
    float dx = getGoalX() - robot.getX();
  
    if (dx > 0) {
       return 1;

    } else if (dx < 0) {
       return -1;

    } else {
       return 0;
    }
 }

 bool isHeadingUp(ArRobot &robot) {
    return robot.getTh() >= 85 && robot.getTh() <= 95;
 }

 bool isHeadingDown(ArRobot &robot) {
    return robot.getTh() >= -95 && robot.getTh() <= -85; 
 }

 bool isHeadingRight(ArRobot &robot) {
    return robot.getTh() >= -5 && robot.getTh() <= 5;   
 }

 bool isHeadingLeft(ArRobot &robot) {
    return robot.getTh() >= 175 || robot.getTh() <= -175;
 }
 
 /**
  * Monitoring Commands
  */
 void updateRobotCoordinatesRotationSpeed(ArRobot &robot) {
    robot.lock();
    setLatestX(robot.getX());
    setLatestY(robot.getY());
    setLatestRotation(robot.getTh());
    setLatestSpeed(robot.getVel());
    robot.unlock();
 }
 
 void updateSonarReadings(ArRobot *robot) {
    ArRangeDevice *mySonar = robot->findRangeDevice("sonar");
    
    // if the sonar is null we can't do anything, so deactivate
    if (mySonar != NULL) {
       double leftRange, frontRange, rightRange;
      
       // Get the left readings and right readings off of the sonar
       robot->lock();
       leftRange = (mySonar->currentReadingPolar(80, 120) - robot->getRobotRadius());
       frontRange = (mySonar->currentReadingPolar(-15, 15) - robot->getRobotRadius());
       rightRange = (mySonar->currentReadingPolar(-120, -80) - robot->getRobotRadius());
       robot->unlock(); 

       setLatestLeftDistance(leftRange);
       setLatestFrontDistance(frontRange);
       setLatestRightDistance(rightRange);
    }
 }

/**
 * Logs
 */ 
 void logRobotStatus(int stepsCount, ArRobot &robot) {
    // Open log
    robot.lock();
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "Step : %.2d", stepsCount);
    ArLog::log(ArLog::Normal, ".....................................");

    // Log the current status
    ArLog::log(ArLog::Normal, "Robot status: X and Y  : [%.2f,%.2f]", getLatestX(), getLatestY());
    ArLog::log(ArLog::Normal, "Robot status: Rotation : [%.2f]", getLatestRotation());
    ArLog::log(ArLog::Normal, "Robot status: Speed    : [%.2f]", getLatestSpeed());
    
    // Print left and right status
    ArLog::log(ArLog::Normal, "Left range : [%.2f]", getLatestLeftDistance());
    ArLog::log(ArLog::Normal, "Front range : [%.2f]", getLatestFrontDistance());
    ArLog::log(ArLog::Normal, "Right range : [%.2f]", getLatestRightDistance()); 

    // Close log
    ArLog::log(ArLog::Normal, "....................................."); 
    ArLog::log(ArLog::Normal, "");
    robot.unlock();
 }

 void logRobotStatusAndGoal(ArRobot &robot) {
    robot.lock();
  
    // Log the current status of the robot and load the new candidate status
    ArLog::log(ArLog::Normal, "Robot status according to the goal");
    ArLog::log(ArLog::Normal, "Distance from goal: [%.2f]", getDistanceFromGoal());
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "leftRange : [%.2f]", getLatestLeftDistance());
    ArLog::log(ArLog::Normal, "frontRange : [%.2f]", getLatestFrontDistance());
    ArLog::log(ArLog::Normal, "rightRange : [%.2f]", getLatestRightDistance());
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "X : [%.2f]", robot.getX());
    ArLog::log(ArLog::Normal, "Y : [%.2f]", robot.getY());
    ArLog::log(ArLog::Normal, "Rotation : [%.2f]", robot.getTh());
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "getGoalX() : [%.2f]", getGoalX());
    ArLog::log(ArLog::Normal, "getGoalY() : [%.2f]", getGoalY());
    ArLog::log(ArLog::Normal, "");

    robot.unlock();
 }

 /**
  * DFS - Online
  */
 State getGoalState(ArRobot &robot) {
    robot.lock();
    MapBlock mapBlock;
    mapBlock.x1 = getGoalX() - (getGoalX() / 2);
    mapBlock.x2 = getGoalX() + (getGoalX() / 2);
    mapBlock.y1 = getGoalY() - (getGoalY() / 2);
    mapBlock.y2 = getGoalY() + (getGoalY() / 2);
    
    State state;
    state.stepNumber = -1;
    state.mapBlock = mapBlock;
    state.triedActionForward = false;
    state.triedActionLeft = false;
    state.triedActionRight = false;
    state.triedActionBackwards = false;
    state.isForbiddenState = false;
    robot.unlock();
 }

 void deepFirstSearch(ArRobot &robot, State &root, State &goal) {
    // Initialize the system
    State currentState;
    int stepsCount = 0;
    int action = 0;
    
    // Main loop
    while (!hasAchievedGoal()) {   
      // Clear the screen for new logs
      system("clear");

      if (stepsCount == 0) {
         State currentState = updateAgentState(robot, action, stepsCount);
         goForward(robot, 1);
         currentState.triedActionForward = true;
         addStateToPath(Q, currentState);

      } else {
         // Update the status of the hasAchievedGoal() function
         verifyAchievedGoal(robot);
      
         // Update the status of the sensors and the speed of the robot 
         currentState = updateAgentState(robot, action, stepsCount);
            
         // Verify the candidate positions to where the robot can move to
         action = evaluateCandidatesTakeDecision(robot, currentState);
      
         switch (action) {
      
            case ACTION_BACK:
               goBackward(robot, 1);
               backtrackUntilLatestValidPosition(robot, Q, currentState);
               break;
      
            case ACTION_FRONT:
               goForward(robot, 1);
               addStateToPath(Q, currentState);
               break;
      
            case ACTION_LEFT:
               turnLeft(robot);
               goForward(robot, 1);
               addStateToPath(Q, currentState);
               break;
      
            case ACTION_RIGHT: 
               turnRight(robot);
               goForward(robot, 1);
               addStateToPath(Q, currentState);
               break;
      
            default:
               ArUtil::sleep(1000);
               break;
         }
      }

      stepsCount++;
      
      // Log
      //logRobotStatus(stepsCount, robot);  
   }
   // ./Main loop
 }

 void verifyAchievedGoal(ArRobot &robot) {
    robot.lock();
  
    if (abs(robot.getX() - getGoalX()) < MIN_COLISION_RANGE_MM && 
       abs(robot.getY() - getGoalY()) < MIN_COLISION_RANGE_MM) {
       setHasAchievedGoal(true);
       ArLog::log(ArLog::Normal, "\nRobot has achieved goal [:");
    }

    robot.unlock();
 }

 State updateAgentState(ArRobot &robot, int action, int stepsCount) {
    updateRobotCoordinatesRotationSpeed(robot);
    updateDistanceFromGoal();
    updateSonarReadings(&robot);
    logRobotStatusAndGoal(robot);

    robot.lock();
    MapBlock mapBlock;
    mapBlock.x1 = robot.getX();
    mapBlock.x2 = robot.getX() + MIN_COLISION_RANGE_MM;
    mapBlock.y1 = robot.getY();
    mapBlock.y2 = robot.getY() + MIN_COLISION_RANGE_MM;
    
    State state;
    state.stepNumber = stepsCount;
    state.mapBlock = mapBlock;
    state.triedActionForward = false;
    state.triedActionLeft = false;
    state.triedActionRight = false;
    state.triedActionBackwards = false;
    state.isForbiddenState = false;

    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "Current MapBlock");
    ArLog::log(ArLog::Normal, "mapBlock.x1 %.2f", mapBlock.x1);
    ArLog::log(ArLog::Normal, "mapBlock.x2 %.2f", mapBlock.x2);
    ArLog::log(ArLog::Normal, "mapBlock.y1 %.2f", mapBlock.y1);
    ArLog::log(ArLog::Normal, "mapBlock.y1 %.2f", mapBlock.y2);
    ArLog::log(ArLog::Normal, "");
    robot.unlock();

    return state;
 }

 /**
  * This function sense the possible actions that can be taken and chose the one that is going to position the robot the closest to 
  * the goal, the possible action outputs are:
  *  1 - goFoward
  * -1 - goBacward
  *  2 - goLeft
  *  3 - goRight
  */
 int evaluateCandidatesTakeDecision(ArRobot &robot, State &state) {
    robot.lock();
    
    // If the current state is invalid, move back
    if (isForbiddenItem(state)) {
       return ACTION_BACK;
    }

    bool obstacleFront = false;
    bool obstacleLeft = false;
    bool obstacleRight = false;

    if (getLatestFrontDistance() < ROBOT_X_SIZE * 1.5) {
       ArLog::log(ArLog::Normal, "\nObstacle ahead");
       obstacleFront = true;
       state.triedActionForward = true;
    } 

    if (getLatestLeftDistance() < ROBOT_X_SIZE * 1.5) {
       ArLog::log(ArLog::Normal, "\nObstacle on the left");
       obstacleLeft = true;
       state.triedActionLeft = true;
    } 
  
    if (getLatestRightDistance() < ROBOT_X_SIZE * 1.5) {
       ArLog::log(ArLog::Normal, "\nObstacle on the right");
       obstacleRight = true;
       state.triedActionRight = true;
    }
  
    // Take a decision based on the sensor's perception and return the next action
    if (obstacleFront && obstacleLeft && obstacleRight) {
       state.isForbiddenState = true;
       robot.unlock();
       return -1;

    } else {
       float distCandFront = 100000000.0;
       float distCandLeft = 100000000.0;
       float distCandRight = 100000000.0;

       double projFrontCoordX = 0.0;
       double projFrontCoordY = 0.0;
       double projLeftCoordX = 0.0;
       double projLeftCoordY = 0.0;
       double projRightCoordX = 0.0;
       double projRightCoordY = 0.0;
       double projBackCoordX = 0.0;
       double projBackCoordY = 0.0;

       if (!obstacleFront) {
          
          if (isHeadingRight(robot)) {
             projFrontCoordX = (robot.getX() + ((2 * ROBOT_X_SIZE) / 2));
             projFrontCoordY = robot.getY();
             distCandFront = getDistanceAB(robot.getX() + (2 * ROBOT_X_SIZE), robot.getY(), getGoalX(), getGoalY());

          } else if (isHeadingUp(robot)) {
             projFrontCoordX = robot.getX();
             projFrontCoordY = robot.getY() + (ROBOT_X_SIZE / 2);
             distCandFront = getDistanceAB(robot.getX(), robot.getY() + ROBOT_X_SIZE, getGoalX(), getGoalY());

          } else if (isHeadingLeft(robot)) {
             projFrontCoordX = robot.getX() - ((ROBOT_X_SIZE / 2));
             projFrontCoordY = robot.getY();
             distCandFront = getDistanceAB(robot.getX() - ROBOT_X_SIZE, robot.getY(), getGoalX(), getGoalY());

          } else if (isHeadingDown(robot)) { 
             projFrontCoordX = robot.getX();
             projFrontCoordY = robot.getY() - (ROBOT_X_SIZE / 2);
             distCandFront = getDistanceAB(robot.getX(), robot.getY() - (2 * ROBOT_X_SIZE), getGoalX(), getGoalY());
          }
       }

       if (!obstacleLeft) {

          if (isHeadingRight(robot)) {
             projLeftCoordX = robot.getX();
             projLeftCoordY = robot.getY() + (ROBOT_X_SIZE / 2);
             distCandLeft = getDistanceAB(robot.getX(), robot.getY() + ROBOT_X_SIZE, getGoalX(), getGoalY());
             
          } else if (isHeadingUp(robot)) {
             projLeftCoordX = robot.getX() - (ROBOT_X_SIZE / 2);
             projLeftCoordY = robot.getY();
             distCandLeft = getDistanceAB(robot.getX() - ROBOT_X_SIZE, robot.getY(), getGoalX(), getGoalY());

          } else if (isHeadingLeft(robot)) {
             projLeftCoordX = robot.getX();
             projLeftCoordY = robot.getY() - ((2 * ROBOT_X_SIZE) / 2);
             distCandLeft = getDistanceAB(robot.getX(), robot.getY() - (2 * ROBOT_X_SIZE), getGoalX(), getGoalY());

          } else if (isHeadingDown(robot)) { 
             projLeftCoordX = robot.getX() + ((2 * ROBOT_X_SIZE) / 2);
             projLeftCoordY = robot.getY();
             distCandLeft = getDistanceAB(robot.getX() + (2 * ROBOT_X_SIZE), robot.getY(), getGoalX(), getGoalY());
          }
       }

       if (!obstacleRight) {
          
          if (isHeadingRight(robot)) {
             projRightCoordX = robot.getX();
             projRightCoordY = robot.getY() - ((2 * ROBOT_X_SIZE) / 2);
             distCandRight = getDistanceAB(robot.getX(), robot.getY() - (2 * ROBOT_X_SIZE), getGoalX(), getGoalY());
             
          } else if (isHeadingUp(robot)) {
             projRightCoordX = robot.getX() + ((2 * ROBOT_X_SIZE) / 2);
             projRightCoordY = robot.getY();
             distCandRight = getDistanceAB(robot.getX() + (2 * ROBOT_X_SIZE), robot.getY(), getGoalX(), getGoalY());
             
          } else if (isHeadingLeft(robot)) {
             projRightCoordX = robot.getX();
             projRightCoordY = robot.getY() + (ROBOT_X_SIZE / 2);
             distCandRight = getDistanceAB(robot.getX(), robot.getY() + ROBOT_X_SIZE, getGoalX(), getGoalY());
             
          } else if (isHeadingDown(robot)) { 
             projRightCoordX = robot.getX() - (ROBOT_X_SIZE / 2);
             projRightCoordY = robot.getY();
             distCandRight = getDistanceAB(robot.getX() - ROBOT_X_SIZE, robot.getY(), getGoalX(), getGoalY());
          }
       }

       /*
       ArLog::log(ArLog::Normal, "Distance if go front : [%.2f]", distCandFront);
       ArLog::log(ArLog::Normal, "Distance if go left : [%.2f]", distCandLeft);
       ArLog::log(ArLog::Normal, "Distance if go right : [%.2f]", distCandRight);
       */

       if (distCandFront == 100000000.0) {
          state.triedActionForward = true;
       }

       if (distCandLeft == 100000000.0) {
          state.triedActionLeft = true;
       }

       if (distCandRight == 100000000.0) {
          state.triedActionRight = true;
       }

       // If the left candidate is the closest to the goal, Turn Left
       if (!obstacleLeft && distCandLeft < distCandFront && distCandLeft < distCandRight) {
          
          if (!Q.empty()) {
             
             // If the state was added previously, give priority to the projected states
             if (wasPreviouslyVisited(Q, projLeftCoordX, projLeftCoordY)) {
                         
                if (!obstacleFront && distCandFront <= distCandRight && 
                    !wasPreviouslyVisited(Q, projFrontCoordX, projFrontCoordY)) {
                   state.triedActionForward = true;
                   robot.unlock();
                   return ACTION_FRONT;
            
                } else if (!obstacleRight && 
                           !wasPreviouslyVisited(Q, projRightCoordX, projRightCoordY)) {
                   state.triedActionRight = true;
                   robot.unlock();
                   return ACTION_RIGHT;

                } else {
                   state.triedActionBackwards = true;
                   robot.unlock();
                   return ACTION_BACK;  
                }
             } else {
                // This is a new state and it hasn't been added
                state.triedActionLeft = true;
                robot.unlock();
                return ACTION_LEFT;  
             } 
          } else {
             // This the first iteratiction
             state.triedActionLeft = true;
             robot.unlock();
             return ACTION_LEFT;
          }
       }

       // If the front candidate is the closest to the goal, Go Forward
       if (!obstacleFront && distCandFront <= distCandLeft && distCandFront <= distCandRight) {
          
          if (!Q.empty()) {
             
             // If the state was added previously, give priority to the projected states
             if (wasPreviouslyVisited(Q, projFrontCoordX, projFrontCoordY)) {
                       
                if (!obstacleLeft && distCandLeft <= distCandRight && 
                    !wasPreviouslyVisited(Q, projLeftCoordX, projLeftCoordY)) {
                   state.triedActionLeft = true;
                   robot.unlock();
                   return ACTION_LEFT;
          
                } else if (!obstacleRight && 
                           !wasPreviouslyVisited(Q, projRightCoordX, projRightCoordY)) {
                   state.triedActionRight = true;
                   robot.unlock();
                   return ACTION_RIGHT;

                } else {
                   state.triedActionBackwards = true;
                   robot.unlock();
                   return ACTION_BACK;
                }
             } else {
                // This is a new state and it hasn't been added
                state.triedActionForward = true;
                robot.unlock();
                return ACTION_FRONT;
             }
          } else {
             // This the first iteratiction
             state.triedActionForward = true;
             robot.unlock();
             return ACTION_FRONT;
          }
       }

       // If the right candidate is the closest to the goal, Turn Right
       if (!obstacleRight && distCandRight < distCandLeft && distCandRight < distCandFront) {
          
          if (!Q.empty()) {
             
             // If the state was added previously, give priority to the projected states
             if (wasPreviouslyVisited(Q, projRightCoordX, projRightCoordY)) {
                       
                if (!obstacleLeft && distCandLeft <= distCandRight && 
                    !wasPreviouslyVisited(Q, projLeftCoordX, projLeftCoordY)) {
                   state.triedActionLeft = true;
                   robot.unlock();
                   return ACTION_LEFT;
          
                } else if (!obstacleFront && 
                           !wasPreviouslyVisited(Q, projFrontCoordX, projFrontCoordY)) {
                   state.triedActionForward = true;
                   robot.unlock();
                   return ACTION_FRONT;

                } else {
                   state.triedActionBackwards = true;
                   robot.unlock();
                   return ACTION_BACK;
                }
             } else {
                // This is a new state and it hasn't been added
                state.triedActionRight = true;
                robot.unlock();
                return ACTION_RIGHT;
             }
          } else {
             // This the first iteratiction
             state.triedActionRight = true;
             robot.unlock();
             return ACTION_RIGHT;
          }
       }

       state.triedActionBackwards = true;
       robot.unlock();
       return ACTION_BACK;
    }
 }

 double getDistanceAB(float aX, float aY, float bX, float bY) {
    return sqrt(pow(abs(aX - bX), 2) + pow(abs(aY - bY), 2));
 }

 void addStateToPath(stack<State> &path, State &state) {
    path.push(state);
 }

 void backtrackUntilLatestValidPosition(ArRobot &robot, stack<State> &path, State &state) {
    // The current state was previously added as well as the projected states
    bool foundValidPreviousState = false;

    while (!foundValidPreviousState || path.empty()) {
       // 1 - Recover the previous state in the stack and backtrack the stack until the recovered state
       State previousState = path.top();
      
       // 2 - If there is no valid alternative left, backtrack until the previous state
       if (!previousState.triedActionForward && !previousState.triedActionLeft && !previousState.triedActionRight) {
        
          // 3 - Remove the invalid state and add it to the list of forbidden states
          previousState.isForbiddenState = true;
          addToListForbiddenState(previousState);
          path.pop();

          // 4 - Move backwards
          goBackward(robot, 1);

       } else {
          foundValidPreviousState = true;
       }
    }
 }

 void addToListForbiddenState(State &state) {
    bool alreadyAdded = false;
    std::stack<State> forbiddenListCopy = forbiddenStates;
    int forbiddenListItemSize = forbiddenStates.size();
    
    for (int i = 0; i < forbiddenListItemSize; i++) {
      State forbiddenItem = forbiddenStates.top();
      forbiddenStates.pop();

      if (forbiddenItem.mapBlock.isCoordWithinBlock(state.mapBlock.x1 + 100, state.mapBlock.y1 + 100)) {
         alreadyAdded = true;
         break;  
      }
    }

    forbiddenStates = forbiddenListCopy;

    if (!alreadyAdded) {
      forbiddenStates.push(state);
    }
 }

 bool isForbiddenItem(State &state) {
    bool alreadyAdded = false;
    std::stack<State> forbiddenListCopy = forbiddenStates;
    int forbiddenListItemSize = forbiddenStates.size();
    
    for (int i = 0; i < forbiddenListItemSize; i++) {
       State forbiddenItem = forbiddenStates.top();
       forbiddenStates.pop();

       if (forbiddenItem.mapBlock.isCoordWithinBlock(state.mapBlock.x1 + 100, state.mapBlock.y1 + 100)) {
          alreadyAdded = true;
          break;  
       }
    }

    forbiddenStates = forbiddenListCopy;

    return alreadyAdded;
 }

 bool wasPreviouslyVisited(stack<State> &path, double x, double y) {
    stack<State> reversePathOriginGoal;
    int pathSize = path.size();

    // Invert the order of the items putting the first one at the top
    for (int i = 0; i < pathSize; ++i) {
       State state = path.top();
       path.pop();
       reversePathOriginGoal.push(state);
    }

    // Print the coordinates of each valid state of the path from the origin to the goal
    printf("\n--------------------------------------");
    printf("\nCHECKING IF BLOCK WAS PREVIOUSLY ADDED");

    for (int i = 0; i < pathSize; ++i) {
       // Rebuild path from origin to goal
       State state = reversePathOriginGoal.top();
       reversePathOriginGoal.pop();
       path.push(state);
       
       // Print information about each state 
       state.toString();

       if (state.mapBlock.isCoordWithinBlock(x, y)) {
          printf("\n---------------------");
          printf("\nWAS PREVIOUSLY ADDED");
          printf("\nx: %.2f, y: %.2f", x, y);
          printf("\nFINISHED PATH SEARCH FOR PREVIOUS POSITIONS");
          printf("\n-------------------------------------------");

          return true;
       }
    }

    printf("\nFINISHED PATH SEARCH FOR PREVIOUS POSITIONS");
    printf("\n-------------------------------------------");

    return false;
 }

 void printPath(stack<State> &path) {
    stack<State> reversePathOriginGoal;
    int pathSize = path.size();

    // Invert the order of the items putting the first one at the top
    for (int i = 0; i < pathSize; ++i) {
       State state = path.top();
       path.pop();
       reversePathOriginGoal.push(state);
    }

    // Print the coordinates of each valid state of the path from the origin to the goal
    printf("\n.......................");
    printf("\nFINAL PATH FROM ORIGIN [%.2f, %.2f] TO GOAL [%.2f, %.2f] : ", getInitialX(), getInitialY(), getGoalX(), getGoalY());

    for (int i = 0; i < pathSize; ++i) {
       State state = reversePathOriginGoal.top();
       reversePathOriginGoal.pop();
       printf("\nx[%.2f,%.2f] y[%.2f,%.2f]", state.mapBlock.x1, state.mapBlock.x2, state.mapBlock.y1, state.mapBlock.y2);
    }

    printf("\n.......................\n");
 }
