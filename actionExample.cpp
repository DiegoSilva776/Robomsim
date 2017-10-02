/**
 * actionExample.cpp
 *
 * Authors: Diego M. Silva, diegomsilva.com
 * Licese: MIT
 */
 #include "Aria.h"
 #include <ArRecurrentTask.h>
 #include <string>
 #include <iostream>
 #include <thread>
 
 using namespace std;
 
 /**
  * DATA MANIPULATION VARIABLES
  */
 bool mHasAchievedGoal = false;

 float mLatestX = 0.0;
 float mLatestY = 0.0;
 float mLatestRot = 0.0;
 float mLatestSpeed = 0.0;

 double mLeftDistance = 0.0;
 double mFrontDistance = 0.0;
 double mRightDistance = 0.0;

 
 /**
  * FUNCTIONS' DECLARATION
  */ 
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
 void goForward(ArRobot &robot, int distanceMM);
 void goBackward(ArRobot &robot, int numBlocks);
 void turnLeft(ArRobot &robot);
 void turnRight(ArRobot &robot);
 void turnBackwards(ArRobot &robot);
 void stop(ArRobot &robot);
 
 /**
  * Monitoring Commands
  */
 void updateRobotCoordinatesRotationSpeed(ArRobot &robot);
 void updateSonarReadings(ArRobot *robot);
 
 /**
  * DFS - Online
  */
 int evaluateCandidatesTakeDecision(ArRobot &robot);

 /**
  * Log
  */
 void logRobotStatus(int stepsCount, ArRobot &robot);
 void logSimpleMsg(ArRobot &robot, const char *msg);

 /**
  * MAIN
  */
 int main(int argc, char **argv) {
    // Init the robot and try to connect to it
    Aria::init();
 
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobot robot;
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
   
    // AddDevices
    // Sonar
    ArSonarDevice sonar;
    robot.addRangeDevice(&sonar);
 
    // Turn the robot on
    robot.enableMotors();
    robot.runAsync(true);

    // Start by moving forward
    int stepsCount = 0;
    int action = 0;
    
    // Main program
    while (!hasAchievedGoal()) {
       // Verify the candidate positions to where the robot can move to
       action = evaluateCandidatesTakeDecision(robot);

       switch (action) {

           case -1:
              goBackward(robot, 1);
              break;

           case 1:
              goForward(robot, 1);
              break;

           case 2: 
              turnLeft(robot);
              break;

           case 3: 
              turnRight(robot);
              break;
  
       }

       // Update the status of the sensors and the speed of the robot 
       updateRobotCoordinatesRotationSpeed(robot);
       updateSonarReadings(&robot);

       stepsCount++;

       // Log
       logRobotStatus(stepsCount, robot);
    }
    
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
  * Setters & Getters
  */
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
 void goForward(ArRobot &robot, int numBlocks) {
    const int BASE_BLOCK_VEL_MM_SEC = 300;
    const int BASE_BLOCK_TIME_MOTION_SEC = 2400;
   
    for (int i = 0; i < numBlocks; i++) {
       robot.lock();
       ArLog::log(ArLog::Normal, "Moving forward, block : (%.d/%.d)", i + 1, numBlocks);
       robot.unlock();
 
       robot.lock();
       robot.setRotVel(0);
       robot.setVel(BASE_BLOCK_VEL_MM_SEC);
       robot.unlock();
       ArUtil::sleep(BASE_BLOCK_TIME_MOTION_SEC);
 
       stop(robot);
    }
 }
 
 void goBackward(ArRobot &robot, int numBlocks) {
   const int BASE_BLOCK_VEL_MM_SEC = -300;
   const int BASE_BLOCK_TIME_MOTION_SEC = 2350;
 
   for (int i = 0; i < numBlocks; i++) {
      robot.lock();
      ArLog::log(ArLog::Normal, "Moving backward, block : (%.d/%.d)", i + 1, numBlocks);
      robot.unlock();
 
      robot.lock();
      robot.setRotVel(0);
      robot.setVel(BASE_BLOCK_VEL_MM_SEC);
      robot.unlock();
      ArUtil::sleep(BASE_BLOCK_TIME_MOTION_SEC);
 
      stop(robot);
   }
 }
 
 void turnLeft(ArRobot &robot) {
    robot.lock();
    ArLog::log(ArLog::Normal, "Turning left ...");
    robot.unlock();
 
    robot.lock(); 
    robot.setRotVel(90);
    robot.unlock();
    ArUtil::sleep(1020);
 
    stop(robot);
 }
 
 void turnRight(ArRobot &robot) {
    robot.lock();
    ArLog::log(ArLog::Normal, "Turning right ...");
    robot.unlock();
 
    robot.lock();
    robot.setRotVel(-90);
    robot.unlock();
    ArUtil::sleep(1030);
 
    stop(robot);
 }
 
 void turnBackwards(ArRobot &robot) {
    robot.lock();
    ArLog::log(ArLog::Normal, "Turning backwards ...");
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
       leftRange = (mySonar->currentReadingPolar(80, 100) - robot->getRobotRadius());
       frontRange = (mySonar->currentReadingPolar(0, 10) - robot->getRobotRadius());
       rightRange = (mySonar->currentReadingPolar(-100, -80) - robot->getRobotRadius());
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

 /**
  * DFS - Online
  * 
  * Possible actions:
  *  1 - goFoward
  * -1 - goBacward
  *  2 - goLeft
  *  3 - goRight
  */
 int evaluateCandidatesTakeDecision(ArRobot &robot) {
  float MIN_COLISION_RANGE_MM = 400;
  bool obstacleFront = false;
  bool obstacleLeft = false;
  bool obstacleRight = false;
 
  // Log the current status of the robot and load the new candidate status
  robot.lock();
  ArLog::log(ArLog::Normal, "leftRange : [%.2f]", getLatestLeftDistance());
  ArLog::log(ArLog::Normal, "frontRange : [%.2f]", getLatestFrontDistance());
  ArLog::log(ArLog::Normal, "rightRange : [%.2f]", getLatestRightDistance());
  ArLog::log(ArLog::Normal, "X : [%.2f]", robot.getX());
  ArLog::log(ArLog::Normal, "Y : [%.2f]", robot.getY());
  ArLog::log(ArLog::Normal, "Rotation : [%.2f]", robot.getTh());
  
  if (getLatestFrontDistance() < MIN_COLISION_RANGE_MM) {
     ArLog::log(ArLog::Normal, "Obstacle ahead");
     obstacleFront = true;
  } 

  if (getLatestLeftDistance() < MIN_COLISION_RANGE_MM) {
     ArLog::log(ArLog::Normal, "Obstacle on the left");
     obstacleLeft = true;
  } 
  
  if (getLatestRightDistance() < MIN_COLISION_RANGE_MM) {
     ArLog::log(ArLog::Normal, "Obstacle on the right");
     obstacleRight = true;
  }
  robot.unlock();

  // Take a decision based on the sensor's perception and return the next action
  if (obstacleFront && obstacleLeft && obstacleRight) {
     // Go Backward
     return -1;

  } else {

    if (obstacleFront) {
      
       if (obstacleRight && !obstacleLeft) {
          return 2;
       }

       if (obstacleLeft && !obstacleRight) {
          return 3;
       }

       if (!obstacleLeft && !obstacleRight) {
          return 2;
       }

       return -1;
    } else {
       return 1;
    }

    /* TODO: build the decision upon the smallest distance from a valid candidate to the goal
    float distCandFront = 0.0;
    float distCandLeft = 0.0;
    float distCandRight = 0.0;
    
    if (!obstacleFront) {
       distCandFront = 
    }

    if (!obstacleLeft) {
       distCandLeft = 
    }

    if (!obstacleRight) {
       distCandRight = 
    }
    ./TODO*/
  }
  
}
