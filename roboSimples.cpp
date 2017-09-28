 #include "Aria.h"

int main(int argc, char **argv)
{

  Aria::init();
  ArRobot robot;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  ArLog::log(ArLog::Terse, "WARNING: this program does no sensing or avoiding of obstacles, the robot WILL collide with any objects in the way! Make sure the robot has approximately 3 meters of free space on all sides.");

  // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
  // and then loads parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::shutdown();
    return 1;
  }
  
  ArLog::log(ArLog::Normal, "Robo Simples: Connected.");

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);


  // Print out some data from the SIP.  

  // We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  // Make sure you unlock before any sleep() call or any other code that will
  // take some time; if the robot remains locked during that time, then
  // ArRobot's background thread will be blocked and unable to communicate with
  // the robot, call tasks, etc.
  
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Will start driving in 3 seconds...");
  ArUtil::sleep(3000);

  // Set forward velocity to 50 mm/s
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 250 mm/s for 5 sec...");
  robot.lock();
  robot.enableMotors();
  ArUtil::sleep(1000);
  //robot.setVel(1000);
  //robot.move(2050);
  robot.setRotVel(45);
  robot.unlock();
  ArUtil::sleep(2000);
  
  /*

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(1000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Rotating at 10 deg/s for 5 sec...");
  robot.lock();
  robot.setVel(550);
  robot.setRotVel(10);
  robot.unlock();
  ArUtil::sleep(5000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Rotating at -10 deg/s for 10 sec...");
  robot.lock();
  robot.setVel(550);
  robot.setRotVel(-10);
  robot.unlock();
  ArUtil::sleep(10000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 150 mm/s for 5 sec...");
  robot.lock();
  robot.setRotVel(0);
  robot.setVel(150);
  robot.unlock();
  ArUtil::sleep(5000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(1000);


  // Other motion command functions include move(), setHeading(),
  // setDeltaHeading().  You can also adjust acceleration and deceleration
  // values used by the robot with setAccel(), setDecel(), setRotAccel(),
  // setRotDecel().  See the ArRobot class documentation for more.

 */ 
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

  
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  return 0;
}