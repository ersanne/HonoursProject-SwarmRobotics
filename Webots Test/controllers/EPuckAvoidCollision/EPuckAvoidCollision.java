// File:          EPuckAvoidCollision.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class EPuckAvoidCollision {

  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {

    //int timeStep = 64;    
    double MAX_SPEED = 6.28;

    // create the Robot instance.
    Robot robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor motor = robot.getMotor("motorname");
    //  DistanceSensor ds = robot.getDistanceSensor("dsname");
    //  ds.enable(timeStep);

    // initialize devices
    DistanceSensor[] ps = new DistanceSensor[8];
    String[] psNames = {
      "ps0", "ps1", "ps2", "ps3",
      "ps4", "ps5", "ps6", "ps7"
    };

    for (int i = 0; i < 8; i++) {
      ps[i] = robot.getDistanceSensor(psNames[i]);
      ps[i].enable(timeStep);
    }

    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      //  double val = ds.getValue();

      // Process sensor data here.

      // Enter here functions to send actuator commands, like:
      //  motor.setPosition(10.0);
      
      
      
      
      // read sensors outputs
      double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
      for (int i = 0; i < 8 ; i++)
        psValues[i] = ps[i].getValue();

      // detect obstacles
      boolean right_obstacle =
        psValues[0] > 70.0 ||
        psValues[1] > 70.0 ||
        psValues[2] > 70.0;
      boolean left_obstacle =
        psValues[5] > 70.0 ||
        psValues[6] > 70.0 ||
        psValues[7] > 70.0;

      // initialize motor speeds at 50% of MAX_SPEED.
      double leftSpeed  = 0.5 * MAX_SPEED;
      double rightSpeed = 0.5 * MAX_SPEED;
      // modify speeds according to obstacles
      if (left_obstacle) {
        // turn right
        leftSpeed  += 0.5 * MAX_SPEED;
        rightSpeed -= 0.5 * MAX_SPEED;
      }
      else if (right_obstacle) {
        // turn left
        leftSpeed  -= 0.5 * MAX_SPEED;
        rightSpeed += 0.5 * MAX_SPEED;
      }
      // write actuators inputs
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
    };

    // Enter here exit cleanup code.
  }
}
