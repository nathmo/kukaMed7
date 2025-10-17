package com.kuka.servoing.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.device.common.JointPosition;
import com.kuka.geometry.Frame;
import com.kuka.geometry.LoadData;
import com.kuka.geometry.LocalFrame;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.Tool;
import com.kuka.math.geometry.Vector3D;
import com.kuka.math.geometry.XyzAbcTransformation;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.sensitivity.LBR;
import com.kuka.servoing.api.common.EServoRequestState;
import com.kuka.servoing.api.common.IServoingCapability;
import com.kuka.servoing.api.smartservo.ISmartServo;
import com.kuka.servoing.api.smartservo.ISmartServoRuntime;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;

import java.util.concurrent.TimeUnit;
import javax.inject.Inject;

/**
 * This example activates a SmartServo motion in position control mode, sends a sequence of
 * Cartesian destinations describing a sine function.
 * While not strictly required for position control mode, this example shows also how to perform the
 * load data validation, enabling to use impedance control modes with Servo motions.
 */
public class SmartServoSimpleCartesianMotion extends RoboticsAPIApplication {
  @Inject private LBR _robot;

  @Inject private IServoingCapability _servoingCapability;

  private ITaskLogger _logger;
  private boolean _doDebugPrints = false;

  // Tool Data
  private Tool _tool;
  private static final String TOOL_FRAME = "toolFrame";
  private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
  private static final double MASS = 0;
  private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

  private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
  private static final int NUM_RUNS = 1000;
  private static final double AMPLITUDE = 70;
  private static final double FREQUENCY = 0.6;

  @Override
  public void initialize() {
    _logger = getLogger();

    // Create a Tool
    // This is the tool we want to move with some mass properties and a TCP-Z-offset of 100.
    LoadData loadData = new LoadData();
    loadData.setMass(MASS);
    loadData.setCenterOfMass(CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
        CENTER_OF_MASS_IN_MILLIMETER[2]);
    _tool = new Tool("Tool", loadData);

    XyzAbcTransformation trans = XyzAbcTransformation.of(TRANSLATION_OF_TOOL[0],
        TRANSLATION_OF_TOOL[1], TRANSLATION_OF_TOOL[2]);
    ObjectFrame toolTransformation = _tool.createFrame(TOOL_FRAME, trans);
    _tool.setDefaultMotionFrame(toolTransformation);
    // Attach tool to the robot
    _tool.attachTo(_robot.getFlange());
  }

  @Override
  public void dispose() {
    // Remove tool at the end of the application
    _tool.detach();
  }

  @Override
  public void run() {
    // Move to initial position
    // WARNING: make sure that the pose is collision free.
    _tool.move(
        ptp(JointPosition.ofDeg(0.0, 30.0, 0.0, -60.0, 0.0, 90.0, 0.0)).setJointVelocityRel(0.1));

    // Create a SmartServo motion to the current position
    // By default the servo will stay active 30 seconds to wait for a new target destination
    // This timing can be configured by the user
    JointPosition initialPosition = _robot.getCurrentJointPosition();
    ISmartServo smartServoMotion = _servoingCapability.createSmartServoMotion(initialPosition);

    smartServoMotion.setMinimumTrajectoryExecutionTime(5e-3);

    // Validate the load configuration at the current position
    performLoadValidation(smartServoMotion);

    _logger.info("Starting SmartServo motion in position control mode");
    _tool.moveAsync(smartServoMotion);

    _logger.info("Get the runtime of the SmartServo motion");
    ISmartServoRuntime servoRuntime = smartServoMotion.getRuntime(500, TimeUnit.MILLISECONDS);

    Frame frame = servoRuntime.getCommandedCartesianPosition(_tool.getDefaultMotionFrame());
    double omega = FREQUENCY * 2 * Math.PI * 1e-9;
    long startTimeStamp = System.nanoTime();
    for (int i = 0; i < NUM_RUNS; ++i) {
      // ///////////////////////////////////////////////////////
      // Insert your code here
      // e.g Visual Servoing or the like
      /////////////////////////////////////////////////////////
      ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

      // Update the runtime information with the latest robot status
      EServoRequestState result = servoRuntime.updateWithRealtimeSystem();
      if (EServoRequestState.SUCCESSFUL != result) {
        _logger.error("updateWithRealtimeSystem request failed, with result: " + result);
        return;
      }

      // Compute the target pose
      double curTime = System.nanoTime() - startTimeStamp;
      double sinArgument = omega * curTime;

      Frame destFrame = LocalFrame.copyOfWithRedundancy(frame, frame.getParent());
      double offset = AMPLITUDE * Math.sin(sinArgument);
      destFrame.translate(Vector3D.of(0, 0, offset));

      // Set new destination
      result = servoRuntime.setDestination(destFrame);
      if (EServoRequestState.SUCCESSFUL != result) {
        _logger.error("setDestination request failed, with result: " + result);
        return;
      }

      printDebugData(servoRuntime);
    }

    // Print statistics and parameters of the motion
    _logger.info("Displaying final states after loop ");
    _logger.info(getClass().getName() + servoRuntime.toString());

    // Request to stop the motion
    _logger.info("Send request to stop the SmartServo motion");
    servoRuntime.stopMotion();
  }

  /**
   * Try to perform the load data validation at the current joint position. This validation
   * confirms that, at this moment, the LoadData parameterization is acceptable. This
   * does not mean that the LoadData is going to be valid for the lifetime of this program.
   * 
   * @param servoMotion
   *        the SmartServo motion to be validated
   */
  public void performLoadValidation(ISmartServo servoMotion) {
    try {
      servoMotion.validateForImpedanceMode(_tool);
    } catch (IllegalStateException e) {
      _logger.error(
          "LoadData Validation for Servoing - correct your mass and center of mass settings!");
      _logger.warn(
          "Servoing will be available for PositionControlMode only, until validation is performed");
      _logger.info("Detail: " + e.getMessage());
    }
  }

  /**
   * Print current system information, if the debug printing is enabled.
   * 
   * @param runtime current SmartServo runtime
   */
  public void printDebugData(ISmartServoRuntime runtime) {
    if (_doDebugPrints) {
      // Get the measured cartesian pose (robot root reference)
      Frame msrPose = runtime.getCurrentCartesianPosition(_tool.getDefaultMotionFrame());

      _logger.info("Current joint destination " + runtime.getCurrentJointDestination());
      _logger.info("LBR position "
          + _robot.getCurrentCartesianPosition(_robot.getFlange(), _robot.getRootFrame()));
      _logger.info("Measured cartesian pose from runtime " + msrPose);
    }
  }
}
