package com.kuka.servoing.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.device.common.JointPosition;
import com.kuka.device.common.JointPositionBuilder;
import com.kuka.geometry.LoadData;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.Tool;
import com.kuka.math.geometry.XyzAbcTransformation;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.sensitivity.LBR;
import com.kuka.servoing.api.common.EServoRequestState;
import com.kuka.servoing.api.common.IServoingCapability;
import com.kuka.servoing.api.smartservo.ISmartServo;
import com.kuka.servoing.api.smartservo.ISmartServoRuntime;
import com.kuka.statistics.StatisticTimer;
import com.kuka.statistics.StatisticTimer.OneTimeStep;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;

import java.util.concurrent.TimeUnit;
import javax.inject.Inject;

/**
 * This example activates a SmartServo motion in position control mode, sends a sequence of joint
 * destinations describing a sine function and evaluates the statistic timing.
 * While not strictly required for position control mode, this example shows also how to perform the
 * load data validation, enabling to use impedance control modes with Servo motions.
 */
public class SmartServoSimpleJointMotion extends RoboticsAPIApplication {
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

  private int _count = 0;

  private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
  private static final int NUM_RUNS = 1000;
  private static final double AMPLITUDE = 0.2;
  private static final double FREQUENCY = 0.1;
  private int _steps = 0;

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

    // Set the motion properties to 20% of systems abilities
    smartServoMotion.setJointAccelerationRel(0.2);
    smartServoMotion.setJointVelocityRel(0.2);

    smartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

    // Validate the load configuration at the current position
    performLoadValidation(smartServoMotion);

    _logger.info("Starting SmartServo motion in position control mode");
    _tool.moveAsync(smartServoMotion);

    _logger.info("Get the runtime of the SmartServo motion");
    ISmartServoRuntime servoRuntime = smartServoMotion.getRuntime(500, TimeUnit.MILLISECONDS);

    // Create a JointPositionBuilder instance to play with
    JointPositionBuilder destinationBuilder = new JointPositionBuilder(_robot.getJointCount());

    // For roundtrip time measurement...
    final StatisticTimer timing = new StatisticTimer();

    // Do a cyclic loop
    // Refer to some timing in nanosec
    _logger.info("Start loop");
    final double omega = FREQUENCY * 2 * Math.PI * 1e-9;
    final long startTimeStamp = System.nanoTime();
    for (_steps = 0; _steps < NUM_RUNS; ++_steps) {
      // Timing - draw one step
      final OneTimeStep singleStep = timing.newTimeStep();
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

      // Prepare new destination
      long curTime = System.nanoTime() - startTimeStamp;
      double sinArgument = omega * curTime;
      for (int k = 0; k < _robot.getJointCount(); ++k) {
        destinationBuilder.set(k, Math.sin(sinArgument) * AMPLITUDE + initialPosition.get(k));
        if (k > 5) {
          destinationBuilder.set(k, initialPosition.get(k));
        }
      }
      result = servoRuntime.setDestination(destinationBuilder.build());
      if (EServoRequestState.SUCCESSFUL != result) {
        _logger.error("setDestination request failed, with result: " + result);
        return;
      }

      // Print information if enabled
      printDebugData(servoRuntime);
      singleStep.end();

    }
    
    // Wait until the last target is reached
    _logger.info("Wait until the last set destination has been reached");
    while(!servoRuntime.isDestinationReached()) {
      ThreadUtil.milliSleep(10);
      EServoRequestState result = servoRuntime.updateWithRealtimeSystem();
      if (EServoRequestState.SUCCESSFUL != result) {
        _logger.error("updateWithRealtimeSystem request failed, with result: " + result);
        return;
      }        
    }
    // Print statistics and parameters of the motion
    _logger.info("Displaying final states after loop ");
    _logger.info(getClass().getName() + servoRuntime.toString());

    // Request to stop the motion
    _logger.info("Send request to stop the SmartServo motion");
    servoRuntime.stopMotion();

    _logger.info(_count + " times was the destination reached.");
    _logger.info("StatisticTimer of overall Loop " + timing);
    if (servoRuntime.getTimingSetNewCommand().getMean() > 100) {
      _logger.info(
          "StatisticTimer is unexpectedly slow, you should try to optimize TCP/IP Transfer");
      _logger
          .info("Under Windows, you should change the registry settings, see the e.g. user manual");
    }
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
   * @param runtime the SmartServo runtime
   */
  public void printDebugData(ISmartServoRuntime runtime) {
    if (_doDebugPrints) {
      // Get the current target destination
      JointPosition newGoal = runtime.getCurrentJointDestination();

      getLogger().info("Step " + _steps + " New Goal " + newGoal);
      getLogger().info("Fine ipo finished " + runtime.isDestinationReached());
      if (runtime.isDestinationReached()) {
        _count++;
      }
      getLogger().info("Ipo state " + runtime.getFineIpoState());
      getLogger().info("Remaining time " + runtime.getRemainingTime());
      getLogger().info("LBR Position " + _robot.getCurrentJointPosition());

      // Get the measured position
      JointPosition curMsrJntPose = runtime.getMeasuredJointPosition();
      getLogger().info("Measured LBR Position " + curMsrJntPose);

      if ((_steps % 100) == 0) {
        // Some internal values, which can be displayed
        getLogger().info("Simple Joint Test - step " + _steps + runtime.toString());
      }
    }
  }
}
