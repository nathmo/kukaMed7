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
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartDOF;
import com.kuka.sensitivity.LBR;
import com.kuka.sensitivity.controlmode.CartesianImpedanceControlMode;
import com.kuka.servoing.api.common.EServoRequestState;
import com.kuka.servoing.api.common.IServoingCapability;
import com.kuka.servoing.api.smartservolin.ISmartServoLin;
import com.kuka.servoing.api.smartservolin.ISmartServoLinRuntime;
import com.kuka.statistics.StatisticTimer;
import com.kuka.statistics.StatisticTimer.OneTimeStep;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;

/**
 * This example activates a SmartServoLin motion in Cartesian impedance control mode, sends a
 * sequence of Cartesian destinations, describing a sine function in z-direction and modifies
 * compliance parameters during the motion.
 */
public class SmartServoLinInteractionControl extends RoboticsAPIApplication {
  @Inject private LBR _lbr;

  @Inject private IServoingCapability _servoingCapability;

  private Tool _tool;
  private ITaskLogger _logger;

  // Tool Data
  private static final String TOOL_FRAME = "toolFrame";
  private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
  private static final double MASS = 0;
  private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

  private static final int NUM_RUNS = 600;
  private static final double AMPLITUDE = 70;
  private static final double FREQUENCY = 0.6;

  private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
  private static final double[] MAX_TRANSLATION_VELOCITY = { 150, 150, 150 };

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
    _tool.attachTo(_lbr.getFlange());
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

    // Create a SmartServoLin motion to the current position
    // By default the servo will stay active 30 seconds to wait for a new target destination
    // This timing can be configured by the user
    LocalFrame initialPosition = _lbr.getCurrentCartesianPosition(_tool.getDefaultMotionFrame());
    ISmartServoLin smartServoLinMotion =
        _servoingCapability.createSmartServoLinMotion(initialPosition);

    smartServoLinMotion.setMaxTranslationVelocity(MAX_TRANSLATION_VELOCITY);
    smartServoLinMotion.setMinimumTrajectoryExecutionTime(20e-3);

    // Validate load data
    performLoadValidation(smartServoLinMotion);
    if (!smartServoLinMotion.isValidatedForImpedanceMode()) {
      _logger.error("LoadData validation failed, skipping example!");
      return;
    }

    // Prepare CartesianImpedanceControlMode
    CartesianImpedanceControlMode cartesianImpedance = createCartImp();

    _logger.info("Starting the SmartServoLin in Cartesian impedance control mode");
    _tool.moveAsync(smartServoLinMotion.setMode(cartesianImpedance));

    _logger.info("Get the runtime of the SmartServoLin motion");
    ISmartServoLinRuntime servoRuntime =
        smartServoLinMotion.getRuntime(500, TimeUnit.MILLISECONDS);

    StatisticTimer timing = new StatisticTimer();

    // Start the smart servo lin sine movement
    getLogger().info("Do sine movement");
    Frame frame = servoRuntime.getCommandedCartesianPosition(_tool.getDefaultMotionFrame());
    double omega = FREQUENCY * 2 * Math.PI * 1e-9;
    long startTimeStamp = System.nanoTime();
    for (int i = 0; i < NUM_RUNS; ++i) {
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

      // Modify the stiffness every 60th loop
      if (i % (NUM_RUNS / 10) == 0) {
        // NOTE: DONT CHANGE THE SETTINGS TOO QUICKLY, ELSE YOU WILL DESTABILIZE THE CONTROLLER
        final double aTransStiffVal = Math.max(100.0 * (i / (double) NUM_RUNS + 1), 1000.0);
        final double aRotStiffVal = Math.max(10.0 * (i / (double) NUM_RUNS + 1), 150.0);
        cartesianImpedance.parametrize(CartDOF.TRANSL).setStiffness(aTransStiffVal);
        cartesianImpedance.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);

        // Send the new stiffness to the controller
        result = servoRuntime.changeControlModeSettings(cartesianImpedance);
        if (EServoRequestState.SUCCESSFUL != result) {
          _logger.error("changeControlMode request failed, with result: " + result);
          return;
        }
      }

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
    
    // Request to stop the motion
    _logger.info("Send request to stop the SmartServoLin motion");
    servoRuntime.stopMotion();

    // Print statistics and parameters of the motion
    _logger.info("Displaying final states after loop ");
    _logger.info(getClass().getName() + servoRuntime.toString());

    _logger.info("StatisticTimer of overall loop " + timing);
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
   *        the SmartServoLin motion to be validated
   */
  public void performLoadValidation(ISmartServoLin servoMotion) {
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
   * Create the CartesianImpedanceControlMode class for motion parameterization.
   * 
   * @see CartesianImpedanceControlMode
   * @return the created control mode
   */
  public CartesianImpedanceControlMode createCartImp() {
    final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
    cartImp.parametrize(CartDOF.Z).setStiffness(800.0);
    // For your own safety, shrink the motion abilities to useful limits
    cartImp.setMaxPathDeviation(50.0, 50.0, 50.0, 50.0, 50.0, 50.0);
    return cartImp;
  }

}
