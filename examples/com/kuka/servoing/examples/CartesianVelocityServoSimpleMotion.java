package com.kuka.servoing.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.device.common.JointPosition;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.sensitivity.LBR;
import com.kuka.servoing.api.cartvelocityservo.ECartesianVelocityServoWarning;
import com.kuka.servoing.api.cartvelocityservo.ICartesianVelocityServo;
import com.kuka.servoing.api.cartvelocityservo.ICartesianVelocityServoRuntime;
import com.kuka.servoing.api.common.EServoRequestState;
import com.kuka.servoing.api.common.IServoingCapability;
import com.kuka.statistics.StatisticTimer;
import com.kuka.statistics.StatisticTimer.OneTimeStep;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;
import java.util.List;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;

/**
 * A simple example how to instantiate, parameterize and execute a CartesianVelocityServo motion.
 * WARNING: Make sure that the area around the robot is clear of any obstacles and no tool is mounted that
 * could potentially collide with the robot.
 */
public class CartesianVelocityServoSimpleMotion extends RoboticsAPIApplication
{
  @Inject
  private LBR _robot;

  @Inject
  private IServoingCapability _servoingCapability;

  private ITaskLogger _logger;
  private int _steps = 0;

  private static final int NUM_RUNS = 600;
  private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;

  @Override
  public void initialize()
  {
    _logger = getLogger();
  }

  @Override
  public void run()
  {

    // Move to initial position
    _robot.getFlange().move(
        ptp(JointPosition.ofDeg(70, 40, -10, -60, 90, -60, -70)).setJointVelocityRel(0.1));
    
    // uncomment to start in a singular configuration
    //_robot.getFlange().move(
    //ptp(JointPosition.ofDeg(70, 0, 0, -90, 90, 0, -70)).setJointVelocityRel(0.1));

    // Create a CartesianVelocityServo motion
    // By default the servo will stay active 30 seconds to wait for a new target velocity
    // This timing can be configured by the user
    ICartesianVelocityServo cartesianVelocityServo = _servoingCapability.createCartesianVelocityServoMotion();

    // enable joint limit compensation, you can also try disabling it to see the difference in the motion
    cartesianVelocityServo.enableJointLimitCompensation(true);
    // reduce the max. joint speeds
    cartesianVelocityServo.setMaxJointSpeed(new double[] { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 });
    // reduce the max. translational acceleration to achieve a smoother motion
    cartesianVelocityServo.setMaxTranslationAcceleration(500);

    _logger.info("Starting CartesianVelocityServo motion in position control mode");
    _robot.getFlange().moveAsync(cartesianVelocityServo);

    _logger.info("Get the runtime of the CartesianVelocityServo motion");
    ICartesianVelocityServoRuntime servoRuntime = cartesianVelocityServo.getRuntime(500, TimeUnit.MILLISECONDS);

    // Update the runtime information with the latest robot status
    EServoRequestState result = servoRuntime.updateWithRealtimeSystem();
    if (EServoRequestState.SUCCESSFUL != result)
    {
      _logger.error("updateWithRealtimeSystem request failed, with result: " + result);
      return;
    }
    
    //check distance to singularity
    if (servoRuntime.getDistanceToSingularity() == 0)
    {
        _logger.warn("Too close to singularity. Robot will not move!");
    }
    
    // For roundtrip time measurement...
    final StatisticTimer timing = new StatisticTimer();

    //initial target velocity in mm/s and rad/s
    double[] targetVelocity = { 300, 0, -500, 0, 0, 0 };

    // Do a cyclic loop
    _logger.info("Start loop");
    for (_steps = 0; _steps < NUM_RUNS; ++_steps)
    {
      // Timing - draw one step
      final OneTimeStep singleStep = timing.newTimeStep();

      ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

      //change the target velocity every 100 iterations
      if (Math.floorMod(_steps, 100) == 0)
      {
        targetVelocity[0] = -targetVelocity[0];
      }
      
      //print list of warnings every 200 iterations
      if (Math.floorMod(_steps, 200) == 0)
      {
        List<ECartesianVelocityServoWarning> warningList = servoRuntime.getMotionWarningList();
        if (!warningList.isEmpty())
        {
          _logger.warn("Warnings: " + warningList);
        }
      }

      result = servoRuntime.setCartesianVelocity(targetVelocity);

      if (EServoRequestState.SUCCESSFUL != result)
      {
        _logger.error("setCartesianVelocity request failed, with result: " + result);
        return;
      }

      singleStep.end();

    }
    ThreadUtil.milliSleep(1000);

    // Print statistics and parameters of the motion
    _logger.info("Displaying final states after loop ");
    _logger.info(getClass().getName() + servoRuntime.toString());

    // Request to stop the motion
    _logger.info("Send request to stop the CartesianVelocityServo motion");
    servoRuntime.stopMotion();

    _logger.info("StatisticTimer of overall Loop " + timing);
    if (servoRuntime.getTimingSetNewCommand().getMean() > 100)
    {
      _logger.info(
          "StatisticTimer is unexpectedly slow, you should try to optimize TCP/IP Transfer");
      _logger.info(
          "Under Windows, you should change the registry settings, see the e.g. user manual");
    }
  }
}
