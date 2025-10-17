package com.kuka.servoing.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.device.common.JointPosition;
import com.kuka.geometry.World;
import com.kuka.math.geometry.Rotation;
import com.kuka.math.geometry.Transformation;
import com.kuka.math.geometry.Vector3D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.sensitivity.LBR;
import com.kuka.servoing.api.cartvelocityservo.ICartesianVelocityServo;
import com.kuka.servoing.api.cartvelocityservo.ICartesianVelocityServoRuntime;
import com.kuka.servoing.api.common.EServoRequestState;
import com.kuka.servoing.api.common.IServoingCapability;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;

/**
 * This example demonstrates how a simple proportional controller could be realized to reach a desired pose, 
 * using the CartesianVelocityServo motion. Due to the soft real time interface, such a control loop should only
 * be done with reduced velocity and acceleration to achieve a smooth motion.
 * WARNING: Make sure that the area around the robot is clear of any obstacles and no tool is mounted that
 * could potentially collide with the robot. 
 */
public class CartesianVelocityServoSimpleProportionalControl extends RoboticsAPIApplication {
  @Inject private LBR _robot;

  @Inject private IServoingCapability _servoingCapability;

  @Inject private World _world;
  
  private ITaskLogger _logger;

  // list of desired poses, stored as transformations in the robot root coordinate system
  private List<Transformation> _targetList = new ArrayList<>();

  // transformation from world to robot root coordinate system
  private Transformation _worldToRobRoot;

  private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 10;

  @Override
  public void initialize() {
    _logger = getLogger();
    _worldToRobRoot = _world.getRootFrame().calculateTransformationTo(_robot.getRootFrame());
  }

  @Override
  public void run() {

    // move to initial position
    _robot.getFlange().move(
        ptp(JointPosition.ofDeg(-90, 90.0, 90.0, -90.0, -90.0, 90.0, 0.0)).setJointVelocityRel(0.1));

    prepareTestTargets();
    
    ICartesianVelocityServo cartesianVelocityServo = setupCartesianVelocityServo();

    _logger.info("Starting CartesianVelocityServo motion in position control mode");
    _robot.getFlange().moveAsync(cartesianVelocityServo);
    ICartesianVelocityServoRuntime servoRuntime = cartesianVelocityServo.getRuntime(500, TimeUnit.MILLISECONDS);

    _logger.info("Start loop");
    for (Transformation currentTarget : _targetList) {
      // loop through all target poses, approaching them using a simple proportional controller

      // time before we switch to the next target pose
      double cycleTimeMilli = 20000;

      long timeMilliInit = System.currentTimeMillis();
      long deltaTimeMilli = System.currentTimeMillis() - timeMilliInit;
      while (deltaTimeMilli < cycleTimeMilli)
      {
        ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

        // set the desired Cartesian velocity, given the current target pose
        EServoRequestState result = servoRuntime.setCartesianVelocity(getTargetVelocity(currentTarget));
        if (EServoRequestState.SUCCESSFUL != result) {
          _logger.error("setCartesianVelocity request failed, with result: " + result);
          return;
        }

        // update the runtime information with the latest robot status
        result = servoRuntime.updateWithRealtimeSystem();
        if (EServoRequestState.SUCCESSFUL != result) {
          _logger.error("updateWithRealtimeSystem request failed, with result: " + result);
          return;
        }

        deltaTimeMilli = System.currentTimeMillis() - timeMilliInit;
      }
    }

    // Request to stop the motion
    _logger.info("Send request to stop the CartesianVelocityServo motion");
    servoRuntime.stopMotion();
  }

  private ICartesianVelocityServo setupCartesianVelocityServo() {
    // Create a CartesianVelocityServo motion
    // By default the servo will stay active 30 seconds to wait for a new target velocity
    // This timing can be configured by the user
    ICartesianVelocityServo cartesianVelocityServo = _servoingCapability.createCartesianVelocityServoMotion();

    // set robot root coordinate system as fixed reference
    cartesianVelocityServo.setFixedReference(_worldToRobRoot.getRotation());
    // reduce the max. Cartesian speeds and accelerations to achieve a smooth motion with the soft real time
    // control loop. These values should be further tuned, depending on the application requirements.
    cartesianVelocityServo.setMaxTranslationSpeed(30);
    cartesianVelocityServo.setMaxRotationSpeed(Math.toRadians(5));
    cartesianVelocityServo.setMaxTranslationAcceleration(100);
    cartesianVelocityServo.setMaxRotationAcceleration(Math.toRadians(30));

    // enable joint limit compensation to extend, note that the first pose could not be reached if this
    // option is disabled
    cartesianVelocityServo.enableJointLimitCompensation(true);
    // reduce the max. allowed joint speeds
    cartesianVelocityServo.setMaxJointSpeed(new double[]{0.5,0.5,0.5,0.5,0.5,0.5,0.5});
    
    return cartesianVelocityServo;
  }

  private void prepareTestTargets()
  {
    // the target positions are the corners of a square with this side length in mm
    double sideLength = 450;
    // the target orientations are offsets in the gamma angle of the initial orientation
    double gammaOffset = Math.toRadians(-20);

    Transformation initPose = getCurrentPoseInRobRoot();
     
    Transformation poseOne = Transformation.ofRad(
        initPose.getX(), 
        initPose.getY(), 
        initPose.getZ()+sideLength, 
        initPose.getAlphaRad(), 
        initPose.getBetaRad(), 
        initPose.getGammaRad()+gammaOffset);
    Transformation poseTwo = Transformation.ofRad(
        poseOne.getX(), 
        poseOne.getY()+sideLength, 
        poseOne.getZ(), 
        initPose.getAlphaRad(), 
        initPose.getBetaRad(), 
        initPose.getGammaRad()-gammaOffset);
    Transformation poseThree = Transformation.ofRad(
        poseTwo.getX(), 
        poseTwo.getY(), 
        poseTwo.getZ()-sideLength, 
        initPose.getAlphaRad(), 
        initPose.getBetaRad(), 
        initPose.getGammaRad());

    // put the poses in the targets list
    int nrOfLoops = 3;
    for (int i=0; i<nrOfLoops; ++i)
    {
      _targetList.add(poseOne);
      _targetList.add(poseTwo);
      _targetList.add(poseThree);
      _targetList.add(initPose);
    }
  }

  private Transformation getCurrentPoseInRobRoot() {
    return _robot.getFlange().getTransformationFromParent();
  }

  private double[] getTargetVelocity(Transformation targetPose)
  {
    // proportional gains for the translational and rotational components, to be tuned further depending on the
    // application requirements
    double posGain = 5;
    double rotGain = 3;

    // compute the translational velocity vector from the distance between current and target positions
    Transformation currentPose = getCurrentPoseInRobRoot();
    Vector3D deltaPosition = targetPose.getTranslation().subtract(currentPose.getTranslation());
    Vector3D translationalVelocity = deltaPosition.multiply(posGain);
    
    // compute the rotation from the current to the target orientation  
    Rotation deltaRotation = currentPose.invert().compose(targetPose).getRotation();
    // use the resulting ABC angles (intrinsic Euler ZYX convention) as direction for the angular velocity vector,
    // simply reordering the elements
    // NOTE: This is not a general approach for computing the angular velocity but will converge for rotations 
    //       around only one axis and for small rotations. Quaternions should be used in the general case. 
    Vector3D deltaRotationXYZ_tcp = Vector3D.of(deltaRotation.getGammaRad(), deltaRotation.getBetaRad(), deltaRotation.getAlphaRad());
    // transform the direction to the robot root coordinate system and apply proportional gain to obtain the
    // angular velocity vector
    Vector3D angularVelocity = currentPose.getRotation().applyTo(deltaRotationXYZ_tcp).multiply(rotGain);

    // compose the complete Cartesian velocity vector
    double cartVelCmd[] = {
        translationalVelocity.getX(), 
        translationalVelocity.getY(),
        translationalVelocity.getZ(),
        angularVelocity.getX(),
        angularVelocity.getY(),
        angularVelocity.getZ()
    };

    return cartVelCmd.clone();
  }
}
