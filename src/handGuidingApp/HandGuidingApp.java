package handGuidingApp;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.IRoboticsAPITaskInjectableTypes;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.scenegraph.ISceneGraph;
import javax.inject.Inject;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.device.common.JointPosition;
import com.kuka.device.common.JointPositionBuilder;
import com.kuka.geometry.LoadData;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.Tool;
import com.kuka.math.geometry.XyzAbcTransformation;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartDOF;
import com.kuka.sensitivity.LBR;
import com.kuka.sensitivity.controlmode.CartesianImpedanceControlMode;
import com.kuka.servoing.api.common.EServoRequestState;
import com.kuka.servoing.api.common.IServoingCapability;
import com.kuka.servoing.api.smartservo.ISmartServo;
import com.kuka.servoing.api.smartservo.ISmartServoRuntime;
import com.kuka.statistics.StatisticTimer;
import com.kuka.statistics.StatisticTimer.OneTimeStep;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;
import java.util.concurrent.TimeUnit;


/**
 * Implementation of a hand guiding programme to move the robot around by hand easily
 */

public class HandGuidingApp extends RoboticsAPIApplication {
  @Inject private LBRMed lBR_Med_7_R800_1;
  @Inject private ISceneGraph sceneGraph;
  
  @Inject private LBR _robot;
  @Inject private IServoingCapability _servoingCapability;

  private Tool _tool;
  private ITaskLogger _logger;

  private static final double[] TRANSLATION_OF_TOOL = {0, 0, 100};
  private static final double MASS = 0;
  private static final double[] CENTER_OF_MASS = {0, 0, 100};

  @Override
  public void initialize() {
      sceneGraph.clean();
      _logger = getLogger();

      // Create a Tool
      LoadData loadData = new LoadData();
      loadData.setMass(MASS);
      loadData.setCenterOfMass(CENTER_OF_MASS[0], CENTER_OF_MASS[1], CENTER_OF_MASS[2]);
      _tool = new Tool("Tool", loadData);

      XyzAbcTransformation trans = XyzAbcTransformation.of(
              TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1], TRANSLATION_OF_TOOL[2]);
      ObjectFrame toolFrame = _tool.createFrame("toolFrame", trans);
      _tool.setDefaultMotionFrame(toolFrame);
      _tool.attachTo(_robot.getFlange());
  }

  private void moveToInitialPosition() {
      _tool.move(ptp(JointPosition.ofDeg(90, 30, 0, -60, 0, 90, 0)).setJointVelocityRel(0.1)); //0, 30, 0, -60, 0, 90, 0
  }

  private CartesianImpedanceControlMode createLowCartHighJointStiffness() {
      CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();

      // VERY LOW cartesian stiffness (so it moves easily by hand)
      cartImp.parametrize(CartDOF.TRANSL).setStiffness(10.0);  // N/m
      cartImp.parametrize(CartDOF.ROT).setStiffness(5.0);     // Nm/rad

      // HIGH nullspace stiffness (joint stiffness)
      cartImp.setNullSpaceStiffness(1000.0);

      // Safety limits
      cartImp.setMaxPathDeviation(50, 50, 50, 50, 50, 50);
      return cartImp;
  }

  @Override
  public void run() {
      moveToInitialPosition();

      JointPosition initialPos = _robot.getCurrentJointPosition();
      ISmartServo servoMotion = _servoingCapability.createSmartServoMotion(initialPos);

      // Slow motion settings
      servoMotion.setJointAccelerationRel(0.1);
      servoMotion.setJointVelocityRel(0.1);
      servoMotion.setMinimumTrajectoryExecutionTime(20e-3);

      // Validate for impedance mode
      try { servoMotion.validateForImpedanceMode(_tool); }
      catch (IllegalStateException e) {
          _logger.error("LoadData validation failed: " + e.getMessage());
          return;
      }

      // Cartesian impedance: low in Cartesian, high in joints
      CartesianImpedanceControlMode impedance = createLowCartHighJointStiffness();

      _logger.info("Starting SmartServo hand-guiding mode");
      _tool.moveAsync(servoMotion.setMode(impedance));

      ISmartServoRuntime runtime = servoMotion.getRuntime(500, TimeUnit.MILLISECONDS);

      _logger.info("Hand-guiding active. Push the robot in Cartesian space. Release to hold position.");

      // Keep running until the user stops the application
      while (true) {
          ThreadUtil.milliSleep(20);
          EServoRequestState result = runtime.updateWithRealtimeSystem();
          if (result != EServoRequestState.SUCCESSFUL) {
              _logger.error("Realtime system update failed: " + result);
              break;
          }
      }

      runtime.stopMotion();
      _logger.info("Hand-guiding stopped.");
  }
}
