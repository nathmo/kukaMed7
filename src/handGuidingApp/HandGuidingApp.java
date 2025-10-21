package handGuidingApp;
// using joint Impedance mode with home position

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
import com.kuka.sensitivity.controlmode.JointImpedanceControlMode;
import com.kuka.servoing.api.common.EServoRequestState;
import com.kuka.servoing.api.common.IServoingCapability;
import com.kuka.servoing.api.smartservo.ISmartServo;
import com.kuka.servoing.api.smartservo.ISmartServoRuntime;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;
import java.util.concurrent.TimeUnit;


/**
 * Implementation of a hand guiding programme to move the robot around by hand easily
 */

public class HandGuidingApp extends RoboticsAPIApplication {
  @Inject private ISceneGraph sceneGraph;
  
  @Inject private LBR _robot;
  @Inject private IServoingCapability _servoingCapability;

  private Tool _tool;
  private ITaskLogger _logger;

  private static final double[] TRANSLATION_OF_TOOL = {0, 0, 0};
  private static final double MASS = 0.45; // for an empty robot, light and it fall and more and it balloon
  private static final double[] CENTER_OF_MASS = {0, 0, 0};

  private void moveToInitialPosition() {
    _tool.move(ptp(JointPosition.ofDeg(90, 30, 0, -60, 0, 90, 0)).setJointVelocityRel(0.4)); //0, 30, 0, -60, 0, 90, 0
  }
  
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


  @Override
  public void run() {
      moveToInitialPosition(); // close to the knee on table
      
      JointPosition initialPos = _robot.getCurrentJointPosition();
      ISmartServo servoMotion = _servoingCapability.createSmartServoMotion(initialPos);

      // Slow motion settings
      servoMotion.setJointAccelerationRel(0.8);
      servoMotion.setJointVelocityRel(0.8);
      servoMotion.setMinimumTrajectoryExecutionTime(0.001);

      // Validate for impedance mode
      try { servoMotion.validateForImpedanceMode(_tool); }
      catch (IllegalStateException e) {
          _logger.error("LoadData validation failed: " + e.getMessage());
          return;
      }


      JointImpedanceControlMode impedance =new JointImpedanceControlMode(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

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
