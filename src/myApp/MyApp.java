package myApp;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.IRoboticsAPITaskInjectableTypes;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.scenegraph.ISceneGraph;
import javax.inject.Inject;
import com.kuka.threading.ThreadUtil;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.device.RoboticArm;
import com.kuka.device.common.JointPosition;

/**
 * Implementation of a robot application.
 * 
 * <p>The application provides an {@link #initialize()} and a {@link #run()} method, which will be
 * called successively in the application life cycle. The application will terminate automatically
 * after the {@link #run()} method has finished or after stopping the task. The {@link #dispose()}
 * method will be called, even if an exception is thrown during initialization or run.
 * 
 * @see IRoboticsAPITaskInjectableTypes Types and Services available for Dependency Injection
 * @see RoboticsAPIApplication Application specific services available for Dependency Injection
 */
public class MyApp extends RoboticsAPIApplication {
  @Inject private LBRMed lBR_Med_7_R800_1;
  @Inject private ISceneGraph sceneGraph;
  @Inject private RoboticArm _robot;

  @Override
  public void initialize() throws Exception {
    // Cleans the scene graph by removing all transient objects
    sceneGraph.clean();
    
    // TODO Initialize your application here
  }

  @Override
  public void run() throws Exception {
    // TODO Your application execution starts here
    // --- Define two joint positions ---
    JointPosition posA = new JointPosition(
        Math.toRadians(0), 
        Math.toRadians(30),
        Math.toRadians(0),
        Math.toRadians(-90),
        Math.toRadians(0),
        Math.toRadians(90),
        Math.toRadians(0)
    );

    JointPosition posB = new JointPosition(
        Math.toRadians(10),
        Math.toRadians(20),
        Math.toRadians(10),
        Math.toRadians(-80),
        Math.toRadians(10),
        Math.toRadians(100),
        Math.toRadians(5)
    );

    // --- Move to posA ---
    _robot.getFlange().move(ptp(posA).setJointVelocityRel(0.2));

    // --- Wait for 3 seconds ---
    ThreadUtil.milliSleep(3000);

    // --- Move to posB ---
    _robot.getFlange().move(ptp(posB).setJointVelocityRel(0.2));

    // --- Done ---
    getLogger().info("Movement complete!");
  }
}
