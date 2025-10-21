package handGuidingApp;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.scenegraph.ISceneGraph;
import com.kuka.sensitivity.LBR;
import com.kuka.device.common.JointPosition;
import com.kuka.geometry.*;
import com.kuka.math.geometry.XyzAbcTransformation;
import com.kuka.servoing.api.common.EServoRequestState;
import com.kuka.servoing.api.smartservo.ISmartServo;
import com.kuka.servoing.api.smartservo.ISmartServoRuntime;
import com.kuka.sensitivity.controlmode.JointImpedanceControlMode;
import com.kuka.threading.ThreadUtil;
import com.kuka.task.ITaskLogger;
import javax.inject.Inject;
import java.util.concurrent.TimeUnit;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

public class HandGuidingApp extends RoboticsAPIApplication {
    @Inject private ISceneGraph sceneGraph;
    @Inject private LBR _robot;
    @Inject private com.kuka.servoing.api.common.IServoingCapability _servoingCapability;

    private Tool _tool;
    private ITaskLogger _logger;

    private static final double[] TRANSLATION_OF_TOOL = {0, 0, 0};
    private static final double MASS = 0.45;
    private static final double[] CENTER_OF_MASS = {0, 0, 0};

    // --- Define joint limits (example for LBR Med 7 R800) ---
    private static final double[] JOINT_MIN = {-170, -120, -170, -120, -170, -120, -175};
    private static final double[] JOINT_MAX = { 170,  120,  170,  120,  170,  120,  175};

    private static final double STIFFNESS_MAX = 10.0;
    private static final double THRESHOLD_DEG = 20.0;

    private void moveToInitialPosition() {
        _tool.move(ptp(JointPosition.ofDeg(90, 30, 0, -60, 0, 90, 0)).setJointVelocityRel(0.3));
    }

    @Override
    public void initialize() {
        sceneGraph.clean();
        _logger = getLogger();

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
        moveToInitialPosition();

        JointPosition initialPos = _robot.getCurrentJointPosition();
        ISmartServo servoMotion = _servoingCapability.createSmartServoMotion(initialPos);

        servoMotion.setJointAccelerationRel(0.8);
        servoMotion.setJointVelocityRel(0.8);
        servoMotion.setMinimumTrajectoryExecutionTime(0.001);

        try { servoMotion.validateForImpedanceMode(_tool); }
        catch (IllegalStateException e) {
            _logger.error("LoadData validation failed: " + e.getMessage());
            return;
        }

        JointImpedanceControlMode impedance = new JointImpedanceControlMode(
                0, 0, 0, 0, 0, 0, 0);

        _logger.info("Starting SmartServo hand-guiding with adaptive stiffness.");
        _tool.moveAsync(servoMotion.setMode(impedance));

        ISmartServoRuntime runtime = servoMotion.getRuntime(500, TimeUnit.MILLISECONDS);

        while (true) {
            ThreadUtil.milliSleep(50);
            EServoRequestState result = runtime.updateWithRealtimeSystem();
            if (result != EServoRequestState.SUCCESSFUL) {
                _logger.error("Realtime update failed: " + result);
                break;
            }

            // --- Compute adaptive stiffness ---
            JointPosition currentPos = _robot.getCurrentJointPosition();
            double[] stiffnessValues = new double[7];

            for (int i = 0; i < 7; i++) {
                double angle = Math.toDegrees(currentPos.get(i));
                double distToMin = angle - JOINT_MIN[i];
                double distToMax = JOINT_MAX[i] - angle;

                double stiffness = 0.0;

                if (distToMin < THRESHOLD_DEG)
                    stiffness = STIFFNESS_MAX * (1.0 - distToMin / THRESHOLD_DEG);
                if (distToMax < THRESHOLD_DEG)
                    stiffness = Math.max(stiffness, STIFFNESS_MAX * (1.0 - distToMax / THRESHOLD_DEG));

                stiffness = Math.min(Math.max(stiffness, 0.0), STIFFNESS_MAX);
                stiffnessValues[i] = stiffness;
            }

            // --- Update impedance control dynamically ---
            impedance.setStiffness(stiffnessValues);
            runtime.changeControlModeSettings(impedance);
        }

        runtime.stopMotion();
        _logger.info("Hand-guiding stopped.");
    }
}
