package handGuidingApp;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.TorqueData;
import com.kuka.scenegraph.ISceneGraph;
import com.kuka.sensitivity.LBR;
import com.kuka.device.common.JointEnum;
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
    private static final double MASS = 0.4;//1.2
    private static final double[] CENTER_OF_MASS = {-10, -5, -75};

    // --- Define joint limits (example for LBR Med 7 R800) ---
    private static final double[] JOINT_MIN = {-170, -120, -170, -120, -170, -120, -175};
    private static final double[] JOINT_MAX = { 170,  120,  170,  120,  170,  120,  175};

    private static final double[] JOINT_TORQUE_THRESHOLD = {2,  2,  2,  2,  2,  2,  2};
    private static final double STIFFNESS_MAX = 10.0;
    private static final double THRESHOLD_DEG = 20.0;

    private void moveToInitialPosition() {
      JointPosition currentPos = _robot.getCurrentJointPosition();
      _tool.move(ptp(currentPos).setJointVelocityRel(0.3));
        //_tool.move(ptp(JointPosition.ofDeg(90, 30, 0, -60, 0, 90, 0)).setJointVelocityRel(0.3));
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
        JointEnum[] joints = JointEnum.values();

        moveToInitialPosition(); // this is needed otherwise the robot safety trigger due to movement being done too fast...
        
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

        JointImpedanceControlMode impedance = new JointImpedanceControlMode(0, 0, 0, 0, 0, 0, 0);

        _logger.info("Starting SmartServo hand-guiding with adaptive stiffness.");
        _tool.moveAsync(servoMotion.setMode(impedance));

        ISmartServoRuntime runtime = servoMotion.getRuntime(500, TimeUnit.MILLISECONDS);
        
     // --- Initialize torque averaging ---
        final int AVG_WINDOW = 20;
        double[][] torqueHistory = new double[7][AVG_WINDOW];
        int sampleIndex = 0;
        boolean bufferFilled = false;

        while (true) {
            ThreadUtil.milliSleep(20);
            EServoRequestState result = runtime.updateWithRealtimeSystem();
            if (result != EServoRequestState.SUCCESSFUL) {
                _logger.error("Realtime update failed: " + result);
                break;
            }

            // --- Read current torques and joint positions ---
            TorqueData extTorques = _robot.getExternalTorque();
            JointPosition currentPos = _robot.getCurrentJointPosition();

            // --- Store new torques in circular buffer ---
            for (int j = 0; j < 7; j++) {
                torqueHistory[j][sampleIndex] = extTorques.getSingleTorqueValue(joints[j]);
            }
            sampleIndex = (sampleIndex + 1) % AVG_WINDOW;
            if (sampleIndex == 0) bufferFilled = true;

            // --- Compute adaptive stiffness ---
            double[] stiffnessValues = new double[7];
            for (int j = 0; j < 7; j++) {
                // Compute moving average torque for this joint
                double avgTorque = 0.0;
                int count = bufferFilled ? AVG_WINDOW : sampleIndex;
                for (int k = 0; k < count; k++) {
                    avgTorque += torqueHistory[j][k];
                }
                avgTorque /= (count > 0 ? count : 1);

                // --- Base stiffness from proximity to joint limits ---
                double angleDeg = Math.toDegrees(currentPos.get(j));
                double distToMin = angleDeg - JOINT_MIN[j];
                double distToMax = JOINT_MAX[j] - angleDeg;

                double stiffnessFromLimit = 0.0;
                if (distToMin < THRESHOLD_DEG)
                    stiffnessFromLimit = STIFFNESS_MAX * (1.0 - distToMin / THRESHOLD_DEG);
                if (distToMax < THRESHOLD_DEG)
                    stiffnessFromLimit = Math.max(stiffnessFromLimit,
                            STIFFNESS_MAX * (1.0 - distToMax / THRESHOLD_DEG));

                stiffnessFromLimit = Math.min(Math.max(stiffnessFromLimit, 0.0), STIFFNESS_MAX);

                // --- Adaptive stiffness from torque average ---
                double stiffnessFromTorque;
                if (Math.abs(avgTorque) > JOINT_TORQUE_THRESHOLD[j]) {
                    // External torque sustained -> go soft
                    stiffnessFromTorque = 0.0;
                } else {
                    // Below threshold -> keep robot stiff
                    stiffnessFromTorque = STIFFNESS_MAX;
                }
                stiffnessFromTorque = 0; // comment me to reanble the static friction that dont work...
                // Combine effects (take the stronger one)
                stiffnessValues[j] = Math.max(stiffnessFromLimit, stiffnessFromTorque);
            }

            // --- Update impedance control dynamically ---
            impedance.setStiffness(stiffnessValues);
            runtime.changeControlModeSettings(impedance);
        }

        runtime.stopMotion();
        _logger.info("Hand-guiding stopped.");

    }
}
