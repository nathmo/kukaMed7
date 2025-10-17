package com.kuka.med.lbrmed.examples;

import java.util.EnumSet;
import java.util.Set;

import javax.inject.Inject;

import com.kuka.device.RoboticArm;
import com.kuka.device.common.JointEnum;
import com.kuka.device.common.JointPosition;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.World;
import com.kuka.math.geometry.Transformation;
import com.kuka.med.jogging.EJoggingStopCondition;
import com.kuka.med.jogging.IJoggingService;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.CartesianCoordinates;
import com.kuka.roboticsAPI.motionModel.CartesianJoggingMode;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;

/**
 * Implementation of a sample application to jog from the application instead of using the keys on the smartPAD.
 * Examples for jogging on multiple joints/Cartesian coordinates with runtime update, time-specific jogging command and
 * enable jogging stop motion condition are provided.
 */
public class JoggingSampleApp extends RoboticsAPIApplication
{

    private static final double JOGGING_VEL = 0.3;
    private static final long JOGGING_TIME = 3000;
    private static final long TIMEOUT_DISABLED = -1;

    private static final JointPosition HOME = JointPosition.ofDeg(4, -45, -2, 100, 2, -34, 16);

    @Inject
    private RoboticArm _robot;

    @Inject
    private ITaskLogger _logger;

    @Inject
    private World _world;

    @Override
    public void initialize()
    {
        _logger = getLogger();
    }

    @Override
    public void run()
    {
        // Move to initial position to start jogging from the application
        _robot.getFlange().move(new PTP(HOME).setJointVelocityRel(JOGGING_VEL));

        IJoggingService jogging = _robot.getCapability(IJoggingService.class);

        /* 1. Jogging on single or multiple joints with runtime override update and disabled timeout condition. Note:
         * the
         * runtime update of the jogging parameters overrides the previous ones; thus if the jogging motion is desired
         * on one/set of joints/coordinates which are already active, they must be specified again in the new jogging
         * parameters (e.g. jogging on joint J2). */
        _logger.info("Joint jogging on joint J2.");
        jogging.startJointJogging(EnumSet.of(JointEnum.J2), JOGGING_VEL, TIMEOUT_DISABLED);
        ThreadUtil.milliSleep(JOGGING_TIME); //[ms] 
        jogging.updateJointJogging(EnumSet.range(JointEnum.J2, JointEnum.J4), JOGGING_VEL);
        _logger.info("Runtime update: joint jogging on joints J2-J4.");
        ThreadUtil.milliSleep(JOGGING_TIME); //[ms]
        jogging.stopJointJogging(EnumSet.range(JointEnum.J1, JointEnum.J7));
        _logger.info("Stop joint jogging.");

        /* 2. Time-specific joint jogging on J2 joint with disabled stop motion condition. */
        jogging.stopJoggingAtJointLimitOrSingularity(false, true);
        jogging.startJointJogging(EnumSet.of(JointEnum.J2), -JOGGING_VEL, JOGGING_TIME);
        _logger.info("Time-specific jogging on joint J2");
        if (waitUntilJoggingIsFinished(jogging, 100, JOGGING_TIME))
        {
            _logger.info("Jogging timeout expired after " + JOGGING_TIME + " [ms]");
        }

        /* 3. Cartesian jogging on external Base Frame with enabled stop motion condition. */
        final ObjectFrame base = _world.createFrame("Base",
                Transformation.ofRad(0.0, 0.0, 0.0, Math.PI, 0.0, 0.0));
        jogging.setBase(base);
        Set<CartesianCoordinates> coordinates = EnumSet.of(CartesianCoordinates.Z);
        jogging.startCartesianJogging(coordinates, JOGGING_VEL, CartesianJoggingMode.BASE);
        _logger.info("Cartesian jogging in BASE frame along coordinates " + coordinates.toString());
        if (!waitUntilJoggingIsFinished(jogging, 100, JOGGING_TIME))
        {
            jogging.stopCartesianJogging(coordinates);
        }
        if (jogging.getFiredStopCondition().equals(EJoggingStopCondition.MOTION_IMPOSSIBLE))
        {
            _logger.info("Joint limit or singularity occurred");
        }
    }

    private boolean waitUntilJoggingIsFinished(IJoggingService jogging, long timeStamp, long timeout)
    {
        int count = 0;
        while (jogging.isJoggingActive())
        {
            if (count > timeout)
            {
                return false;
            }
            ThreadUtil.milliSleep(timeStamp);
            count += timeStamp;
        }
        return true;
    }

}
