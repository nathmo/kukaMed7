package com.kuka.fri.lbr.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.device.common.JointPosition;
import com.kuka.fri.api.commanding.IFriCommanding;
import com.kuka.fri.api.commanding.IFriJointOverlay;
import com.kuka.fri.api.monitoring.IFriConfiguration;
import com.kuka.fri.api.monitoring.IFriMonitoring;
import com.kuka.fri.api.monitoring.IFriSession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.sensitivity.LBR;

/**
 * Implementation of a FRI example application to demonstrate how to use a joint overlay with position control mode.
 * <p>
 * The example command three different PTP motions while the client commands a sine overlay
 * (see LBRJointSineOverlayClient).
 */
public class LBRJointSineOverlay extends RoboticsAPIApplication
{
    private String _clientName;
    private IFriSession _friSession;

    @Inject
    private LBR _lbr;

    @Override
    public void initialize()
    {
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "127.0.0.1";
    }

    @Override
    public void dispose()
    {
        getLogger().info("Close connection to client");

        if (null != _friSession)
        {
            _friSession.close();
        }
    }

    @Override
    public void run()
    {
        IFriMonitoring friMonitoring = _lbr.getCapability(IFriMonitoring.class);
        IFriCommanding friCommanding = _lbr.getCapability(IFriCommanding.class);

        // configure and start FRI session
        IFriConfiguration friConfiguration = friMonitoring.createFriConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(5);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        _friSession = friMonitoring.createFriSession(friConfiguration);
        IFriJointOverlay jointOverlay = friCommanding.createJointOverlay(_friSession);

        // wait until FRI session is ready to switch to command mode
        try
        {
            _friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            _friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        // move to start pose
        _lbr.getFlange().move(ptp(JointPosition.ofDeg(90.0, 0.0, 0.0, 90.0, 0.0, -90.0, 0.0)).setJointVelocityRel(0.2));

        // async move with overlay ...
        _lbr.getFlange().moveAsync(ptp(JointPosition.ofDeg(-90.0, 0.0, 0.0, 90.0, 0.0, -90.0, 0.0))
                .setJointVelocityRel(0.2)
                .addMotionOverlay(jointOverlay)
                .setBlendingRel(0.1));

        // ... blending into sync move with overlay
        _lbr.getFlange().move(ptp(JointPosition.ofDeg(90.0, 0.0, 0.0, 90.0, 0.0, -90.0, 0.0))
                .setJointVelocityRel(0.2)
                .addMotionOverlay(jointOverlay));
    }
}
