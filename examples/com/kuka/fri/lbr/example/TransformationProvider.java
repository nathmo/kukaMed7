package com.kuka.fri.lbr.example;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.fri.api.monitoring.IFriConfiguration;
import com.kuka.fri.api.monitoring.IFriMonitoring;
import com.kuka.fri.api.monitoring.IFriSession;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.World;
import com.kuka.math.geometry.Transformation;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.sensitivity.LBR;
import com.kuka.threading.ThreadUtil;

/**
 * Implementation of a FRI example application to demonstrate how to use a transformation provider.
 * <p>
 * The example configures a simple scene tree of frames relative to world and the client side
 * (see TransformationProviderClient) is modifying the base transformation, which is then visible here in the
 * application.
 */
public class TransformationProvider extends RoboticsAPIApplication
{
    private String _clientName;
    private IFriSession _friSession;

    private ObjectFrame _objectBase;
    private ObjectFrame _objectTip;

    @Inject
    private LBR _lbr;

    @Inject
    private World _world;

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

        _world.removeFrame(_objectBase);
    }

    @Override
    public void run()
    {
        IFriMonitoring friMonitoring = _lbr.getCapability(IFriMonitoring.class);

        // Create some frames
        _objectBase = _world.getRootFrame().findFrame("/FRIexampleBase");
        if (null == _objectBase)
        {
            _objectBase = _world.createFrame("FRIexampleBase", Transformation.IDENTITY);
        }

        _objectTip = _world.getRootFrame().findFrame("/FRIexampleBase/FRIexampleTip");
        if (null == _objectTip)
        {
            _objectTip = _world.createFrame("FRIexampleTip", _objectBase,
                    Transformation.ofDeg(10.0, 10.0, 10.0, 45.0, 45.0, 45.0));
        }

        // Configure and start FRI session
        IFriConfiguration friConfiguration = friMonitoring.createFriConfiguration(_lbr, _clientName);

        // Send period from LBR to client
        friConfiguration.setSendPeriodMilliSec(10);

        // Send period multiply with integer gives the receive period from client to robot controller. 
        // In this example it is 30 milliseconds.
        friConfiguration.setReceiveMultiplier(3);

        // Select the frame from the scene graph whose transformation is changed by the client application.
        friConfiguration.registerTransformationProvider("PBase", _objectBase);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        _friSession = friMonitoring.createFriSession(friConfiguration);

        try
        {
            _friSession.await(10, TimeUnit.SECONDS);
        }
        catch (TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            _friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        // Output
        getLogger().info("Transformation from World of");
        for (int i = 0; i < 100; i++)
        {
            ThreadUtil.milliSleep(15);
            getLogger().info("Frame objectBase:\n" + _objectBase.calculateTransformationFromTreeRoot());
            getLogger().info("Frame objectTip:\n" + _objectTip.calculateTransformationFromTreeRoot());
        }
    }
}
