package com.kuka.fri.lbr.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.device.RoboticArm;
import com.kuka.fri.api.monitoring.IFriConfiguration;
import com.kuka.fri.api.monitoring.IFriMonitoring;
import com.kuka.fri.api.monitoring.IFriSession;
import com.kuka.io.IIoDefinitionProvider;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.threading.ThreadUtil;

/**
 * Implementation of a FRI example application to demonstrate which configuration is required to get access to I/O data
 * on the FRI client side.
 * <p>
 * For this example, a specific I/O configuration is copied to the controller, which can be accessed using
 * {@code FriIoGroup}. During the example one output is changed on the application side, which can then be observed on
 * the client side. Depending on this value, the client reacts and commands changes for the ports
 * (see IOAccessClient).
 */
public class FriIoApp extends RoboticsAPIApplication
{
    private String _clientName;
    private IFriSession _friSession;

    @Inject
    private IIoDefinitionProvider _ioDefinitionProvider;

    @Inject
    private RoboticArm _robot;

    @Inject
    private FriIoGroup _friGroup;

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
        IFriMonitoring friMonitoring = _robot.getCapability(IFriMonitoring.class);

        IFriConfiguration friConfiguration = friMonitoring.createFriConfiguration(_robot, _clientName);
        friConfiguration.setSendPeriodMilliSec(10);

        friConfiguration.registerIO(_ioDefinitionProvider.getIoDefinition(FriIoGroup.NAME, "In_Bool_Clock_Enabled"));
        friConfiguration.registerIO(_ioDefinitionProvider.getIoDefinition(FriIoGroup.NAME, "Out_Bool_Enable_Clock"));
        friConfiguration.registerIO(_ioDefinitionProvider.getIoDefinition(FriIoGroup.NAME, "Out_Integer_Seconds"));
        friConfiguration.registerIO(_ioDefinitionProvider.getIoDefinition(FriIoGroup.NAME, "Out_Analog_Deci_Seconds"));

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        _friSession = friMonitoring.createFriSession(friConfiguration);

        // wait until FRI session is ready to switch to command mode
        try
        {
            _friSession.await(20, TimeUnit.SECONDS);
            getLogger().info("Connection to Client established");
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            _friSession.close();
            return;
        }

        getLogger().info("enable clock");
        ThreadUtil.milliSleep(5000);
        _friGroup.setOut_Bool_Enable_Clock(true);

        getLogger().info("do something ...");
        ThreadUtil.milliSleep(10000);

        getLogger().info("disable clock");
        _friGroup.setOut_Bool_Enable_Clock(false);

        _robot.getFlange().move(ptpHome().setJointVelocityRel(0.2));
    }
}
