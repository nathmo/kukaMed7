package com.kuka.fri.lbr.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.device.common.JointPosition;
import com.kuka.fri.api.commanding.IFriCartesianOverlay;
import com.kuka.fri.api.commanding.IFriCommanding;
import com.kuka.fri.api.monitoring.IFriConfiguration;
import com.kuka.fri.api.monitoring.IFriMonitoring;
import com.kuka.fri.api.monitoring.IFriSession;
import com.kuka.geometry.LocalFrame;
import com.kuka.geometry.World;
import com.kuka.math.geometry.Transformation;
import com.kuka.math.geometry.Vector3D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.scenegraph.ISceneGraph;
import com.kuka.sensitivity.LBR;

/**
 * Implementation of a FRI example application to demonstrate how to use a Cartesian overlay.
 * <p>
 * The example provides three different scenarios, that a executed consecutively on a PositionHold:
 * 1. overlay without TCP and base transformation
 * 2. overlay with a TCP transformation
 * 3. overlay with a base transformation
 * <p>
 * In all cases the client commands a circular motion (see LBRCartesianOverlayClient).
 */
public class LBRCartesianOverlay extends RoboticsAPIApplication
{
    @Inject
    private LBR _lbr;
    @Inject
    private ISceneGraph sceneGraph;
    @Inject
    private World _world;

    private String _clientName;
    private IFriSession _friSession;

    @Override
    public void initialize() throws Exception
    {
        // Cleans the scene graph by removing all transient objects
        sceneGraph.clean();

        // **********************************************************************
        // *** change next line to the FRIClient's IP address ***
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

        Vector3D tcpTranslationVector = Vector3D.of(0, 0, 200);
        LocalFrame tcp = new LocalFrame(_lbr.getFlange(), Transformation.of(tcpTranslationVector));

        Vector3D baseTranslationVector = Vector3D.of(-500, -500, 0);
        LocalFrame base = new LocalFrame(_world.getRootFrame(), Transformation.of(baseTranslationVector));

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());

        for (int i = 0; i < 3; i++)
        {
            switch (i)
            {
            case 0:
                getLogger().info("Cartesian Monitoring without TCP or base configuration!");
                friConfiguration.setBase(_world.getRootFrame());
                friConfiguration.setTcp(_lbr.getFlange());
                break;
            case 1:
                getLogger().info("Cartesian Monitoring with configured tcp " + tcpTranslationVector);
                friConfiguration.setBase(_world.getRootFrame());
                friConfiguration.setTcp(tcp);
                break;
            case 2:
                getLogger().info("Cartesian Monitoring with configured base " + baseTranslationVector);
                friConfiguration.setBase(base);
                friConfiguration.setTcp(_lbr.getFlange());
                break;
            default:
                break;
            }

            _friSession = friMonitoring.createFriSession(friConfiguration);
            IFriCartesianOverlay cartOverlay = friCommanding.createCartesianOverlay(_friSession);

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
            _lbr.getFlange().move(
                    ptp(JointPosition.ofDeg(90.0, -30.0, 0.0, 60.0, 0.0, -90.0, 0.0)).setJointVelocityRel(0.2));

            // start FRI Cartesian overlay
            PositionHold posHold = new PositionHold(new PositionControlMode(), 20, TimeUnit.SECONDS);
            _lbr.getFlange().move(posHold.addMotionOverlay(cartOverlay));

            // close the FRI session
            _friSession.close();
        }
    }
}
