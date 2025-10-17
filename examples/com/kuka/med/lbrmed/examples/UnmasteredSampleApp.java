package com.kuka.med.lbrmed.examples;

import javax.inject.Inject;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import com.kuka.med.devicemodel.LBRMed;
import com.kuka.med.mastering.IMasteringService;
import com.kuka.med.mastering.MedApplicationCategory;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.task.ITaskLogger;

/**
 * This example shows how to master the joint number 2 of the robot. If the joint is already mastered, it invalidates
 * the mastering first. The application can even start if the joint is not mastered, due to the annotation
 * "@MedApplicationCategory(checkMastering = false)".
 */
@MedApplicationCategory(checkMastering = false)
public class UnmasteredSampleApp extends RoboticsAPIApplication
{
    @Inject private ITaskLogger _logger;
    @Inject private LBRMed _lbrMed;
    @Inject private IMasteringService _masteringService;

    @Override
    public void run()
    {
        //Invalidate joint 2 mastering only if it is already mastered.
        //Invalidating an already invalidated joint will throw an exception. 
        if (_masteringService.isJointMastered(1))
        {
            _logger.info("Joint 2 is mastered ... invalidating its mastering");
            _masteringService.invalidateMastering(1);
        }

        _logger.info("Mastering Joint 2...");
        if (_masteringService.masterJoint(1))
        {
            _logger.info("Mastering Joint 2 finished");
        }
        else
        {
            _logger.error("Mastering Joint 2 failed");
        }

        //Move to home position (will fail if not all axes are mastered) 
        //WARNING: Make sure, that the pose is collision free!
        _logger.info("Moving home...");
        _lbrMed.getFlange().move(ptpHome().setJointVelocityRel(0.2));

        _logger.info("Application finished");
    }
}
