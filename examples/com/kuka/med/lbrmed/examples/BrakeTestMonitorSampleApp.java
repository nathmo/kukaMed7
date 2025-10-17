package com.kuka.med.lbrmed.examples;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import com.kuka.device.common.JointPosition;
import com.kuka.device.common.JointPositionBuilder;
import com.kuka.med.cyclicbraketest.EBraketestExecutionState;
import com.kuka.med.cyclicbraketest.EBraketestMonitorState;
import com.kuka.med.cyclicbraketest.EBraketestResult;
import com.kuka.med.cyclicbraketest.IBraketestEvent;
import com.kuka.med.cyclicbraketest.IBraketestMonitor;
import com.kuka.med.cyclicbraketest.IBraketestMonitorEvent;
import com.kuka.med.cyclicbraketest.IBraketestMonitorListener;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.task.ITaskLogger;

/**
 * Implementation of a sample application handling the cyclic brake test. As soon as the application is started, the
 * user is asked to manage the initial state of the cyclic monitoring, e.g. start the brake test execution or continue
 * with the application. Then, a continuous motion of the robot between two joint configurations is started to simulate
 * the normal application workflow. The IBrakeTestMonitorListener is used to receive events about state changes of the
 * cyclic brake test monitoring and react accordingly.
 */
public class BrakeTestMonitorSampleApp extends RoboticsAPIApplication implements IBraketestMonitorListener
{

    /**
     * User action for interaction with application workflow.
     */
    private enum UserAction
    {
        NO_ACTION,
        START_BRAKETEST,
        POSTPONE_BRAKETEST,
        EXIT_APPLICATION;
    }

    private static JointPosition HOME = JointPosition.ofDeg(0, 0, 0, 0, 0, 0, 0);
    private static JointPosition LEFT = JointPosition.ofDeg(0, -90, 0, -90, 0, -90, 0);
    private static JointPosition RIGHT = JointPosition.ofDeg(0, 90, 0, 90, 0, 90, 0);

    @Inject
    private ITaskLogger _logger;

    @Inject
    private IApplicationUI _appUI;

    @Inject
    private LBRMed _lbrMed;

    private IBraketestMonitor _brakeTestMonitor;
    private EBraketestMonitorState _currentStatus = EBraketestMonitorState.OK;
    private boolean _isBrakeTestExecutable = true;

    private int _notificationTime = 85000; //seconds

    /** User commanded action in the application dataflow. */
    private UserAction _action = UserAction.NO_ACTION;

    /** Additional thread to handle BrakeTestMonitorEvents with dialog boxes for user feedback. */
    private ExecutorService _workerToHandleEvent = Executors.newSingleThreadExecutor();

    @Override
    public void initialize()
    {
        _brakeTestMonitor = _lbrMed.getCapability(IBraketestMonitor.class);
        _brakeTestMonitor.addMonitoringListener(this);

        _notificationTime = _brakeTestMonitor.setNotificationTime(_notificationTime);
        _logger.info("notification time changed to " + _notificationTime);
    }

    @Override
    public void run()
    {
        // check the current status of the cyclic brake test because an event could have been missed due to
        // late start of the application
        _currentStatus = _brakeTestMonitor.getState();
        _logger.info("current state: " + _currentStatus);
        JointPosition initialBraketestExecutionPosition = _lbrMed.getCurrentJointPosition();
        _isBrakeTestExecutable = _brakeTestMonitor.isBraketestExecutable(initialBraketestExecutionPosition);

        // handle the brake test execution at start of the application
        handleInitialBrakeTestExecution();

        while (!_action.equals(UserAction.EXIT_APPLICATION))
        {
            // ******************* Execution of a brake test commanded via user interaction *******************
            if (_action.equals(UserAction.START_BRAKETEST) && _isBrakeTestExecutable)
            {
                String text = "Where shall the brake test be executed?";
                int position = _appUI.displayModalDialog(ApplicationDialogType.INFORMATION, text, "Here", "Left", "Home", "Right");
                JointPosition target = null;
                switch (position)
                {
                case 1:
                    target = LEFT;
                    break;
                case 2:
                    target = HOME;
                    break;
                case 3:
                    target = RIGHT;
                    break;
                default:
                    target = _lbrMed.getCurrentJointPosition();
                    break;
                }

                // execute the brake test
                EBraketestExecutionState outcome = _brakeTestMonitor.executeBraketest(target);
                _logger.info(String.format("The overall result of the brake test is '%s'!", outcome.toString()));
                if (_brakeTestMonitor.getState().equals(EBraketestMonitorState.OK))
                {
                    _action = UserAction.NO_ACTION;
                }
            }

            requestUserFeedback();
        }
    }

    /**
     * Implementation of the callback method to listen for monitor state events. An additional single thread is required
     * to use dialog boxes for user feedback without blocking the listener callback.
     */
    @Override
    public void onMonitoringStateChanged(IBraketestMonitorEvent event)
    {
        if (!event.getState().equals(_currentStatus))
        {
            _currentStatus = event.getState();
            _isBrakeTestExecutable = event.isBraketestExecutable();

            _logger.info("BrakeTestState changed to " + _currentStatus);

            // Run the handling of the event with dialog boxes in an additional Thread
            _workerToHandleEvent.execute(new Runnable()
            {
                @Override
                public void run()
                {
                    handleCyclicMonitoring();
                }
            });
        }
    }

    @Override
    public void onBraketestFinished(IBraketestEvent event)
    {
        publishCurrentStatus(event);
    }

    @Override
    public void onDiskSpaceWarning(String msg)
    {
        // not implemented
    }

    private void publishCurrentStatus(IBraketestEvent event)
    {
        String text = "The current status of the brake test is:\n";
        Map<Integer, EBraketestResult> results = event.getBraketestResults();
        for (int i = 0; i < results.size(); i++)
        {
            text = String.format("%s\nbrake %d: %s", text, (i + 1), results.get(i));
        }

        text = String.format("%s\nCBTM state: %s", text, _currentStatus);
        text = String.format("%s\nPostponement is %spossible! ", text, _brakeTestMonitor.isBraketestPostponeable() ? "" : "not ");
        text = String.format("%s\nBrake test is %sexecutable! ", text, _brakeTestMonitor.isBraketestExecutable() ? "" : "not ");

        _appUI.displayModalDialog(ApplicationDialogType.INFORMATION, text, "ok");
    }

    private void handleInitialBrakeTestExecution()
    {
        String text = "";

        JointPosition initialBraketestExecutionPosition = _lbrMed.getCurrentJointPosition();
        if (!_isBrakeTestExecutable)
        {
            getLogger().warn("Braketest at current position is not executable...");

            JointPositionBuilder validRange = new JointPositionBuilder(initialBraketestExecutionPosition);
            for (int i = 0; i < _lbrMed.getJointCount(); i++)
            {
                if (Math.abs(validRange.get(i) - _lbrMed.getJointLimits().getMinJointPosition().get(i)) < Math.toRadians(5))
                {
                    validRange.set(i, _lbrMed.getJointLimits().getMinJointPosition().get(i) + Math.toRadians(5.1));
                }
                else if (Math.abs(validRange.get(i) - _lbrMed.getJointLimits().getMaxJointPosition().get(i)) < Math.toRadians(5))
                {
                    validRange.set(i, _lbrMed.getJointLimits().getMaxJointPosition().get(i) - Math.toRadians(5.1));
                }
            }

            if (_brakeTestMonitor.isBraketestExecutable(validRange.build()))
            {
                getLogger().info(String.format("... but it is executable at the following position: %s", validRange.build().toStringDeg()));
                initialBraketestExecutionPosition = validRange.build();
                _isBrakeTestExecutable = true;
            }
            else
            {
                text = "The brake test is not executable! The system is in " + _currentStatus.toString() + " state. "
                        + (_currentStatus.equals(EBraketestMonitorState.FATAL) ? "All robot motions from the "
                                + "application are locked. Please contact customer support!" : "");
                if (0 == _appUI.displayModalDialog(ApplicationDialogType.ERROR, text, "Exit application"))
                {
                    // terminate application
                    _action = UserAction.EXIT_APPLICATION;
                }
            }
        }

        if (_isBrakeTestExecutable)
        {
            text = "Do you want to start a brake test execution now?";
            if (0 == _appUI.displayModalDialog(ApplicationDialogType.QUESTION, text, "Yes", "No"))
            {
                _action = UserAction.START_BRAKETEST;
            }

            if (_action.equals(UserAction.START_BRAKETEST))
            {
                _brakeTestMonitor.executeBraketest(initialBraketestExecutionPosition);
                _action = UserAction.NO_ACTION;
            }
        }
    }

    private void requestUserFeedback()
    {
        List<String> options = new ArrayList<>();
        options.add("Exit");
        options.add("Execute all brake tests");
        options.add("Execute single brake test");
        if (_brakeTestMonitor.isBraketestPostponeable())
        {
            // only available if the current state is 'PENDING' and if less than 3 postponements have been executed, yet.
            options.add("Postpone");
        }

        String text = "What do you want to do next?";
        int option = _appUI.displayModalDialog(ApplicationDialogType.QUESTION, text, options.toArray(new String[0]));
        switch (option)
        {
        case 0:
            _action = UserAction.EXIT_APPLICATION;
            return;
        case 1:
            _action = UserAction.START_BRAKETEST;
            return;
        case 2:
            break;
        case 3:
            _brakeTestMonitor.postponeBrakeTest();
            return;
        default:
            break;
        }

        text = "Which joint shall be tested?";
        option = _appUI.displayModalDialog(ApplicationDialogType.QUESTION, text, "1", "2", "3", "4", "5", "6", "7", "Exit");
        if (_lbrMed.getJointCount() <= option)
        {
            _action = UserAction.EXIT_APPLICATION;
            return;
        }

        _brakeTestMonitor.executeSingleBraketestOnJoint(option);
    }

    private void handleCyclicMonitoring()
    {
        if (_currentStatus.equals(EBraketestMonitorState.PENDING))
        {
            handleBraketestPendingState();
        }
        else if (_currentStatus.equals(EBraketestMonitorState.ERROR) && (_action != UserAction.START_BRAKETEST))
        {
            handleErrorState();
        }
        else if (_currentStatus.equals(EBraketestMonitorState.FATAL))
        {
            String text = "The system is in FATAL_ERROR state. All robot motions from the application are locked. Please "
                    + "contact customer support!";
            if (0 == _appUI.displayModalDialog(ApplicationDialogType.ERROR, text, "Exit application"))
            {
                // terminate application
                _action = UserAction.EXIT_APPLICATION;
            }
        }
    }

    private void handleErrorState()
    {
        String text = "A brake test is overdue! The robot can only be used after a successful brake test. "
                + "Do you want to start it now?";
        if (0 == _appUI.displayModalDialog(ApplicationDialogType.ERROR, text, "Yes", "Exit"))
        {
            _action = UserAction.START_BRAKETEST;
        }
        else
        {
            _action = UserAction.EXIT_APPLICATION;
        }
    }

    private void handleBraketestPendingState()
    {
        String text = "A brake test is due in " + _brakeTestMonitor.getTimeTillBraketestOverdue() + "seconds. "
                + "What would you like to do?";
        List<String> options = new LinkedList<>(Arrays.asList("start"));
        if (_brakeTestMonitor.isBraketestPostponeable())
        {
            options.add("postpone");
        }

        int response = _appUI.displayModalDialog(ApplicationDialogType.QUESTION, text, (String[]) options.toArray());
        switch (response)
        {
        case 0:
            _action = UserAction.START_BRAKETEST;
            break;
        case 1:
            _action = UserAction.POSTPONE_BRAKETEST;
            _brakeTestMonitor.postponeBrakeTest();
            break;
        default:
            _logger.error("Unexpected response!");
            break;
        }
    }

    @Override
    public void dispose() throws Exception
    {
        _brakeTestMonitor.removeMonitoringListener(this);

        // Dispose the additional single thread used to collect user feedback with dialog boxes.
        if (_workerToHandleEvent != null)
        {
            try
            {
                _workerToHandleEvent.shutdown();
                _workerToHandleEvent.awaitTermination(5, TimeUnit.SECONDS);
            }
            catch (InterruptedException e)
            {
                _logger.warn("Interruption in shutting down the event worker!", e);
            }
            _workerToHandleEvent.shutdownNow();
        }

        super.dispose();
    }

}
