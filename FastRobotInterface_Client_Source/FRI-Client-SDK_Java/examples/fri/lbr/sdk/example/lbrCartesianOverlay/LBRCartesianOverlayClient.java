package fri.lbr.sdk.example.lbrCartesianOverlay;

import java.util.logging.Logger;

import com.kuka.fri.api.monitoring.EFriSessionState;
import com.kuka.fri.lbr.clientsdk.clientLBR.LBRClient;

/**
 * Example client that can overlay interpolated Cartesian poses. During command(), a circular motion in the XY plane is
 * going to be performed, while the redundancy is slowly moved between -45 and 45 deg.
 */
public class LBRCartesianOverlayClient extends LBRClient
{
    //!< frequency (Hertz)
    private double _freqHz;

    //!< amplitude (radians)
    private double _amplRad;

    //!< filter coefficient
    private double _filterCoeff;

    //!< offset for current interpolation step (x direction)
    private double _offsetSin;

    //!< offset for current interpolation step (y direction)
    private double _offsetCos;

    //!< current phase
    private double _phi;

    //!< phase step width
    private double _stepWidth;

    //!< current redundancy value for Cartesian poses
    private double _redundancyValue;
    private boolean _increase = true;

    /**
     * Constructor.
     * 
     * @param freqHz
     *            frequency in Hertz
     * @param amplRad
     *            amplitude in radians
     * @param filterCoeff
     *            filter coefficient between 0 (filter off) and 1 (max filter)
     */
    public LBRCartesianOverlayClient(double freqHz, double amplRad, double filterCoeff)
    {
        _freqHz = freqHz;
        _amplRad = amplRad;
        _filterCoeff = filterCoeff;

        Logger.getAnonymousLogger().info("LBRCartesianOverlayClient initialized:\n"
                + "\tfrequency (Hz): " + _freqHz + "\n"
                + "\tamplitude (rad): " + _amplRad + "\n"
                + "\tfilterCoeff: " + _filterCoeff + "\n");
    }

    @Override
    public void onStateChange(EFriSessionState oldState, EFriSessionState newState)
    {
        switch (newState)
        {
        case MONITORING_READY:
            // (re)initialize sine parameters when entering Monitoring
            _offsetSin = 0.0;
            _offsetCos = 0.0;
            _phi = 0.0;
            _stepWidth = 2 * Math.PI * _freqHz * getRobotState().getSampleTime();
            break;
        case IDLE:
        case MONITORING_WAIT:
        default:
            break;
        }
    }

    @Override
    public void monitor()
    {
        _redundancyValue = getRobotState().getMeasuredRedundancyValue();

        Logger.getAnonymousLogger().info(String.format("Base --> TCP pose: %.3f / %.3f / %.3f, redundancy: %.3f",
                getRobotState().getMeasuredCartesianPose()[0], getRobotState().getMeasuredCartesianPose()[1],
                getRobotState().getMeasuredCartesianPose()[2], Math.toDegrees(_redundancyValue)));
    }

    /**
     * Callback for the FRI state 'Commanding Active'.
     */
    @Override
    public void command()
    {
        // calculate new offsets
        final double newOffsetSin = _amplRad * Math.sin(_phi);
        _offsetSin = _offsetSin * _filterCoeff + newOffsetSin * (1.0 - _filterCoeff);
        final double newOffsetCos = _amplRad * Math.cos(_phi);
        _offsetCos = _offsetCos * _filterCoeff + newOffsetCos * (1.0 - _filterCoeff);

        _phi += _stepWidth;
        if (_phi >= 2 * Math.PI)
        {
            _phi -= 2 * Math.PI;
        }

        // add offset to ipo cart pose for x and y directions
        double[] ipoCartPos = getRobotState().getIpoCartesianPose();

        ipoCartPos[0] += _offsetSin;
        ipoCartPos[1] += _offsetCos;

        // changing redundancy value between -45° <= red <= 45°
        if (_increase)
        {
            if (_redundancyValue < Math.toRadians(45.0))
            {
                _redundancyValue += Math.toRadians(0.1);
            }
            else
            {
                _increase = false;
            }
        }
        else
        {
            if (_redundancyValue > Math.toRadians(-45.0))
            {
                _redundancyValue -= Math.toRadians(0.1);
            }
            else
            {
                _increase = true;
            }
        }

        getRobotCommand().setCartesianPose(ipoCartPos, _redundancyValue);
    }
}
