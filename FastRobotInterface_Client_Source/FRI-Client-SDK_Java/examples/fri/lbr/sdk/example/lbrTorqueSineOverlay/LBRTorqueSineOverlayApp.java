package fri.lbr.sdk.example.lbrTorqueSineOverlay;

import java.util.logging.Logger;

import com.kuka.fri.lbr.clientsdk.base.ClientApplication;
import com.kuka.fri.lbr.clientsdk.connection.UdpConnection;

/**
 * Implementation of a FRI client application.
 */
public class LBRTorqueSineOverlayApp
{
    private LBRTorqueSineOverlayApp()
    {
        // nothing to do
    }

    private static final int DEFAULT_PORTID = 30200;
    private static final double DEFAULT_FREQUENCY = 0.25;
    private static final double DEFAULT_AMPLITUDE = 15.0;
    private static final int DEFAULT_JOINTMASK = 0x8;

    /**
     * Main method.
     * 
     * @param argv
     *            command line arguments
     */
    public static void main(String[] argv)
    {
        if ((0 < argv.length) && ("help".equals(argv[0])))
        {
            Logger.getAnonymousLogger()
                    .info("\nKUKA LBR torque sine overlay example application\n\n\tCommand line arguments:");
            Logger.getAnonymousLogger().info("\t1) remote hostname (optional)");
            Logger.getAnonymousLogger().info("\t2) port ID (optional)");
            Logger.getAnonymousLogger().info("\t3) bit mask encoding of joint torques to be overlaid (optional)");
            Logger.getAnonymousLogger().info("\t4) sine frequency in Hertz (optional)");
            Logger.getAnonymousLogger().info("\t5) sine amplitude in Nm (optional)");
            return;
        }

        String hostname = (argv.length >= 1 && !argv[0].isEmpty()) ? argv[0] : null;
        int port = (argv.length >= 2 && !argv[1].isEmpty()) ? Integer.valueOf(argv[1]) : DEFAULT_PORTID;
        int jointMask = (argv.length >= 3 && !argv[2].isEmpty()) ? Integer.valueOf(argv[2]) : DEFAULT_JOINTMASK;
        double frequency = (argv.length >= 4 && !argv[3].isEmpty()) ? Double.valueOf(argv[3]) : DEFAULT_FREQUENCY;
        double amplitude = (argv.length >= 5 && !argv[4].isEmpty()) ? Double.valueOf(argv[4]) : DEFAULT_AMPLITUDE;

        Logger.getAnonymousLogger().info("Enter LBRTorqueSineOverlay Client Application");

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /**************************************************************************/

        // create new sine overlay client
        LBRTorqueSineOverlayClient client = new LBRTorqueSineOverlayClient(jointMask, frequency, amplitude);

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Configuration */
        /*                                                                         */
        /***************************************************************************/

        // create new udp connection
        UdpConnection connection = new UdpConnection();

        // pass connection and client to a new FRI client application
        ClientApplication app = new ClientApplication(connection, client);

        // connect client application to KUKA Sunrise controller
        app.connect(port, hostname);

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Execution mainloop */
        /*                                                                         */
        /***************************************************************************/

        // repeatedly call the step routine to receive and process FRI packets
        boolean success = true;
        while (success)
        {
            success = app.step();
        }

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Dispose */
        /*                                                                         */
        /***************************************************************************/

        // disconnect from controller
        app.disconnect();

        Logger.getAnonymousLogger().info("Exit LBRTorqueSineOverlay Client Application");
    }
}
