package fri.lbr.sdk.example.lbrCartesianOverlay;

import java.util.logging.Logger;

import com.kuka.fri.lbr.clientsdk.base.ClientApplication;
import com.kuka.fri.lbr.clientsdk.connection.UdpConnection;

/**
 * Implementation of a FRI client application.
 */
public class LBRCartesianOverlayApp
{
    private static final int DEFAULT_PORTID = 30200;
    private static final double DEFAULT_FREQUENCY = 0.15;
    private static final double DEFAULT_AMPLITUDE = 30.0;
    private static final double DEFAULT_FILTER_COEFFICIENT = 0.99;

    private LBRCartesianOverlayApp()
    {
        // nothing to do
    }

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
                    .info("\nKUKA LBR Cartesian overlay example application\n\n\tCommand line arguments:");
            Logger.getAnonymousLogger().info("\t1) remote hostname (optional)");
            Logger.getAnonymousLogger().info("\t2) port ID (optional)");
            Logger.getAnonymousLogger().info("\t3) frequency in Hertz (optional)");
            Logger.getAnonymousLogger().info("\t4) amplitude in radians (optional)");
            Logger.getAnonymousLogger().info("\t5) filter coefficient from 0 (off) to 1 (optional)");
            return;
        }

        final String hostname = (argv.length >= 1 && !argv[0].isEmpty()) ? argv[0] : null;
        final int port = (argv.length >= 2 && !argv[1].isEmpty()) ? Integer.valueOf(argv[1]) : DEFAULT_PORTID;
        final double frequency = (argv.length >= 3 && !argv[2].isEmpty()) ? Double.valueOf(argv[2]) : DEFAULT_FREQUENCY;
        final double amplitude = (argv.length >= 4 && !argv[3].isEmpty()) ? Double.valueOf(argv[3]) : DEFAULT_AMPLITUDE;
        final double filterCoeff =
                (argv.length >= 5 && !argv[4].isEmpty()) ? Double.valueOf(argv[4]) : DEFAULT_FILTER_COEFFICIENT;

        Logger.getAnonymousLogger().info("Enter LBRCartesianOverlay Client Application");

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /**************************************************************************/

        // create new Cartesian overlay client
        LBRCartesianOverlayClient client = new LBRCartesianOverlayClient(frequency, amplitude, filterCoeff);

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

        Logger.getAnonymousLogger().info("Exit LBRCartesianOverlay Client Application");
    }
}
