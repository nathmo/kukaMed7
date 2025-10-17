package fri.lbr.sdk.example.transformationProvider;

import java.util.logging.Logger;

import com.kuka.fri.lbr.clientsdk.base.ClientApplication;
import com.kuka.fri.lbr.clientsdk.clientLBR.LBRClient;
import com.kuka.fri.lbr.clientsdk.connection.UdpConnection;

/**
 * Implementation of a FRI client application.
 */
public class TransformationProviderApp
{
    private TransformationProviderApp()
    {
        // nothing to do
    }

    private static final int DEFAULT_PORTID = 30200;

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
                    .info("\nKUKA TransformationProvider example application\n\n\tCommand line arguments:");
            Logger.getAnonymousLogger().info("\t1) remote hostname (optional)");
            Logger.getAnonymousLogger().info("\t2) port ID (optional)");
            return;
        }

        Logger.getAnonymousLogger().info("Enter TransformationProvider Client Application");

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /**************************************************************************/

        String hostname = (argv.length >= 1 && !argv[0].isEmpty()) ? argv[0] : null;
        int port = (argv.length >= 2 && !argv[1].isEmpty()) ? Integer.valueOf(argv[1]) : DEFAULT_PORTID;

        // create a new transformation provider client
        TransformationProviderClient trafoClient = new TransformationProviderClient();

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Configuration */
        /*                                                                         */
        /***************************************************************************/

        // create new udp connection
        UdpConnection connection = new UdpConnection();

        // create new robot client
        LBRClient client = new LBRClient();

        // pass connection and client to a new FRI client application
        ClientApplication app = new ClientApplication(connection, client, trafoClient);

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

        Logger.getAnonymousLogger().info("Exit TransformationProvider Client Application");
    }
}
