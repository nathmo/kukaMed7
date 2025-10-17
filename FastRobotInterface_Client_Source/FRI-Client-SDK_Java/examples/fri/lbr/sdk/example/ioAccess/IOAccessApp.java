package fri.lbr.sdk.example.ioAccess;

import java.util.logging.Logger;

import com.kuka.fri.lbr.clientsdk.base.ClientApplication;
import com.kuka.fri.lbr.clientsdk.connection.UdpConnection;

/**
 * Implementation of a FRI client application.
 */
public class IOAccessApp
{
    private IOAccessApp()
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
            Logger.getAnonymousLogger().info("\nKUKA Fieldbus access example application\n\n\tCommand line arguments:");
            Logger.getAnonymousLogger().info("\t1) remote hostname (optional)");
            Logger.getAnonymousLogger().info("\t2) port ID (optional)");
            return;
        }

        // create new client
        IOAccessClient client = new IOAccessClient();
        Logger.getAnonymousLogger().info("Enter IOAccess Client Application");

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /**************************************************************************/

        String hostname = (argv.length >= 1 && !argv[0].isEmpty()) ? argv[0] : null;
        int port = (argv.length >= 2 && !argv[1].isEmpty()) ? Integer.valueOf(argv[1]) : DEFAULT_PORTID;

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

        Logger.getAnonymousLogger().info("Exit IOAccess Client Application");
    }
}
