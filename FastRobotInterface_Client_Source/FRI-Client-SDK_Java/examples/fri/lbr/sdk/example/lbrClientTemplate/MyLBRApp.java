package fri.lbr.sdk.example.lbrClientTemplate;

import com.kuka.fri.lbr.clientsdk.base.ClientApplication;
import com.kuka.fri.lbr.clientsdk.connection.UdpConnection;

/**
 * Template implementation of a FRI client application.
 */
public class MyLBRApp
{
    private MyLBRApp()
    {
        // nothing to do
    }

    private static final int DEFAULT_PORTID = 30200;

    /**
     * @param argv
     *            the arguments
     */
    public static void main(String[] argv)
    {
        // create new client
        MyLBRClient client = new MyLBRClient();

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
        app.connect(DEFAULT_PORTID);

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
    }
}
