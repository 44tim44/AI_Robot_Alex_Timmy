import com.fasterxml.jackson.core.*;
import com.fasterxml.jackson.core.util.JsonParserSequence;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;


import java.io.*;
import java.math.BigDecimal;
import java.math.BigInteger;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;

/**
 * Created by Timeout on 2017-09-04.
 */
public class TestRobot3 {

    private String host;                // host and port numbers
    private int port;
    private ObjectMapper mapper;        // maps between JSON and Java structures
    private TestRobot3 robot;

    /**
     * Create a robot connected to host "host" at port "port"
     * @param host normally http://127.0.0.1
     * @param port normally 50000
     */
    public TestRobot3(String host, int port)
    {
        this.host = host;
        this.port = port;
        this.robot = this;

        mapper = new ObjectMapper();
    }

    /**
     * This simple main program creates a robot, sets up some speed and turning rate and
     * then displays angle and position for 16 seconds.
     * @param args         not used
     * @throws Exception   not caught
     */
    public static void main(String[] args) throws Exception
    {
        double angle;
        double [] position;

        JsonParser parser = new JsonParser();
        Object obj = parser.parse(new FileReader("D:/MRDS4/Java_project/out/production/Java_project/Path-around-bench-and-sofa.json"));
        JsonArray jsonArray = (JsonArray) obj;

        System.out.println("Creating Robot");
        TestRobot robot = new TestRobot("http://127.0.0.1", 50000);

        System.out.println("Creating response");
        LocalizationResponse lr = new LocalizationResponse();

        System.out.println("Creating request");
        DifferentialDriveRequest dr = new DifferentialDriveRequest();


        dr.setAngularSpeed(0);
        dr.setLinearSpeed(0);

        System.out.println("Start to move robot");
        int rc = robot.putRequest(dr);
        System.out.println("Response code " + rc);

        robot.getResponse(lr);
        angle = robot.getHeadingAngle(lr);
        System.out.println("heading = " + angle);

        position = robot.getPosition(lr);
        System.out.println("position = " + position[0] + ", " + position[1]);

        /*
        System.out.println("Start to move robot");
        int rc = robot.putRequest(dr);
        System.out.println("Response code " + rc);
        */

        for(JsonElement element : jsonArray){

            try
            {
                Thread.sleep(1000);
            }
            catch (InterruptedException ex) {}

            JsonObject jsonObject = element.getAsJsonObject();

            JsonObject pose = jsonObject.get("Pose").getAsJsonObject();
            JsonObject position1 = pose.get("Position").getAsJsonObject();

            double x = position1.get("X").getAsDouble();
            double y = position1.get("Y").getAsDouble();
            //double z = position1.get("Z").getAsDouble();

            while( getDistance(position[0],position[1], x, y) > 0.25)
            {
                while(getBearing(position[0],position[1], x, y) != angle)
                {
                    dr.setAngularSpeed(Math.PI * 0.05);
                    rc = robot.putRequest(dr);
                }
                dr.setAngularSpeed(0);
                dr.setLinearSpeed(0.1);
                rc = robot.putRequest(dr);
            }
        }

        // set up request to stop the robot
        dr.setLinearSpeed(0);
        dr.setAngularSpeed(0);

        System.out.println("Stop robot");
        rc = robot.putRequest(dr);
        System.out.println("Response code " + rc);

    }

    /**
     * Extract the robot heading from the response
     * @param lr
     * @return angle in degrees
     */
    double getHeadingAngle(LocalizationResponse lr)
    {
        double e[] = lr.getOrientation();

        double angle = 2 * Math.atan2(e[3], e[0]);
        return angle * 180 / Math.PI;
    }

    /**
     * Extract the position
     * @param lr
     * @return coordinates
     */
    double[] getPosition(LocalizationResponse lr)
    {
        return lr.getPosition();
    }


    /**
     * Send a request to the robot.
     * @param r request to send
     * @return response code from the connection (the web server)
     * @throws Exception
     */
    public int putRequest(Request r) throws Exception
    {
        URL url = new URL(host + ":" + port + r.getPath());

        HttpURLConnection connection = (HttpURLConnection)url.openConnection();

        connection.setDoOutput(true);

        connection.setRequestMethod("POST");
        connection.setRequestProperty("Content-Type", "application/json");
        connection.setUseCaches (false);

        OutputStreamWriter out = new OutputStreamWriter(
                connection.getOutputStream());

        // construct a JSON string
        String json = mapper.writeValueAsString(r.getData());

        // write it to the web server
        out.write(json);
        out.close();

        // wait for response code
        int rc = connection.getResponseCode();

        return rc;
    }

    /**
     * Get a response from the robot
     * @param r response to fill in
     * @return response same as parameter
     * @throws Exception
     */
    public Response getResponse(Response r) throws Exception
    {
        URL url = new URL(host + ":" + port + r.getPath());
        System.out.println(url);

        // open a connection to the web server and then get the resulting data
        URLConnection connection = url.openConnection();
        BufferedReader in = new BufferedReader(new InputStreamReader(
                connection.getInputStream()));

        // map it to a Java Map
        Map<String, Object> data = mapper.readValue(in, Map.class);
        r.setData(data);

        in.close();

        return r;
    }

    public static double getDistance(double x,double y,double newX,double newY)
    {
        return Math.sqrt((x - newX) * (x - newX) + (y - newY) * (y - newY));
    }

    public static double getBearing(double x,double y,double newX,double newY)
    {
        return Math.atan2(newY - y, newX - x);
    }
}
