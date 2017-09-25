import java.io.*;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.nio.file.Path;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Map;

// Jar file for JSON support
import com.fasterxml.jackson.databind.*;

import static java.lang.Math.*;

/**
 * TestRobot6 interfaces to the (real or virtual) robot over a network connection.
 * It uses Java -> JSON -> HttpRequest -> Network -> DssHost32 -> Lokarria(Robulab) -> Core -> MRDS4
 *
 * @author thomasj
 */
public class TestRobot6
{
    private String host;                // host and port numbers
    private int port;
    private ObjectMapper mapper;        // maps between JSON and Java structures
    private TestRobot6 robot;
    private static final double LOOKAHEAD = 0.7;

    /**
     * Create a robot connected to host "host" at port "port"
     * @param host normally http://127.0.0.1
     * @param port normally 50000
     */
    public TestRobot6(String host, int port)
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
        double bearingAngle;
        double headingAngle;
        double newAngle;
        double [] position;
        LinkedList<Position> pathQueue = new LinkedList<Position>();
        Position roboPos;
        Position carrotPos;
        double dx;
        double dy;
        double distance;

        File pathFile = new File("C:/Users/alexn/Documents/GitHub/AI_Robot_Alex_Timmy/out/production/Java_project/Path-around-bench-and-sofa.json");
        BufferedReader in = new BufferedReader(new InputStreamReader(
                new FileInputStream(pathFile)));
        ObjectMapper mapper2 = new ObjectMapper();
        // read the path from the file
        Collection<Map<String, Object>> data =
                (Collection<Map<String, Object>>) mapper2.readValue(in, Collection.class);
        int nPoints = data.size();
        Position [] path = new Position[nPoints];

        System.out.println("Creating Robot");
        TestRobot6 robot = new TestRobot6("http://127.0.0.1", 50000);

        System.out.println("Creating response");
        LocalizationResponse lr = new LocalizationResponse();

        System.out.println("Creating request");
        DifferentialDriveRequest dr = new DifferentialDriveRequest();

        int index = 0;
        for (Map<String, Object> point : data)
        {
            Map<String, Object> pose = (Map<String, Object>)point.get("Pose");
            Map<String, Object> aPosition = (Map<String, Object>)pose.get("Position");
            double x = (Double)aPosition.get("X");
            double y = (Double)aPosition.get("Y");
            path[index] = new Position(x, y);
            pathQueue.add(new Position(x, y));
            //System.out.println(path[index].getX() + ", " + path[index].getY());
            index++;
        }
        System.out.println(pathQueue.size());
        //dr.setAngularSpeed(0);
        //dr.setLinearSpeed(0);

        //System.out.println("Start to move robot");
        //int rc = robot.putRequest(dr);
       // System.out.println("Response code " + rc);

        //robot.getResponse(lr);
       // angle = robot.getHeadingAngle(lr);
        //roboPos = new Position(robot.getPosition(lr));
        carrotPos = pathQueue.peek();

        while (!pathQueue.isEmpty()){
            try
            {
                Thread.sleep(100);
            }
            catch (InterruptedException ex) {}
            //dx = carrotPos.getX() - roboPos.getX();
            //dy = carrotPos.getY() - roboPos.getY();
            //distance = sqrt(dx*dx + dy*dy);
            robot.getResponse(lr);
            roboPos = new Position(robot.getPosition(lr));
            headingAngle = robot.getHeadingAngle(lr);

            while (roboPos.getDistanceTo(carrotPos) <= LOOKAHEAD){
                carrotPos = pathQueue.poll();
                System.out.println("!!!!!!!!!!!!!!!!!!!!!!!! distance =" + roboPos.getDistanceTo(carrotPos));
                System.out.println("carrot polled!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }

            bearingAngle = roboPos.getBearingTo(carrotPos);//Math.atan2(dy, dx);
            newAngle = bearingAngle - headingAngle;
            System.out.println(newAngle);
            if (Math.abs(newAngle) > Math.PI) {
                newAngle = newAngle - (2*Math.PI);
                newAngle = Math.abs(newAngle);
                dr.setAngularSpeed(1.5 * newAngle / Math.PI);
            }

            else if (newAngle < -(Math.PI)) {
                newAngle = newAngle + (2 *Math.PI);
                dr.setAngularSpeed(1.5 * newAngle / Math.PI);
            }
            else {
                dr.setAngularSpeed(1.5 * newAngle / Math.PI);
            }
            System.out.println(newAngle);
            /*if(Math.abs(newAngle) == 0) {
                newAngle = newAngle;
                //dr.setAngularSpeed(newAngle);
            }
            else if (Math.abs(newAngle) > Math.PI) {
                newAngle = newAngle - (2*Math.PI);
                newAngle = Math.abs(newAngle);
                newAngle = (3.7 * newAngle / Math.PI);
            }
            else if (newAngle < -(Math.PI)) {
                newAngle = newAngle + (2 *Math.PI);
                newAngle = (3.7 * newAngle / Math.PI);
            }
            else {
                newAngle = (3.7 * newAngle / Math.PI);
            }*/

            System.out.println("heading = " + headingAngle);
            System.out.println("bearing = " + bearingAngle);
            System.out.println("roboPos = " + roboPos.getX() + ", " + roboPos.getY());
            System.out.println("carrotPos = " + carrotPos.getX() + ", " + carrotPos.getY());
            System.out.println("distance =" + roboPos.getDistanceTo(carrotPos));
            System.out.println("newAngle = " + newAngle);

            dr.setLinearSpeed(0.4);

           // dr.setAngularSpeed(newAngle);
            int rc = robot.putRequest(dr);
        }
        dr.setAngularSpeed(0);
        dr.setLinearSpeed(0);
       int rc = robot.putRequest(dr);
    }
    /*private Position carrotPoint(LocalizationResponse lr,Queue<Position> pathQueue, double distance) {
        Position robotPos = new Position(lr.getPosition());
        Position carrot = null;
        while (!path.isEmpty() || path.size() > 1) {
            carrot = pathQueue.peek();
            if (robotPos.getDistanceTo(carrot) < distance) {
                pathQueue.poll();
            }
            else {
                return carrot;
            }

        }
        return carrot;/*
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

}

/**
 robotPos.getBearingTo(carrotPoint);
 System.out.println("SIZE: "+path.size());
 System.out.println("BEARING :" + bearingAngle);

 double directionError = bearingAngle - headingAngle;

 System.out.println("ERROR : " + directionError);


 if(directionError == 0) {
    dr.setAngularSpeed(directionError);
 }
 else if (Math.abs(directionError) > Math.PI) {
    directionError = directionError - (2*Math.PI);
    directionError = Math.abs(directionError);
    dr.setAngularSpeed(3.7 * directionError / Math.PI);
 }

 else if (directionError < -(Math.PI)) {
    directionError = directionError + (2 *Math.PI);
    dr.setAngularSpeed(3.7 * directionError / Math.PI);
 }
 else {
    dr.setAngularSpeed(3.7 * directionError / Math.PI);
 }
 */

