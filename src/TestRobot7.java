import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.*;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.util.*;

import static java.lang.Math.*;

/**
 * Created by Timeout on 2017-09-04.
 */
public class TestRobot7 {

    private String host;                // host and port numbers
    private int port;
    private ObjectMapper mapper;        // maps between JSON and Java structures
    private TestRobot7 robot;
    private static final double LOOKAHEAD = 0.25;

    /**
     * Create a robot connected to host "host" at port "port"
     * @param host normally http://127.0.0.1
     * @param port normally 50000
     */
    public TestRobot7(String host, int port)
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
        double roboAngle;
        double [] position;
        Position roboPos;


        File pathFile = new File("D:/MRDS4/Robot_Java/AI_Robot_Alex_Timmy/out/production/Java_project/Path-around-bench-and-sofa.json");
        BufferedReader in = new BufferedReader(new InputStreamReader(
                new FileInputStream(pathFile)));
        ObjectMapper mapper2 = new ObjectMapper();
        // read the path from the file
        Collection<Map<String, Object>> data =
                (Collection<Map<String, Object>>) mapper2.readValue(in, Collection.class);
        int nPoints = data.size();
        Position [] path = new Position[nPoints];

        System.out.println("Creating Robot");
        TestRobot7 robot = new TestRobot7("http://127.0.0.1", 50000);

        System.out.println("Creating response");
        LocalizationResponse lr = new LocalizationResponse();

        System.out.println("Creating request");
        DifferentialDriveRequest dr = new DifferentialDriveRequest();


        // Read Path-file
        int index = 0;
        for (Map<String, Object> point : data)
        {
            Map<String, Object> pose = (Map<String, Object>)point.get("Pose");
            Map<String, Object> aPosition = (Map<String, Object>)pose.get("Position");
            double x = (Double)aPosition.get("X");
            double y = (Double)aPosition.get("Y");
            path[index] = new Position(x, y);
            index++;
        }

        // Convert path[] to Deque-Stack
        List<Position> list = Arrays.asList(path); //1 Convert to a List
        Deque<Position> pathStack = new ArrayDeque<>(); //2 Create new stack
        for(int i = list.size() - 1; i >= 0; i--) {
            pathStack.add(list.get(i));
        }
        for(int i = path.length - 1; i >= 0; i--) {
            pathStack.add(path[i]);
        }

        // Start robot with 0 speed
        dr.setAngularSpeed(0);
        dr.setLinearSpeed(0);
        System.out.println("Start to move robot");
        int rc = robot.putRequest(dr);
        System.out.println("Response code " + rc);

        // Get Robots current Position
        robot.getResponse(lr);
        position = robot.getPosition(lr);
        roboPos = new Position(position);

        while(pathStack!=null ) {

            try
            {
                Thread.sleep(10);
            }
            catch (InterruptedException ex) {}

            // Update robots current Position and Heading
            Position goalPos = getGoalPosition(pathStack,roboPos);
            robot.getResponse(lr);
            roboAngle = robot.getHeadingAngle(lr);
            position = robot.getPosition(lr);
            roboPos = new Position(position);


            // Calculate Curve
            double len = getDistance(roboPos.getX(), roboPos.getY(), goalPos.getX(), goalPos.getY());
            double pointAngle = getBearing(roboPos.getX(), roboPos.getY(), goalPos.getX(), goalPos.getY());

            if(pointAngle < -180){pointAngle = pointAngle + 360;}
            if(roboAngle < -180){roboAngle = roboAngle + 360;}
            if(pointAngle > 180){pointAngle = pointAngle - 360;}
            if(roboAngle > 180){roboAngle = roboAngle - 360;}

            double diffAngle = pointAngle - roboAngle;
            if(diffAngle < -180){diffAngle = diffAngle + 360;}
            if(diffAngle > 180){diffAngle = diffAngle - 360;}
            double relativeX = cos(diffAngle) * len;
            double gamma = (2 * relativeX) / (len * len);

            System.out.println("heading = " + roboAngle);
            System.out.println("position = " + position[0] + ", " + position[1]);
            System.out.println("goalposition = " + goalPos.getX() + ", " + goalPos.getY());
            System.out.println("point angle = " + pointAngle);
            System.out.println("diff angle = " + diffAngle);
            System.out.println("dist to gp = " + len);

            // Set Speed
            dr.setAngularSpeed(0.5*(-gamma));
            dr.setLinearSpeed(0.5);
            rc = robot.putRequest(dr);

        }
        // Stop robot
        dr.setAngularSpeed(0);
        dr.setLinearSpeed(0);
        rc = robot.putRequest(dr);

        // Update robots current Position and Heading
        robot.getResponse(lr);
        roboAngle = robot.getHeadingAngle(lr);
        position = robot.getPosition(lr);
        System.out.println("heading = " + roboAngle);
        System.out.println("position = " + position[0] + ", " + position[1]);
        roboPos = new Position(position);

    }


    /**
     * Extract the robot heading from the response
     * @param lr
     * @return angle in degrees
     */
    double getHeadingAngle(LocalizationResponse lr)
    {
        double angle = lr.getHeadingAngle();
        return angle * 180 / Math.PI;

        /*
        double e[] = lr.getOrientation();

        double angle = 2 * Math.atan2(e[3], e[0]);
        return angle * 180 / Math.PI;
        */
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

    public static Position getGoalPosition(Deque<Position> path, Position roboPos)
    {
        for(Position pos : path)
        {
            double dx = pos.getX() - roboPos.getX();
            double dy = pos.getY() - roboPos.getY();
            double len = sqrt(dx*dx + dy*dy);

            if(len >= LOOKAHEAD)
            {
                return pos;
            }
            else
            {
                path.pop();
            }

        }
        return null;
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
        return sqrt((x - newX) * (x - newX) + (y - newY) * (y - newY));
    }

    public static double getBearing(double x0,double y0,double xp,double yp)
    {
        return (2*atan2(yp - y0, xp - x0))* 180 / Math.PI;
    }
}
