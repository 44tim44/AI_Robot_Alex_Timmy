import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.util.*;

import static java.lang.Math.*;

/**
 * A path-following algorithm for a MRDS robot, which implements a Pure Pursuit algorithm
 */
public class TestRobot13
{
    private RobotCommunication robotcomm;           // Communication drivers
    private static final double LOOKAHEAD = 0.7;    // Lookahead-distance
    private Deque<Position> pathStack;              // Stack containing path to follow.
    private double gamma = 0;                       // Current angular speed

    //private String robotHost = "http://130.239.42.75";
    //private String filePath = "/Users/timmy/IdeaProjects/AI_Robot_Alex_Timmy/out/production/Java_project/Path-around-bench-and-sofa.json";
    private static final String robotHost = "http://127.0.0.1";
    //private static final String filePath = "D:/MRDS4/Java_project/out/production/Java_project/Path-around-bench-and-sofa.json";
    private static final String filePath = "C:/Users/alexn/Documents/GitHub/AI_Robot_Alex_Timmy/src/Path-from-bed.json";

    /**
     * Create a robot connected to host "host" at port "port"
     * @param host normally http://127.0.0.1
     * @param port normally 50000
     */
    @SuppressWarnings("WeakerAccess")
    public TestRobot13(String host, int port)
    {
        robotcomm = new RobotCommunication(host, port);
    }

    /**
     * This simple main program creates a robot, reads a path-file and starts the robot.
     * @param args          not used
     * @throws Exception    not caught
     */
    public static void main(String[] args) throws Exception
    {
        System.out.println("Creating Robot");
        TestRobot13 robot = new TestRobot13(robotHost, 50000);
        robot.readFile();
        long start = System.nanoTime();
        robot.run();
        long elapsedTime = System.nanoTime() - start;
        long timeMs = elapsedTime / 1000000;
        System.out.println("Total execution time : " + timeMs + " ms");
    }

    /**
     * Run method for robot
     * @throws Exception    not caught
     */
    private void run() throws Exception {
        double heading;
        Position robotPos;

        // Initialize the robot
        System.out.println("Creating response");
        LocalizationResponse lr = new LocalizationResponse();
        System.out.println("Creating request");
        DifferentialDriveRequest dr = new DifferentialDriveRequest();

        // Set up request to start the robot in a stopped position
        dr.setAngularSpeed(0);
        dr.setLinearSpeed(0);

        // Start the robot
        System.out.println("Start to move robot");
        int rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);

        while(!pathStack.isEmpty() ) {

            // Update current position information
            robotcomm.getResponse(lr);
            heading = getHeadingAngle(lr);
            robotPos = new Position(lr.getPosition());
            Position goalPos = getGoalPosition(robotPos);

            if(goalPos != null){
                //Calculate distance to the goal point from the robot
                double dx = goalPos.getX() - robotPos.getX();
                double dy = goalPos.getY() - robotPos.getY();
                double length = pythHyp(dx, dy);

                //Calculate the angle between the goal point and the world coordinate system
                double bearing = getBearing(robotPos.getX(),robotPos.getY(),goalPos.getX(),goalPos.getY());

                //Calculate the angle between the goal point and the robot coordinate system
                double diffAngle = bearing - heading;

                //Calculate the goal point 's y-coordinate relative to the robot' s coordinate system
                double yP = sin(diffAngle) / length;

                // Calculate curvature
                gamma = (2*yP)/(Math.pow(length,2));

                // Print info about current position, heading, etc.
                System.out.println("heading = " + heading);
                System.out.println("position = " + robotPos.getX() + ", " + robotPos.getY());
                System.out.println("goalposition = " + goalPos.getX() + ", " + goalPos.getY());
                System.out.println("bearing = " + bearing);
                System.out.println("DiffAngle = " + diffAngle);
                System.out.println("dist to gp = " + length);
                System.out.println("Gamma = " + gamma);

                // Wait 100 milliseconds
                Thread.sleep(10);
            }

            // Set Speed
            dr.setAngularSpeed(1.0*(gamma));
            dr.setLinearSpeed(1.0);
            robotcomm.putRequest(dr);

        }

        // Set up request to stop the robot
        dr.setLinearSpeed(0);
        dr.setAngularSpeed(0);

        // Stop the robot
        System.out.println("Stop robot");
        rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);

    }

    /**
     * Extract the robot heading from the response
     * @param lr LocalizationResponse
     * @return angle in radians
     */
    double getHeadingAngle(LocalizationResponse lr)
    {
        return lr.getHeadingAngle();
    }

    /**
     * Determine a goal-position at a set lookahead distance from the robot.
     * @param roboPos robots current position
     * @return a position for the robot to aim for
     */
    private Position getGoalPosition(Position roboPos)
    {
        for(Position pos : pathStack)
        {
            double dx = pos.getX() - roboPos.getX();
            double dy = pos.getY() - roboPos.getY();
            double len = sqrt(dx*dx + dy*dy);

            if (len < LOOKAHEAD) {
                pathStack.pop();
            } else {
                return pos;
            }
        }
        return null;
    }

    /**
     * Calculates distance between two sets of coordinates
     * @param x0 Origin X
     * @param y0 Origin Y
     * @param xp New X
     * @param yp New Y
     * @return Distance
     */
    public static double getDistance(double x0,double y0,double xp,double yp)
    {
        return sqrt((xp - x0)*(xp - x0) + (yp - y0)*(yp - y0));
    }

    /**
     * Extract the bearing towards some coordinates xp and yp
     * @param x0 Origin X
     * @param y0 Origin Y
     * @param xp New X
     * @param yp New Y
     * @return Angle in radians
     */
    public static double getBearing(double x0,double y0,double xp,double yp)
    {
        return (atan2(yp - y0, xp - x0));
    }


    /**
     * Reads a path, for the robot to follow, from a JSON-file and saves in a stack
     * @throws Exception    not caught
     */
    @SuppressWarnings({"unchecked", "WeakerAccess"})
    public void readFile() throws Exception
    {
        // Read the path from JSON-file
        File pathFile = new File(filePath);
        BufferedReader in = new BufferedReader(new InputStreamReader(
                new FileInputStream(pathFile)));
        ObjectMapper mapper2 = new ObjectMapper();

        // Save path-data to a Collection
        Collection<Map<String, Object>> data =
                (Collection<Map<String, Object>>) mapper2.readValue(in, Collection.class);
        int nPoints = data.size();
        Position [] path = new Position[nPoints];

        // Convert Collection to path[]
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
        pathStack = new ArrayDeque<>(); //2 Create new stack
        for(int i = 0; i < list.size() - 1; i++)
        {
            pathStack.add(list.get(i));
        }
    }

    /**
     * Pythagoras theorem for calculating the hypotenuse
     * @param x Cathetus 1
     * @param y Cathetus 2
     * @return The hypotenuse
     */
    private double pythHyp(double x,double y) {
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }

}

