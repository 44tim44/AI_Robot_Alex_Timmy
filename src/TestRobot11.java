import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.util.*;

import static java.lang.Math.*;


public class TestRobot11
{
    private RobotCommunication robotcomm;  // communication drivers
    private static final double LOOKAHEAD = 0.8;
    private Deque<Position> pathStack;
    private double gamma = 0;


    /**
     * Create a robot connected to host "host" at port "port"
     * @param host normally http://127.0.0.1
     * @param port normally 50000
     */
    public TestRobot11(String host, int port)
    {
        robotcomm = new RobotCommunication(host, port);
    }

    /**
     * This simple main program creates a robot, sets up some speed and turning rate and
     * then displays angle and position for 16 seconds.
     * @param args         not used
     * @throws Exception   not caught
     */
    public static void main(String[] args) throws Exception
    {
        System.out.println("Creating Robot");
        TestRobot11 robot = new TestRobot11("http://127.0.0.1", 50000);
        robot.readFile();

        robot.run();
    }


    private void run() throws Exception
    {
        double heading;
        Position robotPos;

        System.out.println("Creating response");
        LocalizationResponse lr = new LocalizationResponse();

        System.out.println("Creating request");
        DifferentialDriveRequest dr = new DifferentialDriveRequest();

        // set up the request to move in a circle
        dr.setAngularSpeed(0);
        dr.setLinearSpeed(0);

        System.out.println("Start to move robot");
        int rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);

        while(pathStack!=null ) {

            robotcomm.getResponse(lr);
            heading = getHeadingAngle(lr);
            robotPos = new Position(lr.getPosition());
            Position goalPos = getGoalPosition(robotPos);

            if(goalPos != null){
                double [] goalPosRCS = toRCS(robotPos,goalPos,heading);
                double xP = goalPosRCS[0];
                double length = goalPosRCS[1];

                gamma = (2*xP)/(Math.pow(length,2));
                try
                {
                    Thread.sleep(100);
                }
                catch (InterruptedException ex) {}
            }

            // Set Speed
            dr.setAngularSpeed(0.5*(gamma));
            dr.setLinearSpeed(0.5);
            rc = robotcomm.putRequest(dr);

        }

        // set up request to stop the robot
        dr.setLinearSpeed(0);
        dr.setAngularSpeed(0);

        System.out.println("Stop robot");
        rc = robotcomm.putRequest(dr);
        System.out.println("Response code " + rc);

    }

    /**
     * Extract the robot heading from the response
     * @param lr
     * @return angle in degrees
     */
    double getHeadingAngle(LocalizationResponse lr)
    {
        double angle = lr.getHeadingAngle();
        //if(angle < -180){angle = angle + 360;}
        //if(angle > 180){angle = angle - 360;}
        return angle * 180 / Math.PI;
    }

    public Position getGoalPosition(Position roboPos)
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

    public double[] toRCS(Position pos, Position goalPos, double robotAngle) {
        double [] data = new double[2];

    //Calculate distance to the goal point from the robot
        double dx = goalPos.getX() - pos.getX();
        double dy = goalPos.getY() - pos.getY();
        double length = pythHyp(dx, dy);

    //Calculate the angle between the goal point and the world coordinate system
        double pointAngle = atan2(goalPos.getY() - pos.getY(), goalPos.getX() - pos.getX());

    //Calculate the angle between the goal point and the robot coordinate system
        double diffAngle = pointAngle - robotAngle;

    //Calculate the goal point 's y-coordinate relative to the robot' s coordinate system
        double yP = sin(diffAngle) / length;

        data[0] = dx;
        data[1] = length;

        return data;
    }

    public static double getDistance(double x0,double y0,double xp,double yp)
    {
        return sqrt((xp - x0)*(xp - x0) + (yp - y0)*(yp - y0));
    }

    public static double getBearing(double x0,double y0,double xp,double yp)
    {
        return (atan2(yp - y0, xp - x0))* 180 / Math.PI;
    }

    public void readFile() throws Exception{
        File pathFile = new File("D:/MRDS4/Robot_Java/AI_Robot_Alex_Timmy/out/production/Java_project/Path-around-bench-and-sofa.json");
        BufferedReader in = new BufferedReader(new InputStreamReader(
                new FileInputStream(pathFile)));
        ObjectMapper mapper2 = new ObjectMapper();
        // read the path from the file
        Collection<Map<String, Object>> data =
                (Collection<Map<String, Object>>) mapper2.readValue(in, Collection.class);
        int nPoints = data.size();
        Position [] path = new Position[nPoints];

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
        pathStack = new ArrayDeque<>(); //2 Create new stack
        for(int i = list.size() - 1; i >= 0; i--) {
            pathStack.add(list.get(i));
        }
        for(int i = path.length - 1; i >= 0; i--) {
            pathStack.add(path[i]);
        }
    }

    private double pythHyp(double x,double y) {
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }

}

