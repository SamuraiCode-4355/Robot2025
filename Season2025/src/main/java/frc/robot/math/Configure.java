package frc.robot.math;

public class Configure {
    private static int side;
    private static int level;

    public static int getSide(){
        return side;
    }

    public static int getLevel(){
        return level;
    }

    public static void setSide(int Side){
        side=Side;
    }

    public static void setLevel(int Level){
        level=Level;
    }
}
