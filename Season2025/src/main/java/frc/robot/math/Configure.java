package frc.robot.math;

public class Configure {
    
    private static byte side;
    private static byte level;

    public static byte getSide(){
        return side;
    }

    public static byte getLevel(){
        return level;
    }

    public static void setSide(byte Side){
        side = Side;
    }

    public static void setLevel(byte Level){
        level = Level;
    }
}
