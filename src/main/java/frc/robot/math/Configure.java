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

        if(Side == 1 || Side == 2)
            side = Side;
    }

    public static void setLevel(byte Level){

        if(Level >= 1 && Level <= 3)
            level = Level;
    }
}
