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

        if(Side == 1 || Side == 2)
            side = Side;
    }

    public static void setLevel(int Level){

        if(Level >= 1 && Level <= 3)
            level = Level;
    }
}
