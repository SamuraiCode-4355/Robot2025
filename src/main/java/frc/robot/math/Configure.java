package frc.robot.math;

public class Configure {
    
    private static int level;
    private static int side;
    private static boolean coral;
    private static boolean autoShoot = true;

    public static int getLevel(){
        return level;
    }

    public static void setLevel(int Level){

        level = Level;
    }

    public static int getSide(){

        return side;
    }

    public static void setSide(int Side){

        side = Side;
    }

    public static boolean getCoral(){

        return coral;
    }

    public static void setCoral(boolean Coral){

        coral = Coral;
    }

    public static boolean getAutoShoot(){

        return autoShoot;
    }

    public static void setAutoShoot(boolean AutoShoot){

        autoShoot = AutoShoot;
    }
}
