package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestHang;

public class Test_Hang_Up_Command extends Command{
    TestHang test_Hang;

    public Test_Hang_Up_Command(TestHang testHang) {
        TestHang test_Hang = new TestHang();
        addRequirements(test_Hang);

    }
    @Override
    public void execute(){

        
    }

    @Override
    public void initialize(){

    }

    public void end(boolean isInterrupted){

    }
}