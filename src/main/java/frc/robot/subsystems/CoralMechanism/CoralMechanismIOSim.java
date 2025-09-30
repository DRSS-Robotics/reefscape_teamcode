package frc.robot.subsystems.CoralMechanism;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import frc.commands.CoralAutoIntakeCommand;

public class CoralMechanismIOSim implements CoralMechanismIO {
    private final IntakeSimulation intakeSimulation;
    double intakeSpeed;

    //double speed;
    public CoralMechanismIOSim (AbstractDriveTrainSimulation driveTrain) {
        this.intakeSimulation = IntakeSimulation.InTheFrameIntake("Coral", driveTrain, Inches.of(7), IntakeSide.FRONT, 1);
    }

    @Override
    public void runIntake() {
        intakeSpeed = 1;
        System.out.println("Intake running");
    }

    public void runOuttake(){
        intakeSpeed = -1;
        System.out.println("Outtake running");
    }
    @Override
    public void setSpeed(double speed) {
        intakeSpeed  = speed;
    }
    @Override
    public void update(CoralMechanismIOInputs inputs) {
        boolean isReady;
        boolean hasCoral;

        if (intakeSpeed == 0){
            isReady = true;
            System.out.println("Isready");
        } else{
            isReady = false;
        }

        if (intakeSimulation != null){
            if (isReady && intakeSimulation.getGamePiecesAmount() == 1 ){
                 hasCoral = true;
            } else {
                    hasCoral = false;
                }

                if (hasCoral && isReady){
                    runOuttake();
                } else if (isReady && !hasCoral){
                    runIntake();
                } else{
                    //do nothing
                }
            }
        }
    }
