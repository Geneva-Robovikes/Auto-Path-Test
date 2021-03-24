package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopCommand extends CommandBase {
    RobotContainer container;

    Joystick joystick = new Joystick(0);

    public TeleopCommand(RobotContainer container) {
        this.container = container;
        addRequirements(container.getDrive());
    }

    @Override
    public void execute() {
        double stickX = joystick.getRawAxis(1);
        container.getDrive().leftMaster.set(ControlMode.PercentOutput, 0.2 * stickX);
        container.getDrive().rightMaster.set(ControlMode.PercentOutput, 0.2 * stickX);
    }

    @Override
    public void end(boolean interrupted) {
        container.getDrive().leftMaster.set(ControlMode.PercentOutput, 0);
        container.getDrive().rightMaster.set(ControlMode.PercentOutput, 0);
    }
}
