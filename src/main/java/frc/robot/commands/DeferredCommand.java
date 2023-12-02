package frc.robot.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class DeferredCommand extends CommandBase {

  private final Supplier<Command> supplier;
  private Command command;

  public DeferredCommand(Supplier<Command> supplier, Subsystem... requirements) {
    this.supplier = requireNonNullParam(supplier, "supplier", "DeferredCommand");
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    command = supplier.get();
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }
}
