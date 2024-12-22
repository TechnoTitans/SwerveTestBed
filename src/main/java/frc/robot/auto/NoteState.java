package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NoteState {
    private static boolean hasNoteBool = true;
    public static final Trigger hasNote = new Trigger(() -> hasNoteBool);

    public static void setHasNote(final boolean hasNote) {
        hasNoteBool = hasNote;
    }

    public static Command setHasNoteCommand(final boolean hasNote) {
        return Commands.runOnce(() -> setHasNote(hasNote));
    }
}
