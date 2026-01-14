package frc.robot;

import com.teamscreamrobotics.physics.Trajectory;
import com.teamscreamrobotics.physics.Trajectory.GamePiece;

public class test {
    public static void main(String[] args) {
        Trajectory.configure()
            .setGamePiece(GamePiece.CUSTOM)
            .setCustomGamePiece(5, 0.37, 0.6)
            .setInitialHeight(5)
            .setShotVelocity(10)
            .setTargetDistance(9)
            .setTargetHeight(0);
        System.out.println(Trajectory.getOptimalAngle());
    }
}
