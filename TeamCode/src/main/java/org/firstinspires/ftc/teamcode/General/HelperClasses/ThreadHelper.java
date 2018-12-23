// -----------------------------------------------------
//    This is a simple implementation of Threads
//    I have made it so it's easier for everyone to
//    use threads in a program fast and easy
// -----------------------------------------------------

package org.firstinspires.ftc.teamcode.General.HelperClasses;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.General.Robot.RobotDrive;

public abstract class ThreadHelper implements Runnable {
    // Declaring a Thread object
    Thread thread;

    // Declaring some boolean variables to keep track of the state of the thread
    boolean running = true;
    boolean paused = true;

    // initialization function
    public void init() {
        thread = new Thread(this);
    }

    @Override
    public void run() {
        while (running) {
            if (!paused) {
                loop();
            }
        }
    }

    public abstract void loop();

    // Start the thread
    public void start() {
        thread.start();
        paused = false;
    }

    // Kill the thread
    public void kill() {
        running = false;
    }

    // Pause the thread
    public void pause() {
        paused = true;
    }

    // Resume the thread
    public void resume() {
        paused = false;
    }
}
