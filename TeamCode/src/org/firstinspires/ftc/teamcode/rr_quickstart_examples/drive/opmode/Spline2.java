/*
This is a modified version of SplineTest.java from the ACME Robotics Road-Runner-Quickstart project.
Included with permission from ACME Robotics.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.rr_quickstart_examples.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr_quickstart_examples.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Spline2", group = "drive")
public class Spline2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajGetThePlatform = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(30, 12), 0)
                .build();

        drive.followTrajectory(trajGetThePlatform);

        if (isStopRequested()) return;

        sleep(1000);

        Trajectory trajBackThatUp = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(10, 6), Math.toRadians(90))
                .build();

        drive.followTrajectory(trajBackThatUp);

        if (isStopRequested()) return;

        sleep(1000);

        Trajectory trajGetToTheCorner = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(5, 24), Math.toRadians(90))
                .build();

        drive.followTrajectory(trajGetToTheCorner);

        if (isStopRequested()) return;

        sleep(1000);

        Trajectory trajGoCenterLine = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(2, -40), Math.toRadians(270))
                .build();

        drive.followTrajectory(trajGoCenterLine);

    }
}
