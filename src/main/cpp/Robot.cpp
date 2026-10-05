#include <ctre/Phoenix.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Timer.h> // Required for checking spin-up delay

class Robot : public frc::TimedRobot {
  // Left-side Victor SPX motor controllers
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_leftFront{1};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_leftRear{0};

  // Right-side Victor SPX motor controllers
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rightFront{3};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rightRear{2};

  // Launcher Victor SPX motor controllers
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_launcherFlywheel{5};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_launcherIndexer{6};

  // DifferentialDrive sends commands to the two leader controllers.
  frc::DifferentialDrive m_robotDrive{
      [&](double output) { m_leftFront.Set(output); },
      [&](double output) { m_rightFront.Set(output); }};

  // The Logitech F310 is connected to Driver Station USB port 0.
  frc::XboxController m_controller{0};

  // Timer to track how long the flywheel has been spinning up
  frc::Timer m_launcherTimer;
  
  // Adjust this value! How many seconds the CIM motor needs to reach full speed.
  const units::time::second_t kFlywheelSpinUpTime = 1.0_s; 

 public:
  Robot() {
    // Restore the Victor SPXs to their default software settings.
    m_leftFront.ConfigFactoryDefault();
    m_leftRear.ConfigFactoryDefault();
    m_rightFront.ConfigFactoryDefault();
    m_rightRear.ConfigFactoryDefault();
    m_launcherFlywheel.ConfigFactoryDefault();
    m_launcherIndexer.ConfigFactoryDefault();

    // Configure each rear controller to follow its front controller.
    m_leftRear.Follow(m_leftFront);
    m_rightRear.Follow(m_rightFront);

    // One drivetrain side must be inverted because the motors on opposite sides face opposite directions.
    m_leftFront.SetInverted(false);
    m_leftRear.SetInverted(false);   

    m_rightFront.SetInverted(true);
    m_rightRear.SetInverted(true);   

    // Launcher Inversions (Change these if your motors run backward!)
    m_launcherFlywheel.SetInverted(false);
    m_launcherIndexer.SetInverted(false);

    // Use brake mode when the joysticks return to the center.
    m_leftFront.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_leftRear.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_rightFront.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_rightRear.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    
    // Flywheels usually perform best in Coast mode so they don't violently jerk the system when stopped
    m_launcherFlywheel.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    m_launcherIndexer.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    // Add the motor controllers as children of the drivetrain for dashboard and diagnostic information.
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftFront);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftRear);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightFront);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightRear);
  }

  void TeleopPeriodic() override {
    // ---------------- DRIVETRAIN CODE ----------------
    const double leftSpeed = -m_controller.GetLeftY();
    const double rightSpeed = -m_controller.GetRightY();
    m_robotDrive.TankDrive(leftSpeed, rightSpeed);

    // ---------------- LAUNCHER CODE ----------------
    if (m_controller.GetRightBumper()) {
      // --- MODE 1: FORWARD LAUNCH (Right Bumper) ---
      // Step 1: Turn on the flywheel immediately
      m_launcherFlywheel.Set(1.0);

      // Start the timer if it isn't running yet
      if (m_launcherTimer.Get() == 0_s) {
        m_launcherTimer.Start();
      }

      // Step 2: Once the flywheel has spun up long enough, turn on the indexer
      if (m_launcherTimer.Get() >= kFlywheelSpinUpTime) {
        m_launcherIndexer.Set(1.0); // Feeds object up
      } else {
        m_launcherIndexer.Set(0.0); // Still spinning up, keep indexer off
      }

    } else if (m_controller.GetLeftBumper()) {
      // --- MODE 2: REVERSE / UNJAM (Left Bumper) ---
      // Both motors immediately run backward at 0.25 speed, no delay
      m_launcherFlywheel.Set(-0.5);
      m_launcherIndexer.Set(-0.5);

      // Reset the launch timer so it's fresh if we switch back to launching
      m_launcherTimer.Stop();
      m_launcherTimer.Reset();

    } else {
      // --- MODE 3: IDLE (No Bumpers Pressed) ---
      m_launcherFlywheel.Set(0.0);
      m_launcherIndexer.Set(0.0);
      m_launcherTimer.Stop();
      m_launcherTimer.Reset();
    }
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
