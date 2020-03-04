  
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>

#include <frc/smartdashboard/smartdashboard.h>

class Autonomous : public frc::TimedRobot {
public:
  Autonomous() {
    m_robotDrive.SetExpiration(0.1);
    m_timer.Start();
  }
   private:
   frc::PWMVictorSPX m_left{0};
  frc::PWMVictorSPX m_right{1};
     frc::DifferentialDrive m_robotDrive{m_left, m_right};
      frc::Timer m_timer;
  void RobotMove(int time, int param1, int param2){
    while(m_timer.Get() < time) {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(param1, param2);
    }

  }
};