#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "Constants.h"

class Robot: public IterativeRobot {
private:
	TalonSRX * _talon = new TalonSRX(1);
	Joystick * _joy = new Joystick(0);
	std::string _sb;

	int _loops = 0;

	void RobotInit() {
        
		_talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		_talon->SetSensorPhase(true);

		
    
		_talon->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon->ConfigPeakOutputForward(1, kTimeoutMs);
		_talon->ConfigPeakOutputReverse(-1, kTimeoutMs);
    
    
		_talon->Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs);
		_talon->Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
		_talon->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
    
	}
	
	void TeleopPeriodic() {
		
		double leftYstick = _joy->GetY();
		double motorOutput = _talon->GetActiveTrajectoryVelocity();
		
		_sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tspd:");
		_sb.append(std::to_string(_talon->GetSelectedSensorVelocity(kPIDLoopIdx)));
		
		
        	/* Speed mode */
			/* Convert 500 RPM to units / 100ms.
			 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
      
			double targetVelocity_UnitsPer100ms =  500 * 4096 / 600;
      //as per calculations above, this now goes at 500 RPM
			
      _talon->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms); 

		
			_sb.append("\terrNative:");
			_sb.append(std::to_string(_talon->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(targetVelocity_UnitsPer100ms));
        
		
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n",_sb.c_str());
		}
		_sb.clear();
	}
};
START_ROBOT_CLASS(Robot)