#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>

#include <myo/myo.hpp>


class DataCollector : public myo::DeviceListener
{
	/*bool onArm;
	myo::Arm whichArm;
	bool isUnlocked;
	int roll_w;
	int pitch_w;
	int yaw_w;
	myo::Pose currentPose;*/

public:

	DataCollector() :
		onArm(false),
		isUnlocked(false),
		roll_w(0),
		pitch_w(0),
		yaw_w(0),
		currentPose() {}

	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		//Clean up when myo is lost
		onArm = false;
		isUnlocked = false;
		roll_w = 0;
		pitch_w = 0;
		yaw_w = 0;
	}

	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float> &quat)
	{
		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles from the unit quaternion.
		float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
			1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
			1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

		//Convert angles to radiants
		roll_w = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
		pitch_w = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
		yaw_w = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);
	}

	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		currentPose = pose;

		if (pose != myo::Pose::unknown && pose != myo::Pose::rest)
		{
			//If it is a known which is not the rest pose then unlock and stay unlocked
			myo->unlock(myo::Myo::unlockHold);

			//Vibrate and notify human that pose has been registered
			myo->notifyUserAction();
		}
		else
		{
			//Stay unlocked while the poses are being performed. It takes time to capture
			//and perform the poses
			myo->unlock(myo::Myo::unlockTimed);
		}
	}

	//When the sync gesture is noticed
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection, float rotation, myo::WarmupState warmupState)
	{
		onArm = true;
		whichArm = arm;
	}

	//Myo moves from the calibrated position on the arm
	void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
	{
		onArm = false;
	}

	//Unlocked and ready to delive pose events
	void onUnlock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = true;
	}

	//Locked and no pose can be registered
	void onLock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = false;
	}

	void control()
	{
		// Clear the current line
		std::cout << '\r';
		if (currentPose == myo::Pose::fist)
		{
			std::cout << "Forward ";
		}

		if (currentPose == myo::Pose::fingersSpread || currentPose == myo::Pose::waveOut)
		{
			std::cout << "Reverse ";
		}

		if (yaw_w > 12 && yaw_w < 16)
		{
			std::cout << "Right";
		}

		if (yaw_w > 0 && yaw_w < 5)
		{
			std::cout << "Left";
		}

		if (yaw_w == 0)
		{
			std::cout << "Straight";
		}
	}

	void print()
	{
		// Clear the current line
		std::cout << '\r';

		// Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
		std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
			<< '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
			<< '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << yaw_w << ' ' << 18- yaw_w << ']';

		if (onArm) {
			// Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

			// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
			// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
			// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
			std::string poseString = currentPose.toString();

			std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
				<< '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
				<< '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
			
			std::cout << std::endl;

			if (currentPose == myo::Pose::fist)
			{
				std::cout << "Forward";
			}

			if (currentPose == myo::Pose::fingersSpread || currentPose == myo::Pose::waveOut)
			{
				std::cout << "Reverse";
			}

			if (yaw_w > 12 && yaw_w < 16)
			{
				std::cout << "Right";
			}
			else if (yaw_w >  1 &&  yaw_w < 5)
			{
				std::cout << "Left";
			}
			else 
			{
				std::cout << "Straight";
			}
		}

		std::cout << std::flush;
	}

	bool onArm;
	myo::Arm whichArm;
	bool isUnlocked;
	int roll_w;
	int pitch_w;
	int yaw_w;
	myo::Pose currentPose;

};

int main(int argc, char** argv)
{
	try
	{
		myo::Hub hub("com.example.mysphero");
		std::cout << "Connecting with Myo..." << std::endl;

		myo::Myo *myo = hub.waitForMyo(100000);

		if (!myo)
		{
			throw std::runtime_error("Unable to connect to myo");
		}
		
		//Myo found
		std::cout << "Connected to myo!" << std::endl;
		
		DataCollector listener;
		hub.addListener(&listener);

		while (true)
		{
			hub.run(1000 / 20);
			listener.control();	
		}

	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 0;
	}
}