//-----------------------------------------------------------------------------
// Copyright (c) 2012 GarageGames, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#include "platform/input/myo/myoDevice.h"
#include "platform/platformInput.h"
#include "core/module.h"
#include "platform/threads/mutex.h"
#include "console/engineAPI.h"
#include "math/mAngAxis.h"
#include "math/mMatrix.h"

MODULE_BEGIN(MyoDevice)

   MODULE_INIT_AFTER( InputEventManager )
   MODULE_SHUTDOWN_BEFORE( InputEventManager )

   MODULE_INIT
   {
      MyoDevice::staticInit();
      ManagedSingleton< MyoDevice >::createSingleton();
      if (MyoDevice::smEnableDevice)
      {
         MYODEV->enable();
      }

      // Register the device with the Input Event Manager
      INPUTMGR->registerDevice(MYODEV);
   }
   
   MODULE_SHUTDOWN
   {
      INPUTMGR->unregisterDevice(MYODEV);
      ManagedSingleton< MyoDevice >::deleteSingleton();
   }

MODULE_END;

//-----------------------------------------------------------------------------
// LeapMotionDevice
//-----------------------------------------------------------------------------

bool MyoDevice::smEnableDevice = true;

bool MyoDevice::smGenerateIndividualEvents = true;
bool MyoDevice::smKeepHandIndexPersistent = false;
bool MyoDevice::smKeepPointableIndexPersistent = false;

bool MyoDevice::smGenerateSingleHandRotationAsAxisEvents = false;

F32 MyoDevice::smMaximumHandAxisAngle = 25.0f;

bool MyoDevice::smGenerateWholeFrameEvents = false;

U32 MyoDevice::MYO_FIST = 0;
U32 MyoDevice::MYO_SPREAD = 0;
U32 MyoDevice::MYO_TWIST_IN = 0;
U32 MyoDevice::MYO_WAVE_IN = 0;
U32 MyoDevice::MYO_WAVE_OUT = 0;
U32 MyoDevice::MYO_ORIENTATION = 0;

MyoDevice::MyoDevice()
{
   // From IInputDevice
   dStrcpy(mName, "myo");
   mDeviceType = INPUTMGR->getNextDeviceType();

   mMyo = NULL;
   mListener = NULL;
   mActiveMutex = Mutex::createMutex();

   //
   mEnabled = false;
   mActive = false;

   mPrevData = mDataBuffer[0];

   buildCodeTable();
}

MyoDevice::~MyoDevice()
{
   disable();

   Mutex::destroyMutex(mActiveMutex);
}

void MyoDevice::staticInit()
{
   Con::addVariable("pref::LeapMotion::EnableDevice", TypeBool, &smEnableDevice, 
      "@brief If true, the Leap Motion device will be enabled, if present.\n\n"
	   "@ingroup Game");

   Con::addVariable("LeapMotion::GenerateIndividualEvents", TypeBool, &smGenerateIndividualEvents, 
      "@brief Indicates that events for each hand and pointable will be created.\n\n"
	   "@ingroup Game");
   Con::addVariable("LeapMotion::KeepHandIndexPersistent", TypeBool, &smKeepHandIndexPersistent, 
      "@brief Indicates that we track hand IDs and will ensure that the same hand will remain at the same index between frames.\n\n"
	   "@ingroup Game");
   Con::addVariable("LeapMotion::KeepPointableIndexPersistent", TypeBool, &smKeepPointableIndexPersistent, 
      "@brief Indicates that we track pointable IDs and will ensure that the same pointable will remain at the same index between frames.\n\n"
	   "@ingroup Game");

   Con::addVariable("LeapMotion::GenerateSingleHandRotationAsAxisEvents", TypeBool, &smGenerateSingleHandRotationAsAxisEvents, 
      "@brief If true, broadcast single hand rotation as axis events.\n\n"
	   "@ingroup Game");
   Con::addVariable("LeapMotion::MaximumHandAxisAngle", TypeF32, &smMaximumHandAxisAngle, 
      "@brief The maximum hand angle when used as an axis event as measured from a vector pointing straight up (in degrees).\n\n"
      "Shoud range from 0 to 90 degrees.\n\n"
	   "@ingroup Game");

   Con::addVariable("LeapMotion::GenerateWholeFrameEvents", TypeBool, &smGenerateWholeFrameEvents, 
      "@brief Indicates that a whole frame event should be generated and frames should be buffered.\n\n"
	   "@ingroup Game");
}

void MyoDevice::buildCodeTable()
{
   // Obtain all of the device codes
   MYO_FIST = INPUTMGR->getNextDeviceCode();
   MYO_SPREAD = INPUTMGR->getNextDeviceCode();
   MYO_TWIST_IN = INPUTMGR->getNextDeviceCode();
   MYO_WAVE_IN = INPUTMGR->getNextDeviceCode();
   MYO_WAVE_OUT = INPUTMGR->getNextDeviceCode();

   MYO_ORIENTATION = INPUTMGR->getNextDeviceCode();

   // Build out the virtual map
   AddInputVirtualMap(fist, SI_BUTTON, MYO_FIST);
   AddInputVirtualMap(spread, SI_BUTTON, MYO_SPREAD);
   AddInputVirtualMap(twist_in, SI_BUTTON, MYO_TWIST_IN);
   AddInputVirtualMap(wave_in, SI_BUTTON, MYO_WAVE_IN);
   AddInputVirtualMap(wave_out, SI_BUTTON, MYO_WAVE_OUT);

   AddInputVirtualMap(orientation, SI_ROT, MYO_ORIENTATION);
}

bool MyoDevice::enable()
{
   // Start off with disabling the device if it is already enabled
   disable();

   // Create the controller to talk with the Leap Motion along with the listener
   mListener = new MotionListener();

   // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
   // Hub::run() to send events to all registered device listeners.
   mHub.addListener(mListener);

   // We try to find a Myo (any Myo) that's nearby and connect to it. waitForAnyMyo() takes a timeout
   // value in milliseconds. In this case we will try to find a Myo for 10 seconds, and if that fails, the function
   // will return a null pointer.
   mMyo = mHub.waitForAnyMyo(10000);

   // If waitForAnyMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
   if (!mMyo) {
      throw std::runtime_error("Unable to find a Myo!");
   }

   // We've found a Myo, let's output its MAC address.
   //std::cout << "Connected to " << std::hex << std::setfill('0') << std::setw(12) << mMyo->macAddress() << std::dec
   //   << "." << std::endl << std::endl;

   // The device is now enabled but not yet ready to be used
   mEnabled = true;
   MYODEV->setActive(true);

   return false;
}

void MyoDevice::disable()
{
   if(mMyo)
   {
      mMyo = NULL;

      if(mListener)
      {
         mHub.removeListener(mListener);
         delete mListener;
         mListener = NULL;
      }
   }

   setActive(false);
   mEnabled = false;
}

bool MyoDevice::getActive()
{
   Mutex::lockMutex(mActiveMutex);
   bool active = mActive;
   Mutex::unlockMutex(mActiveMutex);

   return active;
}

void MyoDevice::setActive(bool state)
{
   Mutex::lockMutex(mActiveMutex);
   mActive = state;
   Mutex::unlockMutex(mActiveMutex);
}

bool MyoDevice::process()
{
   if(!mEnabled)
      return false;

   if(!getActive())
      return false;

   if (INPUTMGR)

   //TODO: Fetch data from the Myo here, or something similar.
   mHub.runOnce(0);

   if (mListener->oldPose == myo::Pose::fist && mListener->currentPose == myo::Pose::none ||
      mListener->currentPose == myo::Pose::fist && mListener->oldPose == myo::Pose::none)
      INPUTMGR->buildInputEvent(mDeviceType, DEFAULT_MOTION_UNIT, SI_BUTTON, MYO_FIST, mListener->currentPose == myo::Pose::fist ? SI_MAKE : SI_BREAK, mListener->currentPose == myo::Pose::fist ? 1.0f : 0.0f);
   if (mListener->oldPose == myo::Pose::fingers_spread && mListener->currentPose == myo::Pose::none ||
      mListener->currentPose == myo::Pose::fingers_spread && mListener->oldPose == myo::Pose::none)
      INPUTMGR->buildInputEvent(mDeviceType, DEFAULT_MOTION_UNIT, SI_BUTTON, MYO_SPREAD, mListener->currentPose == myo::Pose::fingers_spread ? SI_MAKE : SI_BREAK, mListener->currentPose == myo::Pose::fingers_spread ? 1.0f : 0.0f);
   if (mListener->oldPose == myo::Pose::twist_in && mListener->currentPose == myo::Pose::none ||
      mListener->currentPose == myo::Pose::twist_in && mListener->oldPose == myo::Pose::none)
      INPUTMGR->buildInputEvent(mDeviceType, DEFAULT_MOTION_UNIT, SI_BUTTON, MYO_TWIST_IN, mListener->currentPose == myo::Pose::twist_in ? SI_MAKE : SI_BREAK, mListener->currentPose == myo::Pose::twist_in ? 1.0f : 0.0f);
   if (mListener->oldPose == myo::Pose::wave_in && mListener->currentPose == myo::Pose::none ||
      mListener->currentPose == myo::Pose::wave_in && mListener->oldPose == myo::Pose::none)
      INPUTMGR->buildInputEvent(mDeviceType, DEFAULT_MOTION_UNIT, SI_BUTTON, MYO_WAVE_IN, mListener->currentPose == myo::Pose::wave_in ? SI_MAKE : SI_BREAK, mListener->currentPose == myo::Pose::wave_in ? 1.0f : 0.0f);
   if (mListener->oldPose == myo::Pose::wave_out && mListener->currentPose == myo::Pose::none ||
      mListener->currentPose == myo::Pose::wave_out && mListener->oldPose == myo::Pose::none)
      INPUTMGR->buildInputEvent(mDeviceType, DEFAULT_MOTION_UNIT, SI_BUTTON, MYO_WAVE_OUT, mListener->currentPose == myo::Pose::wave_out ? SI_MAKE : SI_BREAK, mListener->currentPose == myo::Pose::wave_out ? 1.0f : 0.0f);

   INPUTMGR->buildInputEvent(mDeviceType, DEFAULT_MOTION_UNIT, SI_ROT, MYO_ORIENTATION, SI_MOVE, QuatF(mListener->orientation));

   mListener->oldPose = mListener->currentPose;

   return true;
}

//-----------------------------------------------------------------------------
// MyoDevice::MotionListener
//-----------------------------------------------------------------------------

void MyoDevice::MotionListener::onPair(myo::Myo* myo, uint64_t timestamp)
{
   MYODEV->setActive(true);
}

void MyoDevice::MotionListener::onConnect(myo::Myo* myo, uint64_t timestamp)
{
   MYODEV->setActive(true);
}

void MyoDevice::MotionListener::onDisconnect(myo::Myo* myo, uint64_t timestamp)
{
   MYODEV->setActive(false);
}

void MyoDevice::MotionListener::onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
{
   using std::atan2f;
   using std::asinf;
   using std::sqrtf;

   // Calculate the normalized quaternion.
   float norm = sqrtf(quat.x() * quat.x() + quat.y() * quat.y() + quat.z() * quat.z() + quat.w() * quat.w());
   myo::Quaternion<float> normalized(quat.x() / norm, quat.y() / norm, quat.z() / norm, quat.w() / norm);

   orientation = QuatF(quat.x(), quat.y(), quat.z(), quat.w());

   // Calculate Euler angles (roll, pitch, and yaw) from the normalized quaternion.
   float roll = atan2f(2.0f * (normalized.w() * normalized.x() + normalized.y() * normalized.z()),
      1.0f - 2.0f * (normalized.x() * normalized.x() + normalized.y() * normalized.y()));
   float pitch = asinf(2.0f * (normalized.w() * normalized.y() - normalized.z() * normalized.x()));
   float yaw = atan2f(2.0f * (normalized.w() * normalized.z() + normalized.x() * normalized.y()),
      1.0f - 2.0f * (normalized.y() * normalized.y() + normalized.z() * normalized.z()));

   // Convert the floating point angles in radians to a scale from 0 to 20.
   roll_w = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
   pitch_w = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
   yaw_w = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);
}

void MyoDevice::MotionListener::onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel)
{
   //TODO: Parse the acceleration data.
}

void MyoDevice::MotionListener::onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro)
{
   //TODO: Parse the orientation data.
}

void MyoDevice::MotionListener::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
{
   currentPose = pose;

   // Vibrate the Myo whenever we've detected that the user has made a fist.
   if (pose == myo::Pose::fist) {
      myo->vibrate(myo::Myo::VibrationMedium);
   }
}

void MyoDevice::MotionListener::print()
{
   // Clear the current line
   std::cout << '\r';

   // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
   // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
   // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
   std::string poseString = currentPose.toString();

   // Output the current values
   std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
      << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
      << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']'
      << '[' << poseString << std::string(16 - poseString.size(), ' ') << ']'
      << std::flush;
}
//-----------------------------------------------------------------------------

DefineEngineFunction(isMyoActive, bool, (),,
   "@brief Used to determine if the Leap Motion input device is active\n\n"

   "The Leap Motion input device is considered active when the support library has been "
   "loaded and the device has been found.\n\n"

   "@return True if the Leap Motion input device is active.\n"

   "@ingroup Game")
{
   if(!ManagedSingleton<MyoDevice>::instanceOrNull())
   {
      return false;
   }

   return MYODEV->getActive();
}

DefineEngineFunction(angAxisToEuler, EulerF, (AngAxisF axis), , "")
{
   MatrixF mat;
   axis.setMatrix(&mat);
   return mat.toEuler();
}

DefineEngineFunction(eulerToAngAxis, AngAxisF, (EulerF axis),, "")
{
   return AngAxisF(axis);
}