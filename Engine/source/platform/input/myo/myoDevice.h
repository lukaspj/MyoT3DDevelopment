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

#ifndef _MYODEVICE_H_
#define _MYODEVICE_H_

#include "platform/input/IInputDevice.h"
#include "platform/input/event.h"
#include "platformWin32/platformWin32.h"
#include "core/util/tSingleton.h"
#include "math/mQuat.h"

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

#define DEFAULT_MOTION_UNIT 0

struct LeapMotionDeviceData;

class MyoDevice : public IInputDevice
{
protected:

   // Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
   // provides several virtual functions for handling different kinds of events. If you do not override an event, the
   // default behavior is to do nothing.
   class MotionListener : public myo::DeviceListener
   {
   public:
      MotionListener()
         : roll_w(0), pitch_w(0), yaw_w(0), currentPose()
      {
         
      }
      virtual ~MotionListener() {}

      virtual void onPair(myo::Myo* myo, uint64_t timestamp);
      virtual void onConnect(myo::Myo* myo, uint64_t timestamp);
      virtual void onDisconnect(myo::Myo* myo, uint64_t timestamp);

      // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
      // as a quaternion.
      void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat);

      /// Called when a paired Myo has provided new accelerometer data in units of g.
      virtual void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel);

      /// Called when a paired Myo has provided new gyroscope data in units of deg/s.
      virtual void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro);

      // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
      // making a fist, or not making a fist anymore.
      void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);

      // We define this function to print the current values that were updated by the on...() functions above.
      void print();

      // These values are set by onOrientationData() and onPose() above.
      int roll_w, pitch_w, yaw_w;
      QuatF orientation;
      myo::Pose currentPose;
      myo::Pose oldPose;
   };

   /// The Myo hub, the Hub provides access to one or more Myos.
   myo::Hub mHub;

   /// The pointer to the Myo device.
   myo::Myo* mMyo;

   /// Our Leap Motion listener class
   MotionListener* mListener;

   /// Used with the LM listener object
   void* mActiveMutex;

   /// Is the Leap Motion active
   bool mActive;

   /// Buffer to store data Leap Motion data in a Torque friendly way
   LeapMotionDeviceData*  mDataBuffer[2];

   /// Points to the buffers that holds the previously collected data
   LeapMotionDeviceData*  mPrevData;

protected:
   /// Build out the codes used for controller actions with the
   /// Input Event Manager
   void buildCodeTable();

public:
   static bool smEnableDevice;

   // Indicates that events for each hand and pointable will be created
   static bool smGenerateIndividualEvents;

   // Indicates that we track hand IDs and will ensure that the same hand
   // will remain at the same index between frames.
   static bool smKeepHandIndexPersistent;

   // Indicates that we track pointable IDs and will ensure that the same
   // pointable will remain at the same index between frames.
   static bool smKeepPointableIndexPersistent;

   // Broadcast single hand rotation as axis
   static bool smGenerateSingleHandRotationAsAxisEvents;

   // The maximum hand angle when used as an axis event
   // as measured from a vector pointing straight up (in degrees)
   static F32 smMaximumHandAxisAngle;

   // Indicates that a whole frame event should be generated and frames
   // should be buffered.
   static bool smGenerateWholeFrameEvents;

   // Frame action codes
   static U32 MYO_FIST;       // SI_BUTTON
   static U32 MYO_SPREAD;     // SI_BUTTON
   static U32 MYO_TWIST_IN;   // SI_BUTTON
   static U32 MYO_WAVE_IN;    // SI_BUTTON
   static U32 MYO_WAVE_OUT;   // SI_BUTTON

   static U32 MYO_ORIENTATION; // SI_ROT

public:
   MyoDevice();
   ~MyoDevice();

   static void staticInit();

   bool enable();
   void disable();

   bool getActive();
   void setActive(bool state);

   bool process();

public:
   // For ManagedSingleton.
   static const char* getSingletonName() { return "MyoDevice"; }   
};

/// Returns the MyoDevice singleton.
#define MYODEV ManagedSingleton<MyoDevice>::instance()

#endif   // _MYODEVICE_H_
