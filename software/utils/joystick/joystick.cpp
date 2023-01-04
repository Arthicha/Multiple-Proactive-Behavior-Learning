// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#include "joystick.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include "unistd.h"

Joystick::Joystick()
{
  openPath("/dev/input/js0");
}

Joystick::Joystick(int joystickNumber)
{
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath)
{
  openPath(devicePath);
}

Joystick::Joystick(std::string devicePath, bool blocking)
{
  openPath(devicePath, blocking);
}

void Joystick::openPath(std::string devicePath, bool blocking)
{
  // Open the device using either blocking or non-blocking
  _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
}

bool Joystick::sample(JoystickEvent* event)
{
  int bytes = read(_fd, event, sizeof(*event)); 

  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // Joystick instance is likely unusable
  return bytes == sizeof(*event);
}

bool Joystick::isFound()
{
  return _fd >= 0;
}

Joystick::~Joystick()
{
  close(_fd);
}

std::ostream& operator<<(std::ostream& os, const JoystickEvent& e)
{
  os << "type=" << static_cast<int>(e.type)
     << " number=" << static_cast<int>(e.number)
     << " value=" << static_cast<int>(e.value);
  return os;
}

//**********************************************************************//

bool Joystick::risingEdgeButton(int button){

    //when press the button
    if(buttonSignal == inactive){
        if(button == active){
            buttonSignal = active;
            return true;
        }
    }

    // when release the button
    else if(button == inactive){
        buttonSignal = inactive;
        return false;
    }

    // when hold the button
    else{
        return false;
    }

    // printf(x ? "true" : "false");
} // risingEdgeButton(int button)

//_____________________________________________________________________________//

bool Joystick::fallingEdgeButton(int button){
    //when press the button
    if(button == active){
        buttonSignal = active;
        return false;
    }

    // when release the button
    else if(button == inactive){
        if(buttonSignal == active){
            buttonSignal = inactive;
            return true;
        }
    }

    // when hold the button
    else{
        return false;
    }

    // printf(x ? "true" : "false");
} // fallingEdgeButton(int button)

//_____________________________________________________________________________//

float Joystick::analogJoy(int joy_stick_analog){
    float analog = (float)(joy_stick_analog/32769.0);
    if(abs(analog*10) < 0.001){
      analog = 0.0;
    }
    return analog;
}






