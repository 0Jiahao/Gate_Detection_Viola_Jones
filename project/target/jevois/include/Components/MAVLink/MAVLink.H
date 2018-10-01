// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2016 by Laurent Itti, the University of Southern
// California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
//
// This file is part of the JeVois Smart Embedded Machine Vision Toolkit.  This program is free software; you can
// redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software
// Foundation, version 2.  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details.  You should have received a copy of the GNU General Public License along with this program;
// if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
//
// Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, CA 90089-2520 - USA.
// Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*! \file */
#pragma once


#include <jevois/Component/Component.H>
#include <jevois/Core/Serial.H>
#include <jevois/Types/Enum.H>
#include "mavlink_utils.h"
#include <standard/mavlink.h>
#include <common/mavlink.h>
#include <memory>
#include "MavLinkMsgHandler.h"

//! MAVLink Integration
/*! "MAVLink is a very lightweight, header-only message marshalling library for micro air vehicles."

    This components integrates the MAVLink API into the Jevois' platform.

    When initialized as a subcomponent in a module, the shared ptr is passed again to set_instance to save the
    shared_ptr instance to a static global map, specifying if the component is connected to Serial-Over-USB or HARD
    Serial, the instance can be retrieved again by get_instance by specifying the same. This component instantiates a
    serial subcomponent of its own, so make sure to not use the intended serial port by Engine.

    Either USB or Hard serial can be used with this module, to use either one, it must not be used by engine. This is
    done by assigning an empty string to the respective serial name macro in Engine.h or in the params.cfg file.

    This has the  potential to allow for more than one mavlink instance to be used, on both USB and Hard serial, but
    note that jevois must have one serial terminal connected in order to run.

    --------
    You can daisy chain a jevois host and jevois platform to replicate MAVLink messages and have a debugger running
    on the host.

    Or, if you you two FTDI adapters you can loop back into the PC then run a jevois host instance and QGroundControl.

    You can connect jevois platform directly to QGroundControl, you should see the list of parameters
    and analyze the dummy output vision position values in a running plot.

    And of course, you can connect it to a mavlink capable autopilot once a useful vision processing module is
    incorporated. Refer to the mavlink repository https://github.com/mavlink/mavlink and website
    http://qgroundcontrol.org/mavlink/start to learn more on how to add specific message definitions in xml files and
    regenerate the mavlink header files.


    @author Ali AlSaibie

    @videomapping NONE 0 0 0 YUYV 640 480 30.0 JeVois MAVLinkTest
    @videomapping YUYV 640 480 30.0 YUYV 640 480 30.0 JeVois MAVLinkTest
    @email ali@alsaibie.com
    @address University of Southern California, HNB-07A, 3641 Watt Way, Los Angeles, CA 90089-2520, USA
    @copyright Copyright (C) 2016 by Laurent Itti, iLab and the University of Southern California
    @mainurl http://jevois.org
    @supporturl http://jevois.org/doc
    @otherurl http://iLab.usc.edu
    @license GPL v3
    @distribution Unrestricted
    @restrictions None
    \ingroup modules */


#ifdef JEVOIS_PLATFORM
// ! On platform hardware, device for the 4-pin hardware serial port
#define JEVOIS_SERIAL_DEFAULT "/dev/ttyS0"
#define JEVOIS_USBSERIAL_DEFAULT "/dev/ttyGS0"
#else
// ! On generic computer hardware, device for serial port should be the MAVLink USB Device
#define JEVOIS_SERIAL_DEFAULT "/dev/ttyUSB0"
#define JEVOIS_USBSERIAL_DEFAULT "/dev/ttyUSB0"
//TODO: Add UDP?
#endif

#define ONBOARD_PARAM_NAME_LENGTH  15
#define READ_BUFFER_SIZE 500
#define COMPONENT_ID MAV_COMP_ID_CAMERA
#define SYSTEM_ID 85
#define AUTOPILOT_TYPE MAV_AUTOPILOT_GENERIC

namespace mavlink {
    static jevois::ParameterCategory const ParamCateg("MAVLink Options");
    //! Parameter \relates mavlink
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(serialdev, std::string, "Hardware (4-pin connector) serial device name, "
            "or 'stdio' to use the console, or empty for no hardware serial port",
                                           JEVOIS_SERIAL_DEFAULT, ParamCateg);
    //! Parameter \relates mavlink
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(usbserialdev, std::string, "Over-the-USB serial device name, or empty",
                                           JEVOIS_USBSERIAL_DEFAULT, ParamCateg);
}

using namespace mavlink;

class MAVLink : public jevois::Component {
public:
    //! Enum for the interface type
    typedef enum class Type_t {
        Hard, USB
    };

    //! Constructor
    MAVLink(std::string const &instance, Type_t type);

    //! Destructor
    virtual ~MAVLink();

    //! Initialize, create the detector and read the config files
    void postInit() override;

    //! Un-initialize, nuke allocated resources
    void postUninit() override;


    //! System State
    void heartbeat(void);

    //! Receive
    void receive(void);

    // static to access from c function implementation
    static std::shared_ptr<MAVLink> get_instance(Type_t type);

    void set_instance(std::shared_ptr<MAVLink> &inst);

    mavlink_status_t *get_status();

    mavlink_message_t *get_buffer();

    // itsSerial is a Serial instance, can be either USB or Hard Serial. Instead of having
    // multiple itsSerials, we can have multiple MAVLinks and each with its single itsSerial
    std::shared_ptr<jevois::Serial> itsSerial;


    void setMsgHandler(MavLinkMsgHandler *handler){ msgHandler = handler;}
    mavlink_channel_t getChannel(){return mavlink_channel;}
private:
    uint8_t MAVLinkReceiveBuf[READ_BUFFER_SIZE];
    MavLinkMsgHandler *msgHandler;
protected:
    struct MAVLink_data_struct *itsMAVLinkData; // TODO: change to smart ptr
    mavlink_channel_t mavlink_channel;
    Type_t itsType;
}; // class MAVLink
