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

#include <jevois/Core/Module.H>
#include <jevois/Image/RawImageOps.H>
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <jevois/Debug/Timer.H>
#include <iostream>
#include "VJGateDetector.h"

// icon by Catalin Fertu in cinema at flaticon

//! JeVois sample module
/*! This module is provided as an example of how to create a new standalone module.

    JeVois provides helper scripts and files to assist you in programming new modules, following two basic formats:

    - if you wish to only create a single module that will execute a specific function, or a collection of such modules
      where there is no shared code between the modules (i.e., each module does things that do not relate to the other
      modules), use the skeleton provided by this sample module. Here, all the code for the sample module is compiled
      into a single shared object (.so) file that is loaded by the JeVois engine when the corresponding video output
      format is selected by the host computer.

    - if you are planning to write a collection of modules with some shared algorithms among several of the modules, it
      is better to first create machine vision Components that implement the algorithms that are shared among several of
      your modules. You would then compile all your components into a first shared library (.so) file, and then compile
      each module into its own shared object (.so) file that depends on and automatically loads your shared library file
      when it is selected by the host computer. The jevoisbase library and collection of components and modules is an
      example for how to achieve that, where libjevoisbase.so contains code for Saliency, ObjectRecognition, etc
      components that are used in several modules, and each module's .so file contains only the code specific to that
      module.

    @author Sample Author

    @videomapping YUYV 640 480 28.5 YUYV 640 480 28.5 SampleVendor CascadeDetector
    @email sampleemail\@samplecompany.com
    @address 123 First Street, Los Angeles, CA 90012
    @copyright Copyright (C) 2017 by Sample Author
    @mainurl http://samplecompany.com
    @supporturl http://samplecompany.com/support
    @otherurl http://samplecompany.com/about
    @license GPL v3
    @distribution Unrestricted
    @restrictions None */

class CascadeDetector : public jevois::Module
{ private:
    CascadeClassifier tl_light;
    CascadeClassifier tr_light;
    CascadeClassifier bl_light;
    CascadeClassifier br_light;
    
  public:
    //! Default base class constructor ok
    using jevois::Module::Module;

    // load the corner detector
    virtual void postInit() override {
        // load the trained model
        tl_light.load("/jevois/modules/MavLab/CascadeDetector/model/0.8_50_t_l_corner_bw.xml");
        tr_light.load("/jevois/modules/MavLab/CascadeDetector/model/0.8_50_t_r_corner_bw.xml");
        br_light.load("/jevois/modules/MavLab/CascadeDetector/model/0.8_50_b_r_corner_bw.xml");
        bl_light.load("/jevois/modules/MavLab/CascadeDetector/model/0.8_50_b_l_corner_bw.xml");
    }

    //! Virtual destructor for safe inheritance
    virtual ~CascadeDetector() { }

    //! Processing function
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {
      static jevois::Timer timer("processing");

      // Wait for next available camera image:
      jevois::RawImage const inimg = inframe.get(true);

      timer.start();

      // We only support YUYV pixels in this example, any resolution:
      inimg.require("input", inimg.width, inimg.height, V4L2_PIX_FMT_YUYV);

      // While we process it, start a thread to wait for out frame and paste the input into it:
      jevois::RawImage outimg;
      auto paste_fut = std::async(std::launch::async, [&]() {
        outimg = outframe.get();
        outimg.require("output", inimg.width, inimg.height, inimg.fmt);
        // Paste the current input image:
        jevois::rawimage::paste(inimg, outimg, 0, 0);
        jevois::rawimage::writeText(outimg, "Viola and Jones Corner Detector", 3, 3, jevois::yuyv::LightGreen);
        });      
      
      // converter the image type
      cv::Mat img, bw;
      img = jevois::rawimage::convertToCvBGR(inimg);
      cv::cvtColor(img, img, CV_BGR2YCrCb);
      cv::inRange(img, cv::Scalar(60, 130, 75),
                  cv::Scalar(145, 255, 125), bw);

      // to output image
      cv::cvtColor(bw,bw,CV_GRAY2BGR);
      paste_fut.get(); inframe.done();
      VJDetector(bw,tl_light,tr_light,br_light,bl_light);
      
      // Wait for paste thread to complete and let camera know we are done processing the input image:
      jevois::rawimage::convertCvBGRtoRawImage(bw,outimg,1);

      // Show informations
      
      std::string const & fpscpu = timer.stop();
      jevois::rawimage::writeText(outimg, fpscpu, 3, inimg.height - 13, jevois::yuyv::LightGreen);
      // jevois::rawimage::writeText(outimg,std::to_string(n_gates), 3, 13, jevois::yuyv::LightGreen);
      // Let camera know we are done processing the input image:

      // Send the output image with our processing results to the host over USB:
      outframe.send(); // NOTE: optional here, outframe destructor would call it anyway
    }
};

// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(CascadeDetector);
