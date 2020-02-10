#include <exception>
#include <iostream>
#include <ostream>

#include <visionaray/thin_lens_camera.h>

// Private visionaray_common includes!
#include <common/config.h>

#include <common/manip/arcball_manipulator.h>
#include <common/manip/pan_manipulator.h>
#include <common/manip/zoom_manipulator.h>

#if VSNRAY_COMMON_HAVE_SDL2
#include <common/viewer_sdl2.h>
#else
#include <common/viewer_glut.h>
#endif

using namespace visionaray;

#if VSNRAY_COMMON_HAVE_SDL2
using ViewerBase = viewer_sdl2;
#else
using ViewerBase = viewer_glut;
#endif

struct InstanceViewer : ViewerBase
{
    InstanceViewer()
        : ViewerBase(800, 800, "InstanceViewer")
    {
    }

    thin_lens_camera cam;
};

int main(int argc, char** argv)
{
    InstanceViewer viewer;

    try
    {
        viewer.init(argc, argv);
    }
    catch (std::exception const& e)
    {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    viewer.add_manipulator(std::make_shared<arcball_manipulator>(viewer.cam, mouse::Left));
    viewer.add_manipulator(std::make_shared<pan_manipulator>(viewer.cam, mouse::Middle));
    // Additional "Alt + LMB" pan manipulator for setups w/o middle mouse button
    viewer.add_manipulator(std::make_shared<pan_manipulator>(viewer.cam, mouse::Left, keyboard::Alt));
    viewer.add_manipulator(std::make_shared<zoom_manipulator>(viewer.cam, mouse::Right));

    viewer.event_loop();
}
