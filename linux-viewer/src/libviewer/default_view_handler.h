#ifndef __default_view_handler_h__
#define __default_view_handler_h__

#ifdef __cplusplus
extern "C" {
#endif

#include "viewer.h"

typedef struct _DefaultViewHandler DefaultViewHandler;
struct _DefaultViewHandler
{
    EventHandler ehandler;
    ViewHandler  vhandler;

    int width, height;
    double aspect_ratio;

    double fov_degrees;

    double last_mouse_x;
    double last_mouse_y;

    int projection_type;

    // these three variables determine the position and orientation of
    // the camera.
    double lookat[3];
    double eye[3];
    double up[3];

    // when there's a mouse press in the scene, we determine which
    // point in the scene is the one being manipulated. This
    // manipulation point is preserved under the mouse cursor during
    // any pan operations.
    double manipulation_point[4];

    // should be recomputed whenever look/eye/up is modified.  must be
    // kept in-sync with look/eye/up
    double model_matrix[16]; 

    // should be recomputed whenever camera is modified.
    double projection_matrix[16]; 

    int viewport[4];

    // these are used to implement follow: they correspond to the last
    // position of the object.  we manipulate the lookat/eye/up values
    // to make the position of the object invariant to motions of this
    // point.
    int have_last;
    double lastpos[3], lastquat[4];
};

DefaultViewHandler *default_view_handler_new(Viewer *viewer);

#ifdef __cplusplus
}
#endif

#endif
