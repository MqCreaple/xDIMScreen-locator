bl_info = {
    "name": "xDIMScreen locator addon",
    "blender": (3, 0, 0),
    "category": "Object",
}

import bpy
from bpy import context
from mathutils import *
import numpy as np
import socket
import json
import select

BIND_OBJ_NAME = "simple tag"   # TODO: give user the option to change the bind object name
WHEEL_SENSITIVITY = 50.0

class XDimScreenMoveObjectOperator(bpy.types.Operator):
    bl_idname = "object.xdimscreen_move"
    bl_label = "xDIMScreen Move Object Operator"

    _timer = None
    _instance = None   # keep the running instance

    # Class-level parameters
    tag_location_scale = 0.2

    # Object-level parameters
    sock = None
    buffer = ""
    obj = None
    tag_location_zero = None
    tag_rotation_zero_inv = None
    object_location_zero = None
    object_rotation_zero = None

    def modal(self, context, event):
        if self.sock != None:
            if event.type == "TIMER":
                # try to poll data from the socket
                try:
                    ready, _, _ = select.select([self.sock], [], [], 0)
                    if ready:
                        data = self.sock.recv(4096).decode("utf-8")
                        if not data:  # connection closed
                            self.report({"INFO"}, "Socket closed by remote")
                            return self.cancel(context)
                        self.buffer += data
                        while "\n" in self.buffer:
                            line, self.buffer = self.buffer.split("\n", 1)
                            self.handle_packet(context, json.loads(line.strip()))
                except Exception as e:
                    self.report({"ERROR"}, f"Socket error ({type(e)}): {e}")
                    return self.cancel(context)
                return {"RUNNING_MODAL"}
            elif event.type == "WHEELDOWNMOUSE":
                # Scroll wheel down to increase the scale
                XDimScreenMoveObjectOperator.tag_location_scale *= 1.0 + WHEEL_SENSITIVITY / 1024
                return {"RUNNING_MODAL"}
            elif event.type == "WHEELUPMOUSE":
                # Scroll wheel up to decrease the scale
                XDimScreenMoveObjectOperator.tag_location_scale *= 1.0 - WHEEL_SENSITIVITY / 1024
                return {"RUNNING_MODAL"}

        return {"PASS_THROUGH"}     # for all other types of events

    def handle_packet(self, context, packet: dict):
        try:
            if packet["name"] != BIND_OBJ_NAME:
                # Not the bound object
                return

            region_3d = next(area.spaces.active.region_3d for area in bpy.context.screen.areas if area.type == "VIEW_3D")
            view_location = region_3d.view_location
            view_rotation = region_3d.view_rotation
            eye_position = view_location + view_rotation @ Vector((0.0, 0.0, region_3d.view_distance))
            if self.obj == None:
                # Object not initalized. Initialize object.
                view_rotation_inv = view_rotation.inverted()
                # Initialize
                self.obj = context.active_object
                self.obj.rotation_mode = "QUATERNION"
                self.object_location_zero = view_rotation_inv @ (self.obj.location - eye_position)
                self.object_rotation_zero = view_rotation_inv @ self.obj.rotation_quaternion
                self.tag_location_zero = Vector(packet["transform"]["t"])
                rq = packet["transform"]["rq"]
                self.tag_rotation_zero_inv = Quaternion((rq[3], rq[0], rq[1], rq[2])) # Change from xyzw to wxyz
                self.tag_rotation_zero_inv.invert()
            else:
                # Use the stored intial position of the object and the tag to calculate position changes
                if packet["transform"]["t"]:
                    delta_location = Vector(packet["transform"]["t"]) - self.tag_location_zero
                    self.obj.location = view_rotation @ (self.object_location_zero + XDimScreenMoveObjectOperator.tag_location_scale * delta_location) \
                                        + eye_position
                if packet["transform"]["rq"]:
                    rq = packet["transform"]["rq"]
                    delta_rotation = self.tag_rotation_zero_inv @ Quaternion((rq[3], rq[0], rq[1], rq[2]))
                    self.obj.rotation_quaternion = view_rotation @ self.object_rotation_zero @ delta_rotation
        except Exception as e:
            self.report({"ERROR"}, f"Packet corrupted. Detail: \"{e}\"")
            return self.cancel(context)

    def execute(self, context):
        if XDimScreenMoveObjectOperator._instance != None and XDimScreenMoveObjectOperator._instance.sock != None:
            # Already running, stop it
            self.report({"INFO"}, "Stopping socket listener")
            return XDimScreenMoveObjectOperator._instance.cancel(context)

        self.report({"INFO"}, "Starting socket listener")
        # connect to the location server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setblocking(True)
        self.sock.connect(("127.0.0.1", 30002))
        self.sock.setblocking(False)  # Wait for the socket to connect before set blocking to False

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.025, window=context.window)  # poll at 40Hz
        wm.modal_handler_add(self)

        XDimScreenMoveObjectOperator._instance = self
        return {"RUNNING_MODAL"}

    def cancel(self, context):
        wm = context.window_manager
        if self._timer:
            wm.event_timer_remove(self._timer)
        if self.sock:
            self.sock.close()
            self.sock = None
        XDimScreenMoveObjectOperator._instance = None
        return {"CANCELLED"}

# --- Setup ---
addon_keymaps = []  # Records all keymaps used by this addon

def register():
    bpy.utils.register_class(XDimScreenMoveObjectOperator)
    
    # Add keymap entry
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon
    if kc:  # only if addon keymap is available
        # Ctrl+Shift+D to toggle 3D location
        km = kc.keymaps.new(name="3D View", space_type="VIEW_3D")
        kmi = km.keymap_items.new(XDimScreenMoveObjectOperator.bl_idname, "D", "PRESS", ctrl=True, shift=True)
        addon_keymaps.append((km, kmi))

def unregister():
    # Remove keymap
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()

    bpy.utils.unregister_class(XDimScreenMoveObjectOperator)

if __name__ == "__main__":
    register()
