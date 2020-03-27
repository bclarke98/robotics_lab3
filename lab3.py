#!/usr/bin/env python3
import os
import sys
import cv2
import time
import math
import cozmo
import cozmo.util
import numpy as np

from fysom import Fysom
from cozmo.util import degrees, distance_inches, speed_mmps

class BoxAnnotator(cozmo.annotate.Annotator):
    cube = None
    imBox = None
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        if imBox is not None:
            cozmo.annotate.add_img_box_to_image(image, imBox, "green", text=None)
            BoxAnnotator.imBox = None
        elif BoxAnnotator.cube is not None:
            #double size of bounding box to match size of rendered image
            BoxAnnotator.cube = np.multiply(BoxAnnotator.cube,2)
            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BoxAnnotator.cube[0]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[1]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[2], BoxAnnotator.cube[2])
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)
            BoxAnnotator.cube = None

def oledImg(fn):
    image = Image.open(fn)
    resized = image.resize(cozmo.oled_face.dimensions(), Image.NEAREST)
    return cozmo.oled_face.convert_image_to_screen_data(resized, invert_image=True)

async def run(robot: cozmo.robot.Robot):
    robot.world.image_annotator.annotation_enabled = False
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True
    gain,exposure,mode = 390,3,1

    cubeOne = robot.world.define_custom_cube(CustomObjectTypes.CustomType02,
                                             CustomObjectMarkers.Circles4,
                                             44, 30, 30, True)
    cubeTwo = robot.world.define_custom_cube(CustomObjectTypes.CustomType06,
                                             CustomObjectMarkers.Diamonds4,
                                             44, 30, 30, True)

    # pseudo-global variables since we need to update these values from within
    # callback functions.
    # 'view' is the AR marked cube we see, 'num' is which cube we're finding
    gl = {
        'view': None,
        'num': 0
    }
    faceOne = oledImg('res/1.png')
    faceTwo = oledImg('res/2.png')
    faceThree = oledImg('res/3.png')

    async def updateFace(faceData, duration=0.5):
        await robot.display_oled_face_image(faceData, duration * 1000.0).wait_for_completed()

    def distance_from(p0, p1):
        rx,ry,rz = p0.pose.position.x_y_z
        tx,ty,tz = p1.position.x_y_z
        return math.sqrt(math.pow(rx - tx, 2) +  math.pow(ry - ty, 2))

    def angle_between(p0, p1):
        rx,ry,rz = p0.pose.position.x_y_z
        tx,ty,tz = p1.position.x_y_z
        rV = [rx, ry]
        tV = [tx, ty]
        rUnit = rV / np.linalg.norm(rV)
        tUnit = tV / np.linalg.norm(tV)
        dotp = np.dot(rUnit, tUnit)
        return np.arccos(dotp)

    async def on_search(e):
        # no need for a call to move() here since searching will be interrupted
        # by handle_object_appeared callback
        if e.src is not e.dst:
            print(f'Searching for cube {e.args[0]}')
            await robot.say_text('Target aquired').wait_for_completed()
            updateFace(faceOne)
            # robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
        await robot.turn_in_place(degrees(5)).wait_for_completed()

    async def on_move(e):
        if e.src is not e.dst:
            print(f'Moving to cube {e.args[0]}')
            await robot.say_text('Target aquired').wait_for_completed()
            await updateFace(faceTwo)
            # robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
        targ = e.args[0]
        deltapos = distance_from(robot, targ)
        ang = angle_between(robot, targ)
        if deltapos < 1:
            robot.stop_all_motors()
            await e.fsm.arrive() # transition to arrive state
        else:
            if not robot.is_moving:
                if abs(ang) > 2:
                    await robot.turn_in_place(degrees(ang)).wait_for_completed()
                robot.drive_wheel_motors(50, 50)

    async def on_arrive(e):
        if e.src is not e.dst:
            print(f'Arrived at cube {e.args[0]}')
            await robot.say_text('Target aquired').wait_for_completed()
            await updateFace(faceThree)
            # robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
        if not gl['num']:
            gl['num'] = 1
            await e.fsm.search()

    FSM = Fysom({
        'final': 'arrive2',
        'events': [
            {'name': 'search', 'src': 'none', 'dst': 'search1'},
            {'name': 'search', 'src': 'search1', 'dst': 'search1'},
            {'name': 'search', 'src': 'arrive1', 'dst': 'search2'},
            {'name': 'search', 'src': 'search2', 'dst': 'search2'},
            {'name': 'search', 'src': 'move1', 'dst': 'search1'},
            {'name': 'search', 'src': 'move2', 'dst': 'search2'},
            {'name': 'move', 'src': 'search1', 'dst': 'move1'},
            {'name': 'move', 'src': 'search2', 'dst': 'move2'},
            {'name': 'move', 'src': 'move1', 'dst': 'move1'},
            {'name': 'move', 'src': 'move2', 'dst': 'move2'},
            {'name': 'arrive', 'src': 'move1', 'dst': 'arrive1'},
            {'name': 'arrive', 'src': 'move2', 'dst': 'arrive2'}
        ],
        'callbacks': {
            'on_search': on_search, # runs after all FSM.search(...) calls
            'on_move': on_move,     # runs after all FSM.move(...) calls
            'on_arrive': on_arrive  # runs after all FSM.arrive(...) calls
        }
    })

    cubes = [cubeOne, cubeTwo]

    async def handle_object_appeared(evt, **kw):
        if isinstance(evt.obj, CustomObject):
            if evt.obj is cubes[gl['num']]:
                await FSM.move(evt.obj) # cozmo sees object

    async def handle_object_observed(evt, **kw):
        if isinstance(evt.obj, CustomObject):
            if evt.obj is cubes[gl['num']]:
                gl['view'] = evt.image_box
                await FSM.move(evt.obj)

    async def handle_object_disappeared(evt, **kw):
        if isinstance(evt.obj, CustomObject):
            await FSM.search() # cozmo lost sight of object
            gl['view'] = None

    robot.add_event_handler(cozmo.objects.EvtObjectAppeared, handle_object_appeared)
    robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, handle_object_disappeared)
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    try:
        await FSM.search()
        while True:
            #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
            if event.image is not None:
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)
                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,fixed_gain)
                BoxAnnotator.imBox = gl['view']

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
