#!/usr/bin/env python3
"""
Team Name: Donatello
Team Number: 26
Topic: Autonomous lane-following robot

Developed by the entire team
1) U2210088 Sarvar Islamov (Team Leader)
2) U2210013 Fuzaylkhon Abdurakhimov
3) U2210077 Gayday Aleksey
4) U2210086 Inomjonov Javohirbek
5) U2210099 Kalimullin Rinat
6) U2210122 Shovkatjon Komilov
7) U2210251 Javohirbek Xatamov

"""


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class LaneDriver:
    def __init__(self):
        rospy.init_node('lane_driver')
        self.bridge = CvBridge()
        self.image = None

        # HSV thresholds for yellow lane boundaries
        self.Y_H_LOW  = 23
        self.Y_H_HIGH = 65
        self.Y_S_LOW  = 40
        self.Y_S_HIGH = 255
        self.Y_L_LOW  = 117
        self.Y_L_HIGH = 255

        # HSV thresholds for white dashed highway lines
        self.W_H_LOW  = 0
        self.W_H_HIGH = 179
        self.W_S_LOW  = 0
        self.W_S_HIGH = 50
        self.W_L_LOW  = 208
        self.W_L_HIGH = 255

        # HSV thresholds for red parking square
        self.R_H_LOW  = 163
        self.R_H_HIGH = 179
        self.R_S_LOW  = 83
        self.R_S_HIGH = 255
        self.R_L_LOW  = 85
        self.R_L_HIGH = 255

        # Speed limits for different driving phases
        self.MAX_SPEED = 0.10
        self.MIN_SPEED = 0.04
        self.WHITE_SPEED = 0.10
        self.SEEK_RED_SPEED = 0.07
        self.PARK_APPROACH_SPEED = 0.04
        self.PARK_FINAL_PUSH_SPEED = 0.05

        # PD controller gains for each driving mode
        self.Kp = 0.004
        self.Kd = 0.006
        self.Kp_WHITE = 0.003
        self.Kd_WHITE = 0.005
        self.Kp_RED = 0.005

        self.MAX_ANGULAR = 1.5
        self.ROI_TOP_RATIO = 0.55              # use bottom 45% of frame for detection
        self.MIN_PIXELS = 200                  # min blob area for yellow line
        self.MIN_WHITE_PIXELS = 80             # min blob area for a white dash
        self.MIN_RED_PIXELS = 500              # min blob area to consider as red square
        self.MIN_LINE_SEPARATION = 200         # min px between left/right yellow lines
        self.DEADBAND = 10                     # ignore tiny errors near center
        self.LANE_OFFSET = 160                 # px offset from a single line to lane center

        self.MIN_DASH_BLOBS = 2                # min white blobs to enter highway mode
        self.WHITE_LOCK_FRAMES = 8             # hysteresis frames for white mode

        # T-junction triggers when no lane lines seen for this many frames
        self.T_DETECT_FRAMES = 25

        # Final mission (turn + park) only unlocks after passing the white highway.
        # Prevents false triggers during yellow lane navigation.
        self.MIN_WHITE_FRAMES_TO_UNLOCK = 15
        self.UNLOCK_COOLDOWN_FRAMES = 10

        # Left turn parameters
        # Phase A: open-loop commit, Phase B: closed-loop yellow-line avoidance
        self.TURN_PHASE_A_DURATION = 1.0
        self.TURN_PHASE_A_LINEAR   = 0.05
        self.TURN_PHASE_A_ANGULAR  = 0.5

        self.TURN_PHASE_B_DURATION = 3.0
        self.TURN_PHASE_B_LINEAR   = 0.06
        self.TURN_PHASE_B_BASE_ANGULAR = 0.35
        self.YELLOW_SAFETY_DIST = 100          # distance from center; yellow must stay outside
        self.Kp_TURN_AVOID = 0.012             # correction strength when yellow encroaches
        self.TURN_END_MIN_LINE_SEP = 250       # min line separation to consider turn complete

        # Red parking thresholds
        self.RED_AREA_PARKED   = 8000          # red big enough to switch to PARK mode
        self.RED_AREA_DONE     = 35000         # red filling frame = robot on top of it
        self.RED_CY_DONE_RATIO = 0.90          # red center this far down = on top
        self.RED_LOST_TIMEOUT  = 2.5
        self.FINAL_PUSH_DURATION = 1.5         # extra forward push to fully cover red

        self.TARGET_SMOOTH_WIN = 4
        self.ERROR_SMOOTH_WIN = 3

        # PD controller state
        self.last_error = 0.0
        self.last_target = None
        self.last_known_side = None
        self.last_white_side = None
        self.no_detection_count = 0
        self.error_history = []
        self.target_history = []

        # Mode tracking
        self.in_white = False
        self.white_lock = 0
        self.lost_lane_count = 0
        self.red_lost_start = None

        # Mission unlock state
        self.white_frame_count = 0
        self.final_stage_unlocked = False
        self.unlock_cooldown = 0

        # Mission state machine
        self.state = "LANE_YELLOW"
        self.state_start_time = rospy.get_time()
        self.turn_phase = "A"

        # Debug visualization data
        self.dbg_left_x = None
        self.dbg_right_x = None
        self.dbg_target = None
        self.dbg_error = 0
        self.dbg_linear = 0.0
        self.dbg_angular = 0.0
        self.dbg_yellow_mask = None
        self.dbg_white_mask = None
        self.dbg_red_mask = None
        self.dbg_mode = "INIT"
        self.dbg_white_count = 0
        self.dbg_red_x = None
        self.dbg_red_y = None
        self.dbg_red_area = 0
        self.dbg_active_color = "YELLOW"

        rospy.Subscriber('/camera/rgb/image_raw',
                         Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.on_shutdown(self.stop_robot)
        rospy.loginfo("Lane Driver started.")

    def image_cb(self, msg):
        """Receive camera frames and convert to OpenCV BGR."""
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Image error: %s" % e)

    def get_mask(self, img, h_low, h_high, s_low, s_high, l_low, l_high):
        """Build a binary mask for a color range in HSV with noise cleanup."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([h_low, s_low, l_low])
        upper = np.array([h_high, s_high, l_high])
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        # Open removes specks, close fills small gaps
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def count_blobs(self, mask, min_pixels):
        """Count connected components above a minimum area."""
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        count = 0
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_pixels:
                count += 1
        return count

    def find_red_square(self, red_mask):
        """Return centroid (cx, cy) and area of the largest red blob, or (None, None, 0)."""
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(red_mask, connectivity=8)
        best_area = 0
        best_idx = -1
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= self.MIN_RED_PIXELS and area > best_area:
                best_area = area
                best_idx = i

        if best_idx < 0:
            return None, None, 0

        cx = int(centroids[best_idx][0])
        cy = int(centroids[best_idx][1])
        return cx, cy, best_area

    def find_lines_in_mask(self, mask, min_pixels, line_separation, side_memory_attr):
        """
        Detect left/right lane lines from a mask using connected components.
        Returns (left_x, right_x). Falls back to single-line classification
        when only one line is visible (e.g. on curves).
        """
        h, w = mask.shape
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

        valid_blobs = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= min_pixels:
                cx = centroids[i][0]
                valid_blobs.append((cx, area))

        if len(valid_blobs) == 0:
            return None, None

        valid_blobs.sort(key=lambda b: b[0])

        if len(valid_blobs) == 1:
            return self._classify_single(valid_blobs[0][0], w, side_memory_attr)

        # Pick the two biggest blobs, then check they are far enough apart
        # to be truly two separate lines (otherwise it's one curved line split)
        valid_blobs.sort(key=lambda b: b[1], reverse=True)
        top2 = sorted(valid_blobs[:2], key=lambda b: b[0])
        x1, x2 = top2[0][0], top2[1][0]

        if (x2 - x1) >= line_separation:
            return int(x1), int(x2)
        else:
            largest = max(valid_blobs[:2], key=lambda b: b[1])
            return self._classify_single(largest[0], w, side_memory_attr)

    def _classify_single(self, cx, image_width, side_memory_attr):
        """
        Decide whether a single detected line is left or right.
        Uses memory of previous side to handle sharp curves where a line
        can drift across the image midpoint.
        """
        midpoint = image_width // 2
        last_side = getattr(self, side_memory_attr)

        if cx < midpoint:
            # Line is left of center but might still be the "right" line that curved over
            if last_side == 'right' and cx > midpoint * 0.4:
                return None, int(cx)
            setattr(self, side_memory_attr, 'left')
            return int(cx), None
        else:
            if last_side == 'left' and cx < midpoint * 1.6:
                return int(cx), None
            setattr(self, side_memory_attr, 'right')
            return None, int(cx)

    def find_white_lines_clustered(self, white_mask):
        """
        Group white dashes into left/right clusters by image midpoint
        and return the area-weighted centroid of each side.
        Dashes are discontinuous, so simple line detection won't work.
        """
        h, w = white_mask.shape
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(white_mask, connectivity=8)

        midpoint = w // 2
        left_blobs = []
        right_blobs = []

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < self.MIN_WHITE_PIXELS:
                continue
            cx = centroids[i][0]
            if cx < midpoint:
                left_blobs.append((cx, area))
            else:
                right_blobs.append((cx, area))

        left_x = None
        right_x = None

        if len(left_blobs) > 0:
            total_area = sum(b[1] for b in left_blobs)
            left_x = int(sum(b[0] * b[1] for b in left_blobs) / total_area)

        if len(right_blobs) > 0:
            total_area = sum(b[1] for b in right_blobs)
            right_x = int(sum(b[0] * b[1] for b in right_blobs) / total_area)

        return left_x, right_x

    def compute_target_yellow(self, left_x, right_x, image_width):
        """Compute steering target for yellow-lane mode."""
        image_center = image_width // 2

        if left_x is not None and right_x is not None:
            self.no_detection_count = 0
            self.last_known_side = 'both'
            return (left_x + right_x) // 2, "BOTH"

        # Single line visible: assume lane center is at fixed offset
        if left_x is not None:
            self.no_detection_count = 0
            return left_x + self.LANE_OFFSET, "LEFT_ONLY"

        if right_x is not None:
            self.no_detection_count = 0
            return right_x - self.LANE_OFFSET, "RIGHT_ONLY"

        # No lines: brief memory window before giving up
        self.no_detection_count += 1
        if self.no_detection_count < 30 and self.last_target is not None:
            return self.last_target, "LOST_MEMORY"
        return None, "LOST"

    def compute_target_white(self, left_x, right_x, image_width):
        """Compute steering target for white-dashed highway mode."""
        image_center = image_width // 2

        if left_x is not None and right_x is not None:
            return (left_x + right_x) // 2, "WHITE_BOTH"

        if left_x is not None:
            return left_x + self.LANE_OFFSET, "WHITE_LEFT"

        if right_x is not None:
            return right_x - self.LANE_OFFSET, "WHITE_RIGHT"

        if self.last_target is not None:
            return self.last_target, "WHITE_MEMORY"
        return image_center, "WHITE_CENTER"

    def reset_pd(self):
        """Clear PD history. Called on state transitions to avoid stale errors."""
        self.last_error = 0.0
        self.error_history = []
        self.target_history = []

    def state_transition(self, new_state):
        """Switch mission state and reset relevant counters."""
        rospy.loginfo("STATE: %s -> %s" % (self.state, new_state))
        self.state = new_state
        self.state_start_time = rospy.get_time()
        self.red_lost_start = None
        if new_state == "TURN_LEFT":
            self.turn_phase = "A"
        self.reset_pd()

    def execute_smart_turn(self, y_left, y_right, w):
        """
        Two-phase left turn:
          Phase A: open-loop arc to commit to the turn.
          Phase B: closed-loop, slows/corrects if a yellow line gets too close to center.
        Returns (linear, angular, status_string).
        """
        elapsed = rospy.get_time() - self.state_start_time
        image_center = w // 2

        if self.turn_phase == "A":
            if elapsed < self.TURN_PHASE_A_DURATION:
                return (self.TURN_PHASE_A_LINEAR,
                        self.TURN_PHASE_A_ANGULAR,
                        "TURN_A t=%.1f" % elapsed)
            else:
                self.turn_phase = "B"
                self.state_start_time = rospy.get_time()
                rospy.loginfo("TURN: Phase A -> Phase B")
                elapsed = 0

        # Turn complete: both lines visible and properly separated
        if (y_left is not None and y_right is not None and
                (y_right - y_left) >= self.TURN_END_MIN_LINE_SEP):
            rospy.loginfo("TURN: complete")
            return (0.0, 0.0, "TURN_DONE")

        if elapsed >= self.TURN_PHASE_B_DURATION:
            rospy.loginfo("TURN: phase B timeout")
            return (0.0, 0.0, "TURN_TIMEOUT")

        # Push harder left if right yellow encroaches, less if left yellow does
        base_angular = self.TURN_PHASE_B_BASE_ANGULAR
        avoid_correction = 0.0

        if y_right is not None:
            dist_right = y_right - image_center
            if dist_right < self.YELLOW_SAFETY_DIST:
                encroach = self.YELLOW_SAFETY_DIST - dist_right
                avoid_correction += self.Kp_TURN_AVOID * encroach

        if y_left is not None:
            dist_left = image_center - y_left
            if dist_left < self.YELLOW_SAFETY_DIST:
                encroach = self.YELLOW_SAFETY_DIST - dist_left
                avoid_correction -= self.Kp_TURN_AVOID * encroach

        angular = base_angular + avoid_correction
        angular = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular))

        # If yellow is dangerously close, almost stop forward motion (rotate in place)
        linear = self.TURN_PHASE_B_LINEAR
        if y_right is not None and (y_right - image_center) < 60:
            linear *= 0.3
        if y_left is not None and (image_center - y_left) < 60:
            linear *= 0.3

        return (linear, angular,
                "TURN_B t=%.1f L=%s R=%s adj=%+.2f" %
                (elapsed, str(y_left), str(y_right), avoid_correction))

    def control_loop(self, event):
        """Main control loop: detect colors, run state machine, publish velocity."""
        if self.image is None:
            return

        h, w = self.image.shape[:2]
        image_center = w // 2

        # Build masks for all three colors
        yellow_mask = self.get_mask(self.image,
                                     self.Y_H_LOW, self.Y_H_HIGH,
                                     self.Y_S_LOW, self.Y_S_HIGH,
                                     self.Y_L_LOW, self.Y_L_HIGH)
        white_mask = self.get_mask(self.image,
                                    self.W_H_LOW, self.W_H_HIGH,
                                    self.W_S_LOW, self.W_S_HIGH,
                                    self.W_L_LOW, self.W_L_HIGH)
        red_mask = self.get_mask(self.image,
                                  self.R_H_LOW, self.R_H_HIGH,
                                  self.R_S_LOW, self.R_S_HIGH,
                                  self.R_L_LOW, self.R_L_HIGH)

        self.dbg_yellow_mask = yellow_mask
        self.dbg_white_mask = white_mask
        self.dbg_red_mask = red_mask

        # Crop to bottom region of frame (where road is)
        roi_y = int(h * self.ROI_TOP_RATIO)
        yellow_roi = yellow_mask[roi_y:, :]
        white_roi = white_mask[roi_y:, :]

        white_count = self.count_blobs(white_roi, self.MIN_WHITE_PIXELS)
        self.dbg_white_count = white_count
        y_left, y_right = self.find_lines_in_mask(
            yellow_roi, self.MIN_PIXELS, self.MIN_LINE_SEPARATION, 'last_known_side')

        red_cx, red_cy, red_area = self.find_red_square(red_mask)
        self.dbg_red_x = red_cx
        self.dbg_red_y = red_cy
        self.dbg_red_area = red_area

        # ---------- STATE: DONE ----------
        if self.state == "DONE":
            self.publish_cmd(0.0, 0.0)
            self.dbg_mode = "DONE"
            self.dbg_linear = 0.0
            self.dbg_angular = 0.0
            return

        # ---------- STATE: TURN_LEFT ----------
        if self.state == "TURN_LEFT":
            linear, angular, status = self.execute_smart_turn(y_left, y_right, w)
            self.dbg_left_x = y_left
            self.dbg_right_x = y_right
            self.dbg_mode = status
            self.dbg_linear = linear
            self.dbg_angular = angular

            if status in ("TURN_DONE", "TURN_TIMEOUT"):
                self.state_transition("SEEK_RED")
                return

            self.publish_cmd(linear, angular)
            return

        # ---------- STATE: SEEK_RED ----------
        # Cruise forward, steering toward red blob when visible
        if self.state == "SEEK_RED":
            self.dbg_active_color = "RED"
            if red_cx is not None:
                error = red_cx - image_center
                self.dbg_error = int(error)

                if red_area >= self.RED_AREA_PARKED:
                    self.state_transition("PARK")
                else:
                    angular = -self.Kp_RED * error
                    angular = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular))
                    self.publish_cmd(self.SEEK_RED_SPEED, angular)
                    self.dbg_mode = "SEEK_RED area=%d" % red_area
                    self.dbg_target = red_cx
                    self.dbg_linear = self.SEEK_RED_SPEED
                    self.dbg_angular = angular
                    return
            else:
                self.publish_cmd(self.SEEK_RED_SPEED, 0.0)
                self.dbg_mode = "SEEK_RED (no red yet)"
                self.dbg_target = image_center
                self.dbg_linear = self.SEEK_RED_SPEED
                self.dbg_angular = 0.0
                return

        # ---------- STATE: PARK ----------
        # Slow approach centered on red until stop conditions are met
        if self.state == "PARK":
            if red_cx is not None:
                self.red_lost_start = None

                error = red_cx - image_center
                self.dbg_error = int(error)
                self.dbg_target = red_cx

                # Stop when red fills the frame OR has reached bottom of view
                red_huge      = red_area >= self.RED_AREA_DONE
                red_at_bottom = red_cy   >= int(h * self.RED_CY_DONE_RATIO)

                if red_huge or red_at_bottom:
                    rospy.loginfo("PARK: stop condition met -> FINAL_PUSH")
                    self.state_transition("FINAL_PUSH")
                    return

                angular = -self.Kp_RED * error
                angular = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular))
                self.publish_cmd(self.PARK_APPROACH_SPEED, angular)
                self.dbg_mode = "PARK area=%d/%d cy=%d/%d" % (
                    red_area, self.RED_AREA_DONE,
                    red_cy, int(h * self.RED_CY_DONE_RATIO))
                self.dbg_linear = self.PARK_APPROACH_SPEED
                self.dbg_angular = angular
                return
            else:
                # Red disappeared from view = robot is now over it
                if self.red_lost_start is None:
                    self.red_lost_start = rospy.get_time()
                    rospy.loginfo("PARK: red disappeared -> FINAL_PUSH")
                    self.state_transition("FINAL_PUSH")
                    return

                self.publish_cmd(self.PARK_APPROACH_SPEED, 0.0)
                self.dbg_mode = "PARK (red gone)"
                self.dbg_linear = self.PARK_APPROACH_SPEED
                self.dbg_angular = 0.0
                return

        # ---------- STATE: FINAL_PUSH ----------
        # Short forward push to fully position robot on top of the red square
        if self.state == "FINAL_PUSH":
            elapsed = rospy.get_time() - self.state_start_time
            if elapsed < self.FINAL_PUSH_DURATION:
                self.publish_cmd(self.PARK_FINAL_PUSH_SPEED, 0.0)
                self.dbg_mode = "FINAL_PUSH t=%.1f/%.1f" % (elapsed, self.FINAL_PUSH_DURATION)
                self.dbg_linear = self.PARK_FINAL_PUSH_SPEED
                self.dbg_angular = 0.0
                return
            else:
                self.publish_cmd(0.0, 0.0)
                self.state_transition("DONE")
                rospy.loginfo("*** PARKING COMPLETE ***")
                return

        # ---------- Regular lane following below ----------

        # T-junction detection: count frames with no visible lane lines.
        # Only allowed to trigger after the highway section has been completed.
        yellow_seen = (y_left is not None) or (y_right is not None)
        if not yellow_seen and white_count == 0:
            self.lost_lane_count += 1
        else:
            self.lost_lane_count = 0

        if (self.final_stage_unlocked
                and self.unlock_cooldown == 0
                and self.lost_lane_count >= self.T_DETECT_FRAMES):
            rospy.loginfo("T-JUNCTION DETECTED -> TURN_LEFT")
            self.state_transition("TURN_LEFT")
            return

        # Decide between yellow and white modes with hysteresis
        yellow_both_visible = (y_left is not None and y_right is not None)

        if white_count >= self.MIN_DASH_BLOBS and not yellow_both_visible:
            self.in_white = True
            self.white_lock = self.WHITE_LOCK_FRAMES
        elif yellow_both_visible:
            self.in_white = False
            self.white_lock = 0
        elif self.in_white and self.white_lock > 0:
            self.white_lock -= 1
        else:
            self.in_white = False

        # Mission unlock: enough frames in white = highway section was traversed.
        # Cooldown after leaving white prevents a false T-junction trigger.
        if self.in_white:
            self.white_frame_count += 1
            if (not self.final_stage_unlocked
                    and self.white_frame_count >= self.MIN_WHITE_FRAMES_TO_UNLOCK):
                self.final_stage_unlocked = True
                rospy.loginfo("*** FINAL STAGE UNLOCKED ***")
        else:
            if self.final_stage_unlocked and self.unlock_cooldown == 0 and self.white_frame_count > 0:
                if not yellow_both_visible:
                    self.unlock_cooldown = self.UNLOCK_COOLDOWN_FRAMES
            if self.unlock_cooldown > 0:
                self.unlock_cooldown -= 1

        # ---------- WHITE highway mode ----------
        if self.in_white:
            self.state = "LANE_WHITE"
            self.dbg_active_color = "WHITE"
            w_left, w_right = self.find_white_lines_clustered(white_roi)
            target, mode = self.compute_target_white(w_left, w_right, w)

            self.dbg_left_x = w_left
            self.dbg_right_x = w_right
            self.dbg_mode = mode

            # Smooth target over a few frames to reduce jitter from flickering dashes
            self.target_history.append(target)
            if len(self.target_history) > self.TARGET_SMOOTH_WIN:
                self.target_history.pop(0)
            smoothed_target = int(sum(self.target_history) / len(self.target_history))
            self.last_target = smoothed_target
            self.dbg_target = smoothed_target

            error = smoothed_target - image_center
            self.error_history.append(error)
            if len(self.error_history) > self.ERROR_SMOOTH_WIN:
                self.error_history.pop(0)
            smoothed_error = sum(self.error_history) / len(self.error_history)

            derivative = smoothed_error - self.last_error
            self.last_error = smoothed_error

            if abs(smoothed_error) < self.DEADBAND:
                angular = 0.0
            else:
                angular = -(self.Kp_WHITE * smoothed_error + self.Kd_WHITE * derivative)
                angular = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular))

            self.publish_cmd(self.WHITE_SPEED, angular)
            self.dbg_error = int(smoothed_error)
            self.dbg_linear = self.WHITE_SPEED
            self.dbg_angular = angular
            return

        # ---------- YELLOW lane mode (default) ----------
        self.state = "LANE_YELLOW"
        self.dbg_active_color = "YELLOW"
        target, mode = self.compute_target_yellow(y_left, y_right, w)

        self.dbg_left_x = y_left
        self.dbg_right_x = y_right
        self.dbg_mode = mode

        if target is None:
            self.publish_cmd(0.0, 0.0)
            self.dbg_target = None
            self.dbg_linear = 0.0
            self.dbg_angular = 0.0
            return

        # Smooth target and error to suppress noise
        self.target_history.append(target)
        if len(self.target_history) > self.TARGET_SMOOTH_WIN:
            self.target_history.pop(0)
        smoothed_target = int(sum(self.target_history) / len(self.target_history))
        self.last_target = smoothed_target
        self.dbg_target = smoothed_target

        error = smoothed_target - image_center
        self.error_history.append(error)
        if len(self.error_history) > self.ERROR_SMOOTH_WIN:
            self.error_history.pop(0)
        smoothed_error = sum(self.error_history) / len(self.error_history)

        derivative = smoothed_error - self.last_error
        self.last_error = smoothed_error

        if abs(smoothed_error) < self.DEADBAND:
            angular = 0.0
        else:
            angular = -(self.Kp * smoothed_error + self.Kd * derivative)
            angular = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular))

        # Slow down proportionally to error magnitude (for sharp curves)
        abs_error = abs(smoothed_error)
        speed_factor = min(1.0, abs_error / 150.0)
        linear = self.MAX_SPEED - (self.MAX_SPEED - self.MIN_SPEED) * speed_factor

        self.publish_cmd(linear, angular)
        self.dbg_error = int(smoothed_error)
        self.dbg_linear = linear
        self.dbg_angular = angular

    def build_decision_view(self):
        """Compose the visualization window with overlays and status panel."""
        if self.image is None:
            return None
        img = self.image.copy()
        h, w = img.shape[:2]
        roi_y = int(h * self.ROI_TOP_RATIO)
        image_center = w // 2

        cv2.line(img, (0, roi_y), (w, roi_y), (100, 100, 100), 1)
        cv2.line(img, (image_center, 0), (image_center, h), (0, 255, 255), 1)

        # Highlight the safety zone during turning
        if self.state == "TURN_LEFT":
            safe_left = image_center - self.YELLOW_SAFETY_DIST
            safe_right = image_center + self.YELLOW_SAFETY_DIST
            overlay = img.copy()
            cv2.rectangle(overlay, (safe_left, 0), (safe_right, h), (0, 150, 0), -1)
            cv2.addWeighted(overlay, 0.15, img, 0.85, 0, img)
            cv2.line(img, (safe_left, 0), (safe_left, h), (0, 255, 0), 1)
            cv2.line(img, (safe_right, 0), (safe_right, h), (0, 255, 0), 1)

        # Color-code the L/R markers by which color is being tracked
        if self.dbg_active_color == "WHITE":
            l_color, r_color = (255, 255, 255), (200, 200, 200)
        elif self.dbg_active_color == "RED":
            l_color, r_color = (0, 0, 255), (0, 0, 255)
        else:
            l_color, r_color = (255, 100, 0), (0, 100, 255)

        if self.dbg_left_x is not None:
            cv2.line(img, (self.dbg_left_x, roi_y), (self.dbg_left_x, h), l_color, 4)
        if self.dbg_right_x is not None:
            cv2.line(img, (self.dbg_right_x, roi_y), (self.dbg_right_x, h), r_color, 4)

        # Only show red overlay after the highway section
        if self.final_stage_unlocked and self.dbg_red_x is not None and self.dbg_red_y is not None:
            cv2.circle(img, (self.dbg_red_x, self.dbg_red_y), 25, (0, 0, 255), 3)
            cv2.putText(img, "RED", (self.dbg_red_x - 20, self.dbg_red_y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            park_y = int(h * self.RED_CY_DONE_RATIO)
            cv2.line(img, (0, park_y), (w, park_y), (0, 255, 255), 1)
            cv2.putText(img, "PARK LINE", (5, park_y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        if self.dbg_target is not None:
            tx = max(0, min(w - 1, self.dbg_target))
            cv2.line(img, (tx, roi_y), (tx, h), (0, 255, 0), 3)
            cv2.circle(img, (tx, h - 30), 14, (0, 255, 0), 3)

        # Build the status bar below the image
        status_bg = np.zeros((195, w, 3), dtype=np.uint8)

        state_color_map = {
            "LANE_YELLOW":  (0, 200, 200),
            "LANE_WHITE":   (255, 255, 255),
            "TURN_LEFT":    (255, 0, 255),
            "SEEK_RED":     (0, 100, 255),
            "PARK":         (0, 200, 0),
            "FINAL_PUSH":   (0, 255, 100),
            "DONE":         (0, 255, 0),
        }
        sc = state_color_map.get(self.state, (255, 255, 255))
        cv2.rectangle(status_bg, (0, 0), (w, 30), (sc[0] // 4, sc[1] // 4, sc[2] // 4), -1)
        cv2.putText(status_bg, ">> STATE: %s <<" % self.state,
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, sc, 2)

        cv2.putText(status_bg,
            "L=%s R=%s TGT=%s ERR=%+d  MODE=%s" % (
                str(self.dbg_left_x), str(self.dbg_right_x),
                str(self.dbg_target), self.dbg_error, self.dbg_mode),
            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        cv2.putText(status_bg, "LINEAR=%.2f m/s  ANGULAR=%+.2f rad/s" %
                    (self.dbg_linear, self.dbg_angular),
                    (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.putText(status_bg,
            "white_count=%d  red_area=%d  lost_lane=%d" %
            (self.dbg_white_count, self.dbg_red_area, self.lost_lane_count),
            (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        if self.final_stage_unlocked:
            unlock_text = "FINAL STAGE: UNLOCKED (white_frames=%d, cooldown=%d)" % (
                self.white_frame_count, self.unlock_cooldown)
            unlock_color = (0, 255, 100)
        else:
            unlock_text = "FINAL STAGE: LOCKED (white_frames=%d / need %d)" % (
                self.white_frame_count, self.MIN_WHITE_FRAMES_TO_UNLOCK)
            unlock_color = (100, 100, 255)
        cv2.putText(status_bg, unlock_text,
                    (10, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.45, unlock_color, 1)

        if self.dbg_red_x is not None and self.final_stage_unlocked:
            cv2.putText(status_bg, "RED x=%d y=%d area=%d" %
                        (self.dbg_red_x, self.dbg_red_y, self.dbg_red_area),
                        (10, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 255), 1)

        return np.vstack([img, status_bg])

    def publish_cmd(self, linear, angular):
        """Send velocity command to the robot."""
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.pub_cmd.publish(t)

    def stop_robot(self):
        """Send several zero-velocity commands on shutdown to ensure robot halts."""
        rospy.loginfo("Stopping robot...")
        for _ in range(5):
            self.publish_cmd(0.0, 0.0)
            rospy.sleep(0.05)
        cv2.destroyAllWindows()

    def run_gui(self):
        """Display debug windows and handle quit key."""
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.image is not None:
                cv2.imshow('ORIGINAL', self.image)
            if self.dbg_yellow_mask is not None:
                cv2.imshow('YELLOW MASK', self.dbg_yellow_mask)
            if self.dbg_white_mask is not None:
                cv2.imshow('WHITE MASK', self.dbg_white_mask)
            if self.dbg_red_mask is not None:
                cv2.imshow('RED MASK', self.dbg_red_mask)
            decision = self.build_decision_view()
            if decision is not None:
                cv2.imshow('DECISION', decision)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                rospy.signal_shutdown("Quit by user")
                break
            rate.sleep()


if __name__ == '__main__':
    try:
        node = LaneDriver()
        node.run_gui()
    except rospy.ROSInterruptException:
        pass