#!/usr/bin/env python3
"""Natural-language pick commands with Gemini Robotics-ER ("set of marks").

  /fr3_demo/pick_command (std_msgs/String), e.g. "gắp cái chuột"
    1. /fr3_demo/move_to_observe: arm out of the camera's view, perception refreshed
    2. latest RGB frame + /fr3_demo/perceived_objects; each object's centre is
       projected into the image and marked with its number
    3. Gemini gets the marked image, a table of what each number is (shape,
       size, colour from layer-1 geometry) and the command, and answers JSON
       {"id": <number or null>, "reason": "..."}
    4. the answer is checked (number exists, object graspable); then
       /fr3_demo/pick_target = obj_N and /fr3_demo/run_mtc_pick is called
  Progress and the outcome go to /fr3_demo/pick_status; the marked image to
  /fr3_demo/gemini/annotated and ~/.ros/fr3_gemini/last_query.png.

Gemini only chooses WHICH object; where it is comes from the depth geometry, so
its 2D answers never set a grasp pose. Anything unclear -> the robot does not move.
The API key is read from GEMINI_API_KEY or ~/.config/fr3_demo/gemini_api_key and
is never logged.
"""
import io
import json
import os
import re
import threading
import time

import numpy as np
import rclpy
from fairino_fr3_msgs.msg import PerceivedObjectArray
from PIL import Image as PILImage, ImageDraw, ImageFont
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

PROMPT = """You help a robot arm decide which object on the table to pick up.
The image shows the table seen by the robot's camera. Every object the robot's
depth sensor found carries a red number. The numbers are:
{table}
The user's command (it may be in Vietnamese): "{command}"
Which numbered object does the user want picked? Use the image (appearance,
colour, position such as left/right/near/far) and the table. Answer ONLY with
JSON: {{"id": <number>, "reason": "<short reason>"}}. If no numbered object
matches, or several match equally well, answer {{"id": null, "reason": "<why>"}}."""


def read_api_key():
    key = os.environ.get("GEMINI_API_KEY", "").strip()
    path = os.path.expanduser("~/.config/fr3_demo/gemini_api_key")
    if not key and os.path.exists(path):
        with open(path) as stream:
            key = stream.read().strip()
    return key


class GeminiTargetSelector(Node):
    def __init__(self):
        super().__init__("gemini_target_selector")
        p = self.declare_parameter
        self.model = p("model", "gemini-robotics-er-2-preview").value
        self.auto_pick = p("auto_pick", True).value
        self.move_to_observe = p("move_to_observe", True).value
        self.timeout = p("request_timeout", 60.0).value
        self.save_dir = os.path.expanduser(p("save_dir", "~/.ros/fr3_gemini").value)
        os.makedirs(self.save_dir, exist_ok=True)

        self.key = read_api_key()
        self.client = None
        if self.key:
            from google import genai
            self.client = genai.Client(api_key=self.key)
        else:
            self.get_logger().error(
                "no Gemini API key (GEMINI_API_KEY or ~/.config/fr3_demo/gemini_api_key)")

        self.rgb = None
        self.info = None
        self.objects = None
        self.busy = threading.Lock()
        group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Sim defaults (Gazebo bridge); the real Astra uses /camera/color/... (config *_real.yaml).
        rgb_topic = self.declare_parameter("rgb_topic", "/oak/rgb/image_raw").value
        info_topic = self.declare_parameter("camera_info_topic", "/oak/rgb/camera_info").value
        self.create_subscription(Image, rgb_topic,
                                 lambda m: setattr(self, "rgb", m), 1, callback_group=group)
        self.create_subscription(CameraInfo, info_topic,
                                 lambda m: setattr(self, "info", m), 1, callback_group=group)
        self.create_subscription(PerceivedObjectArray, "/fr3_demo/perceived_objects",
                                 lambda m: setattr(self, "objects", m), 1, callback_group=group)
        self.create_subscription(String, "/fr3_demo/pick_command", self.on_command, 10,
                                 callback_group=group)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(String, "/fr3_demo/pick_target", latched)
        self.status_pub = self.create_publisher(String, "/fr3_demo/pick_status", 10)
        self.annotated_pub = self.create_publisher(Image, "/fr3_demo/gemini/annotated", 1)
        self.observe_client = self.create_client(Trigger, "/fr3_demo/move_to_observe",
                                                 callback_group=group)
        self.pick_client = self.create_client(Trigger, "/fr3_demo/run_mtc_pick",
                                              callback_group=group)
        self.get_logger().info(f"Ready: send commands on /fr3_demo/pick_command (model {self.model})")

    # ----------------------------------------------------------------- helpers
    def status(self, text, error=False):
        # rclpy (humble) refuses to change the severity used at one call site,
        # so error and info need separate logging lines.
        if error:
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def call(self, client, timeout):
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def project(self, objects, frame_id):
        """Image pixel of each object's centre (objects behind the camera or
        outside the image are left out)."""
        K = np.array(self.info.k).reshape(3, 3)
        try:
            tf = self.tf_buffer.lookup_transform(
                frame_id, objects.header.frame_id, rclpy.time.Time()).transform
        except TransformException as error:
            self.status(f"camera TF unavailable: {error}", error=True)
            return {}
        R = Rotation.from_quat([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
        t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        pixels = {}
        for obj in objects.objects:
            pos = obj.pose.pose.position
            camera = R.apply([pos.x, pos.y, pos.z]) + t
            if camera[2] <= 0.05:
                continue
            u, v, _ = K @ (camera / camera[2])
            if 0 <= u < self.info.width and 0 <= v < self.info.height:
                pixels[obj.id] = (float(u), float(v))
        return pixels

    def annotate(self, image, numbered, pixels):
        draw = ImageDraw.Draw(image)
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
        except OSError:
            font = ImageFont.load_default()
        for number, obj in numbered.items():
            u, v = pixels[obj.id]
            draw.ellipse([u - 6, v - 6, u + 6, v + 6], fill=(255, 0, 0))
            box = [u + 8, v - 28, u + 8 + 18 * len(str(number)) + 10, v - 2]
            draw.rectangle(box, fill=(255, 255, 255), outline=(255, 0, 0), width=2)
            draw.text((box[0] + 5, box[1] + 1), str(number), fill=(255, 0, 0), font=font)
        return image

    @staticmethod
    def describe(obj):
        dims = ", ".join(f"{d * 1000:.0f}" for d in obj.dimensions)
        size = (f"box {dims} mm (L x W x H)" if obj.shape == "box"
                else f"cylinder height/radius {dims} mm")
        return (f"{obj.color} {size}, centre at x={obj.pose.pose.position.x:.2f} "
                f"y={obj.pose.pose.position.y:.2f} m (world), "
                f"{'graspable' if obj.graspable else 'too wide for the gripper'}")

    # ------------------------------------------------------------------- flow
    def on_command(self, msg):
        if not self.busy.acquire(blocking=False):
            self.status("busy with a previous command; ignored: " + msg.data, error=True)
            return
        try:
            self.process_command(msg.data.strip())
        except Exception as error:  # report, never crash the node
            text = str(error).replace(self.key, "<KEY>") if self.key else str(error)
            self.status(f"command failed: {type(error).__name__}: {text[:300]}", error=True)
        finally:
            self.busy.release()

    def process_command(self, command):
        if self.client is None:
            self.status("no Gemini API key; cannot interpret commands", error=True)
            return
        self.status(f"command: '{command}'")
        if self.move_to_observe:
            self.status("moving to the observe pose and refreshing perception ...")
            result = self.call(self.observe_client, 90.0)
            if result is None or not result.success:
                self.status("could not reach the observe pose: "
                            + (result.message if result else "no response"), error=True)
                return
        if self.rgb is None or self.info is None or self.objects is None:
            self.status("no camera image / camera info / perceived objects yet", error=True)
            return
        rgb_msg, objects = self.rgb, self.objects
        pixels = self.project(objects, rgb_msg.header.frame_id)
        numbered = {i + 1: obj for i, obj in enumerate(
            o for o in objects.objects if o.id in pixels)}
        if not numbered:
            self.status("no objects visible to choose from", error=True)
            return

        rgb = np.frombuffer(rgb_msg.data, np.uint8).reshape(
            rgb_msg.height, rgb_msg.step)[:, : rgb_msg.width * 3].reshape(
            rgb_msg.height, rgb_msg.width, 3)
        if rgb_msg.encoding == "bgr8":  # the OAK driver published BGR (the Astra sends rgb8)
            rgb = rgb[:, :, ::-1]
        image = self.annotate(PILImage.fromarray(rgb.copy()), numbered, pixels)
        image.save(os.path.join(self.save_dir, "last_query.png"))
        annotated = Image(header=rgb_msg.header, height=image.height, width=image.width,
                          encoding="rgb8", step=image.width * 3, data=image.tobytes())
        self.annotated_pub.publish(annotated)

        table = "\n".join(f"  {n}: {self.describe(o)}" for n, o in numbered.items())
        answer, seconds = self.ask_gemini(image, PROMPT.format(table=table, command=command))
        self.get_logger().info(f"Gemini ({seconds:.1f} s): {answer}")
        choice = self.parse(answer)
        if choice is None:
            self.status(f"could not read Gemini's answer: {answer[:200]}", error=True)
            return
        number, reason = choice
        if number is None:
            self.status(f"no unique object for '{command}': {reason}", error=True)
            return
        if number not in numbered:
            self.status(f"Gemini chose number {number}, which is not on the image", error=True)
            return
        obj = numbered[number]
        self.status(f"'{command}' -> #{number} = {obj.id} ({obj.color} {obj.shape}): {reason}")
        if not obj.graspable:
            self.status(f"{obj.id} is too wide for the gripper; not picking", error=True)
            return
        self.target_pub.publish(String(data=obj.id))
        if not self.auto_pick:
            self.status(f"target set to {obj.id}; call /fr3_demo/run_mtc_pick to pick it")
            return
        time.sleep(0.5)  # let the MTC node receive the target
        self.status(f"picking {obj.id} ...")
        result = self.call(self.pick_client, 300.0)
        if result is None:
            self.status("pick: no response from /fr3_demo/run_mtc_pick", error=True)
        else:
            self.status(("pick done: " if result.success else "pick failed: ") + result.message,
                        error=not result.success)

    def ask_gemini(self, image, prompt):
        from google.genai import types
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        started = time.time()
        response = self.client.models.generate_content(
            model=self.model,
            contents=[types.Part.from_bytes(data=buffer.getvalue(), mime_type="image/png"), prompt],
            config=types.GenerateContentConfig(
                http_options=types.HttpOptions(timeout=int(self.timeout * 1000))),
        )
        return response.text or "", time.time() - started

    @staticmethod
    def parse(text):
        match = re.search(r"\{.*\}", text, re.S)
        if not match:
            return None
        try:
            data = json.loads(match.group(0))
        except json.JSONDecodeError:
            return None
        number = data.get("id")
        if number is not None:
            try:
                number = int(number)
            except (TypeError, ValueError):
                return None
        return number, str(data.get("reason", ""))


def main():
    rclpy.init()
    node = GeminiTargetSelector()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
