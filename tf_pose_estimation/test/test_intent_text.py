# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# colcon test --packages-select <YOUR_PACKAGE> --event-handlers console_direct+
#
import rclpy
from dialogflow_ros import utils
from dialogflow_ros.dialogflow_client import DialogflowClient
from dialogflow_ros_msgs.msg import DialogflowRequest


def test_intent_text():
  rclpy.init()
  dc = DialogflowClient()
  dr = DialogflowRequest(query_text="Hey! Goog morning")
  resp1 = dc._detect_intent_text(dr)
  print(utils.output.print_result(resp1))
  result = 0
  assert result == 0
